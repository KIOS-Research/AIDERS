"""StreamCV -- zero-disk-I/O drone video detection pipeline.

FastAPI application that pulls live RTMP frames and runs inference,
annotates in-memory, and streams the result as MJPEG -- all without ever
touching the filesystem for frame data. Three detection types are supported,
selected per-session via ``StartRequest.detection_type``:

* ``WALDO_DETECTOR`` -- general YOLOv7 object detector (onnxruntime),
  tracked with ``supervision.ByteTrack`` and annotated with boxes/labels/traces.
* ``CROWD_LOCALIZATION`` -- HRNet+ESPCN density-map person localization
  (PyTorch), untracked (points aren't tracked across frames), annotated
  with dots only.
* ``DISASTER_CLASSIFICATION`` -- EfficientNet-B0 whole-frame classifier
  (PyTorch) over Earthquake/Fire/Flood/Normal, untracked and unboxed (no
  spatial component at all), annotated with a text HUD of the three
  disaster-class probabilities.

Inference runs in a standalone background thread (`_inference_loop`)
that is started by `/api/streamcv/start/{drone_id}` and keeps running
independently of whether any MJPEG client is connected. The MJPEG
endpoint (`_video_generator`) is a pure reader: it only reads the
latest annotated frame from shared memory and performs zero inference.
This mirrors the legacy CVN/LSC architecture where detection compute
(DETECT) and video display (DET. LIVE) were fully decoupled.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Generator

import cv2
import numpy as np
import supervision as sv
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from streamcv.core.stream_reader import ThreadedCamera
from streamcv.db import repository as db
from streamcv.ml.crowd_detector import CrowdDetector
from streamcv.ml.detectors import CLASS_NAMES, WaldoDetector
from streamcv.ml.disaster_detector import DisasterDetector
from streamcv.utils.geo_utils import (
    DJI_MAVIC_FOV,
    apply_gimbal_correction,
    pixel_to_gps,
)

# Detection type dispatch keys -- exact string values from Django's
# Detection.DetectionTypeStrChoices, forwarded unchanged from the frontend's
# DETECTION_TYPES constants through DetectionStartOrStopAPIView and
# postDetectionStartToCv into StartRequest.detection_type.
WALDO_DETECTOR = "WALDO_DETECTOR"
CROWD_LOCALIZATION = "CROWD_LOCALIZATION"
DISASTER_CLASSIFICATION = "DISASTER_CLASSIFICATION"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-24s  %(levelname)-5s  %(message)s",
)
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------
# FastAPI app
# -----------------------------------------------------------------------

app = FastAPI(title="StreamCV", version="0.1.0")

# -----------------------------------------------------------------------
# Shared singletons (created once at startup)
# -----------------------------------------------------------------------

detector: WaldoDetector | None = None
crowd_detector: CrowdDetector | None = None
disaster_detector: DisasterDetector | None = None


@app.on_event("startup")
def _startup() -> None:
    global detector, crowd_detector, disaster_detector

    db.init_pools(num_pools=1, prefix="streamcv")

    # All three models are loaded eagerly at startup (not lazily on first use
    # of a given detection_type) so that switching detection types mid-operation
    # never incurs cold-start latency.
    detector = WaldoDetector()
    crowd_detector = CrowdDetector()
    disaster_detector = DisasterDetector()

    logger.info("StreamCV startup complete")


# -----------------------------------------------------------------------
# Per-drone state
# -----------------------------------------------------------------------

@dataclass
class DroneState:
    camera: ThreadedCamera
    detector_kind: str
    session_id: int
    operation_id: int
    drone_id: int
    user_id: int
    # WALDO_DETECTOR path: box/label/trace annotation + tracking.
    tracker: sv.ByteTrack | None = None
    box_annotator: sv.BoxAnnotator | None = None
    label_annotator: sv.LabelAnnotator | None = None
    trace_annotator: sv.TraceAnnotator | None = None
    # CROWD_LOCALIZATION path: no tracker, dot annotation only.
    dot_annotator: sv.DotAnnotator | None = None
    latest_annotated: np.ndarray | None = None
    annotated_lock: threading.Lock = field(default_factory=threading.Lock)
    inference_thread: threading.Thread | None = None
    video_writer: cv2.VideoWriter | None = None
    video_path: str = ""


_drones: dict[int, DroneState] = {}

# Per-drone locks serializing start/stop so that two overlapping requests
# for the same drone_id (e.g. a rapid double-toggle of DETECT) can never
# race on `_drones[drone_id]`. Without this, one request's teardown could
# pop/replace the state another concurrent request just installed, leaving
# an orphaned camera/video_writer/inference thread and a transient window
# where `drone_id not in _drones` even though a session was just started
# (observed as intermittent 404s from `/api/streamcv/live/{drone_id}`).
_locks_guard = threading.Lock()
_drone_locks: dict[int, threading.Lock] = {}


def _get_drone_lock(drone_id: int) -> threading.Lock:
    """Return the (lazily created) lock guarding `_drones[drone_id]`."""
    with _locks_guard:
        lock = _drone_locks.get(drone_id)
        if lock is None:
            lock = threading.Lock()
            _drone_locks[drone_id] = lock
        return lock


# -----------------------------------------------------------------------
# Request / response models
# -----------------------------------------------------------------------


class StartRequest(BaseModel):
    rtmp_url: str
    operation_id: int
    user_id: int
    detection_type: str = WALDO_DETECTOR


# -----------------------------------------------------------------------
# Endpoints
# -----------------------------------------------------------------------


@app.post("/api/streamcv/start/{drone_id}")
def start_stream(drone_id: int, req: StartRequest):
    """Spin up camera + tracker for *drone_id*, create a DB session, and
    start the background inference loop (independent of any MJPEG
    client connecting)."""

    with _get_drone_lock(drone_id):
        # Fully tear down and join any pre-existing state for this drone_id
        # before creating a new one, so no two inference threads ever run
        # concurrently for the same drone_id, and no overlapping start/stop
        # request can pop/replace a state another request just installed.
        existing_state = _drones.pop(drone_id, None)
        if existing_state is not None:
            _teardown_drone(drone_id, existing_state)

        session_id = db.deactivate_sessions_and_create_new(
            drone_id=drone_id,
            operation_id=req.operation_id,
            user_id=req.user_id,
        )

        detector_kind = req.detection_type
        camera = ThreadedCamera(req.rtmp_url)

        tracker = None
        box_annotator = None
        label_annotator = None
        trace_annotator = None
        dot_annotator = None
        if detector_kind == CROWD_LOCALIZATION:
            # No tracking for crowd localization -- points aren't tracked
            # across frames, only visualized and stored per-frame.
            dot_annotator = sv.DotAnnotator()
            model_w, model_h = crowd_detector.input_size
        elif detector_kind == DISASTER_CLASSIFICATION:
            # No tracking, no box/dot annotation -- disaster classification
            # is a whole-frame label with no per-object geometry to draw.
            model_w, model_h = disaster_detector.input_size
        else:
            tracker = sv.ByteTrack()
            box_annotator = sv.BoxAnnotator(thickness=2)
            label_annotator = sv.LabelAnnotator(text_scale=0.4, text_padding=4)
            trace_annotator = sv.TraceAnnotator(thickness=2, trace_length=40)
            model_w, model_h = detector.input_size

        # --- Video recording setup ---
        video_dir = "/media/live"
        os.makedirs(video_dir, exist_ok=True)
        video_filename = f"detector_{drone_id}_session{session_id}.mp4"
        video_path = os.path.join(video_dir, video_filename)
        codec_priority_raw = os.getenv("STREAMCV_VIDEO_CODEC_PRIORITY", "mp4v,avc1")
        codec_priority = [c.strip() for c in codec_priority_raw.split(",") if c.strip()]
        video_writer = None
        selected_codec = None

        for codec in codec_priority:
            writer = cv2.VideoWriter(
                video_path,
                cv2.VideoWriter_fourcc(*codec),
                15.0,
                (model_w, model_h),
            )
            if writer.isOpened():
                video_writer = writer
                selected_codec = codec
                break
            writer.release()

        if video_writer is None:
            logger.warning(
                "No configured video codec is available (tried=%s); recording disabled for this session",
                codec_priority,
            )
        else:
            logger.info("Video writer opened with codec=%s", selected_codec)

        state = DroneState(
            camera=camera,
            detector_kind=detector_kind,
            tracker=tracker,
            box_annotator=box_annotator,
            label_annotator=label_annotator,
            trace_annotator=trace_annotator,
            dot_annotator=dot_annotator,
            session_id=session_id,
            operation_id=req.operation_id,
            drone_id=drone_id,
            user_id=req.user_id,
            video_writer=video_writer,
            video_path=video_path,
        )
        _drones[drone_id] = state

        # --- Start the inference thread ---
        t = threading.Thread(target=_inference_loop, args=(drone_id,), daemon=True)
        t.start()
        state.inference_thread = t

        # The legacy CVN code hardcoded "YOLO" as the detection_model label
        # even for the (HRNet-based) crowd detector and the (EfficientNet-
        # based) disaster classifier -- an inaccurate label carried over
        # from copy-pasted scaffolding. Record the real model actually
        # running for each detection_type here instead.
        if detector_kind == CROWD_LOCALIZATION:
            model_label = "HRNET_ESPCN"
        elif detector_kind == DISASTER_CLASSIFICATION:
            model_label = "EFFICIENTNET_B0"
        else:
            model_label = "YOLO"
        db.update_detection_entry(
            drone_id, "DETECTION_CONNECTED", req.detection_type, model_label,
        )

        logger.info(
            "Stream started  drone=%d  session=%d  detector_kind=%s  rtmp=%s  video=%s",
            drone_id, session_id, detector_kind, req.rtmp_url, video_path,
        )
        return {"status": "started", "drone_id": drone_id, "session_id": session_id}


@app.post("/api/streamcv/stop/{drone_id}")
def stop_stream(drone_id: int):
    """Tear down camera + tracker for *drone_id* and wait for the
    inference thread to exit."""
    with _get_drone_lock(drone_id):
        state = _drones.pop(drone_id, None)
        if state is not None:
            _teardown_drone(drone_id, state)

    db.update_detection_entry(
        drone_id, "DETECTION_DISCONNECTED", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL",
    )

    logger.info("Stream stopped  drone=%d", drone_id)
    return {"status": "stopped", "drone_id": drone_id}


def _teardown_drone(drone_id: int, state: DroneState) -> None:
    """Fully tear down a drone's camera, inference thread, and video writer.

    Safe to call even if the inference thread has already exited. This is
    the single place that releases the camera, joins the inference thread,
    releases the video writer, and persists the video path + ends the
    session -- used by both `stop_stream` and `start_stream` (when
    replacing a pre-existing state for the same drone_id).
    """
    state.camera.release()

    # Wait for the inference loop to notice it's been replaced/removed and exit.
    if state.inference_thread is not None:
        state.inference_thread.join(timeout=10.0)
        if state.inference_thread.is_alive():
            logger.warning(
                "Inference thread for drone %d did not exit within timeout",
                drone_id,
            )

    # The inference loop releases its own video_writer on exit, but guard
    # against it still being open here just in case join() timed out.
    if state.video_writer is not None:
        state.video_writer.release()
        state.video_writer = None

    if state.video_path:
        db.submit(db.save_video_path, state.session_id, state.video_path)

    db.submit(db.end_session, state.session_id)


@app.get("/api/streamcv/live/{drone_id}")
def live_stream(drone_id: int):
    """MJPEG stream of annotated detection frames."""
    if drone_id not in _drones:
        raise HTTPException(404, f"No active stream for drone {drone_id}")
    return StreamingResponse(
        _video_generator(drone_id),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# -----------------------------------------------------------------------
# Background inference loop
# -----------------------------------------------------------------------

_JPEG_QUALITY = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
_DB_WRITE_INTERVAL = 15  # Write to DB every N frames (~1 write/sec at 15fps)


def _inference_loop(drone_id: int) -> None:
    """Standalone inference loop running in a background thread.

    Produces annotated frames, records video, and schedules DB writes
    independently of any MJPEG client connections.
    """
    state = _drones.get(drone_id)
    if state is None:
        return

    is_crowd = state.detector_kind == CROWD_LOCALIZATION
    is_disaster = state.detector_kind == DISASTER_CLASSIFICATION

    cam = state.camera
    trk = state.tracker
    box_ann = state.box_annotator
    label_ann = state.label_annotator
    trace_ann = state.trace_annotator
    dot_ann = state.dot_annotator
    frame_count = 0

    while _drones.get(drone_id) is state:
        frame = cam.read()
        if frame is None:
            time.sleep(0.03)
            continue

        if is_crowd:
            # --- Detect (no tracking -- crowd points aren't tracked) ---
            detections, padded = crowd_detector.predict(frame)
            count = len(detections)

            # --- Annotate (in-memory, on the padded frame) -------------
            annotated = dot_ann.annotate(padded.copy(), detections)
            annotated = _draw_people_count(annotated, count)

            with state.annotated_lock:
                state.latest_annotated = annotated

            if state.video_writer is not None:
                state.video_writer.write(annotated)

            frame_count += 1
            _schedule_crowd_db_write(detections, count, state, frame_count)
            continue

        if is_disaster:
            # --- Classify (whole frame -- no tracking, no per-object
            # geometry). `frame` is passed in raw/unresized: this model
            # does its own independent resize to 224x224 internally,
            # regardless of any other detector's canvas convention. ---
            probabilities = disaster_detector.predict(frame)

            # --- Annotate: resize to the fixed display/recording canvas.
            # This resize exists only because cv2.VideoWriter needs a size
            # decided in advance at stream-start time -- it has no bearing
            # on the classification itself, which already happened above
            # on the untouched frame. ---
            display_w, display_h = disaster_detector.input_size
            annotated = cv2.resize(frame, (display_w, display_h))
            annotated = _draw_disaster_probabilities(annotated, probabilities)

            with state.annotated_lock:
                state.latest_annotated = annotated

            if state.video_writer is not None:
                state.video_writer.write(annotated)

            frame_count += 1
            _schedule_disaster_db_write(probabilities, state, frame_count)
            continue

        # --- Detect ------------------------------------------------
        detections, padded = detector.predict(frame)

        # --- Track -------------------------------------------------
        detections = trk.update_with_detections(detections)

        # --- Annotate (in-memory, on the padded frame) -------------
        labels = _build_labels(detections)
        annotated = padded.copy()
        annotated = box_ann.annotate(annotated, detections)
        annotated = label_ann.annotate(annotated, detections, labels)
        annotated = trace_ann.annotate(annotated, detections)

        # --- Store latest frame for MJPEG consumers -----------------
        with state.annotated_lock:
            state.latest_annotated = annotated

        # --- Record to video file ------------------------------------
        if state.video_writer is not None:
            state.video_writer.write(annotated)

        # --- Sampled DB writes ----------------------------------------
        frame_count += 1
        _schedule_db_writes(detections, state, frame_count)

    # Cleanup: release video writer when loop exits
    if state.video_writer is not None:
        state.video_writer.release()
        state.video_writer = None
        logger.info("Video writer released for drone %d: %s", drone_id, state.video_path)


# -----------------------------------------------------------------------
# Pipeline generator (pure reader, zero inference)
# -----------------------------------------------------------------------


def _video_generator(drone_id: int) -> Generator[bytes, None, None]:
    """Yield MJPEG frames by reading the latest annotated frame from shared
    memory. Does NOT perform any inference -- that runs in `_inference_loop`.
    """
    state = _drones.get(drone_id)
    if state is None:
        return

    while _drones.get(drone_id) is state:
        with state.annotated_lock:
            frame = state.latest_annotated

        if frame is None:
            time.sleep(0.03)
            continue

        ok, jpeg = cv2.imencode(".jpg", frame, _JPEG_QUALITY)
        if not ok:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
        )
        time.sleep(0.033)  # Cap at ~30fps to prevent CPU spin


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------


def _build_labels(detections: sv.Detections) -> list[str]:
    labels: list[str] = []
    for i in range(len(detections)):
        cls = CLASS_NAMES[detections.class_id[i]]
        conf = detections.confidence[i]
        tid = detections.tracker_id[i] if detections.tracker_id is not None else ""
        labels.append(f"#{tid} {cls} {conf:.2f}")
    return labels


# HUD-style people-count overlay for the crowd localization path. A total
# count isn't a per-detection label the way sv.LabelAnnotator is designed
# for, so this uses supervision's standalone sv.draw_text primitive instead
# -- the idiomatic supervision tool for arbitrary scene-level text, not tied
# to any single detection.
_COUNT_TEXT_SCALE = 0.7
_COUNT_TEXT_THICKNESS = 2
_COUNT_TEXT_PADDING = 10
_COUNT_TEXT_MARGIN = 15  # distance from the frame's top-right corner


def _draw_people_count(frame: np.ndarray, count: int) -> np.ndarray:
    text = f"People Detected: {count}"
    text_width, text_height = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, _COUNT_TEXT_SCALE, _COUNT_TEXT_THICKNESS
    )[0]
    frame_width = frame.shape[1]
    # Pre-measuring the text lets the padded background box's top-right
    # corner land at a fixed (frame_width - _COUNT_TEXT_MARGIN, _COUNT_TEXT_MARGIN)
    # regardless of how many digits `count` has -- sv.draw_text anchors
    # from the text's center, not its corner.
    anchor = sv.Point(
        x=frame_width - _COUNT_TEXT_MARGIN - _COUNT_TEXT_PADDING - text_width // 2,
        y=_COUNT_TEXT_MARGIN + _COUNT_TEXT_PADDING + text_height // 2,
    )
    return sv.draw_text(
        scene=frame,
        text=text,
        text_anchor=anchor,
        text_color=sv.Color.WHITE,
        text_scale=_COUNT_TEXT_SCALE,
        text_thickness=_COUNT_TEXT_THICKNESS,
        text_padding=_COUNT_TEXT_PADDING,
        background_color=sv.Color.BLACK,
    )


# HUD-style probability overlay for the disaster classification path,
# drawn every frame regardless of whether the DB write threshold fires --
# this text is purely informational, unlike the gated DB row below.
# Anchored top-right using the same fixed-corner, pre-measured-text-width
# technique as _draw_people_count (top-right for that HUD too) -- the two
# can never render on the same frame today since CROWD_LOCALIZATION and
# DISASTER_CLASSIFICATION are mutually-exclusive branches on a single
# session's fixed detector_kind, so sharing the same corner is not a
# conflict, just a coincidence of both defaulting to it independently.
_DISASTER_TEXT_SCALE = 0.7
_DISASTER_TEXT_THICKNESS = 2
_DISASTER_TEXT_PADDING = 10
_DISASTER_TEXT_MARGIN = 15  # distance from the frame's top-right corner
_DISASTER_TEXT_ORDER = ("Fire", "Flood", "Earthquake")
_DISASTER_TEXT_SEPARATOR = " | "

# Semantically-distinct color per class, used only for whichever class is
# currently leading (highest probability) -- the other two stay white.
# sv.Color has no "orange"/"amber" constants, so RED/BLUE/YELLOW are used
# as the closest built-ins that still keep all three visually distinct.
_DISASTER_CLASS_COLORS: dict[str, "sv.Color"] = {
    "Fire": sv.Color.RED,
    "Flood": sv.Color.BLUE,
    "Earthquake": sv.Color.YELLOW,
}


def _draw_disaster_probabilities(frame: np.ndarray, probabilities: dict[str, float]) -> np.ndarray:
    # sv.draw_text takes one text_color for the whole string, so per-class
    # coloring needs one cv2.putText call per segment instead of a single
    # sv.draw_text call -- verified there's no per-substring coloring
    # option on the installed supervision version (0.29.1)'s draw_text.
    leading_class = max(probabilities, key=probabilities.get) if probabilities else None
    segments = [f"{name}: {probabilities.get(name, 0.0):.0f}%" for name in _DISASTER_TEXT_ORDER]

    # Segment widths are measured individually and summed, rather than
    # measuring the full concatenated string in one cv2.getTextSize call --
    # verified empirically that the two don't exactly agree (off by a few
    # px for this font/scale), so using the summed total as the single
    # source of truth for both the background box below and the chained
    # per-segment x-offsets keeps them mutually consistent with each other,
    # which matters more here than matching what a single-string
    # measurement would have produced.
    segment_widths = [
        cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, _DISASTER_TEXT_SCALE, _DISASTER_TEXT_THICKNESS)[0][0]
        for s in segments
    ]
    separator_width = cv2.getTextSize(
        _DISASTER_TEXT_SEPARATOR, cv2.FONT_HERSHEY_SIMPLEX, _DISASTER_TEXT_SCALE, _DISASTER_TEXT_THICKNESS
    )[0][0]
    text_height = cv2.getTextSize(
        segments[0], cv2.FONT_HERSHEY_SIMPLEX, _DISASTER_TEXT_SCALE, _DISASTER_TEXT_THICKNESS
    )[0][1]
    text_width = sum(segment_widths) + separator_width * (len(_DISASTER_TEXT_ORDER) - 1)

    frame_width = frame.shape[1]
    # Same fixed-corner anchor math as before -- computed from the combined
    # label's total width, so the whole three-part label's right edge lands
    # at the same place regardless of which class (and color) is leading.
    anchor_x = frame_width - _DISASTER_TEXT_MARGIN - _DISASTER_TEXT_PADDING - text_width // 2
    anchor_y = _DISASTER_TEXT_MARGIN + _DISASTER_TEXT_PADDING + text_height // 2

    # One shared background box behind the whole label, replicating
    # sv.draw_text's own anchor-centered-then-padded Rect geometry (see
    # supervision.draw.utils.draw_text) since that all-in-one call can't
    # be used here -- this keeps the same solid-background look as before.
    background_rect = sv.Rect(
        x=anchor_x - text_width // 2,
        y=anchor_y - text_height // 2,
        width=text_width,
        height=text_height,
    ).pad(_DISASTER_TEXT_PADDING)
    frame = sv.draw_filled_rectangle(scene=frame, rect=background_rect, color=sv.Color.BLACK)

    x = anchor_x - text_width // 2
    baseline_y = anchor_y + text_height // 2
    for i, class_name in enumerate(_DISASTER_TEXT_ORDER):
        color = _DISASTER_CLASS_COLORS[class_name] if class_name == leading_class else sv.Color.WHITE
        cv2.putText(
            frame, segments[i], (x, baseline_y), cv2.FONT_HERSHEY_SIMPLEX,
            _DISASTER_TEXT_SCALE, color.as_bgr(), _DISASTER_TEXT_THICKNESS, cv2.LINE_AA,
        )
        x += segment_widths[i]
        if i < len(_DISASTER_TEXT_ORDER) - 1:
            cv2.putText(
                frame, _DISASTER_TEXT_SEPARATOR, (x, baseline_y), cv2.FONT_HERSHEY_SIMPLEX,
                _DISASTER_TEXT_SCALE, sv.Color.WHITE.as_bgr(), _DISASTER_TEXT_THICKNESS, cv2.LINE_AA,
            )
            x += separator_width

    return frame


def _schedule_db_writes(detections: sv.Detections, state: DroneState, frame_count: int) -> None:
    """Offload DB writes to the thread pool, sampled at `_DB_WRITE_INTERVAL`."""
    if len(detections) == 0:
        return
    if frame_count % _DB_WRITE_INTERVAL != 0:
        return

    drone_id = state.drone_id
    session_id = state.session_id
    operation_id = state.operation_id
    model_w, model_h = detector.input_size

    xyxy = detections.xyxy
    class_ids = detections.class_id
    confidences = detections.confidence
    tracker_ids = (
        detections.tracker_id
        if detections.tracker_id is not None
        else np.zeros(len(detections), dtype=int)
    )

    centers = np.column_stack([
        (xyxy[:, 0] + xyxy[:, 2]) / 2,
        (xyxy[:, 1] + xyxy[:, 3]) / 2,
    ])

    def _write() -> None:
        telemetry = db.get_drone_telemetry(drone_id)
        if telemetry is None:
            return

        frame_id = db.save_frame(session_id, drone_id)
        if frame_id is None:
            return

        pitch = apply_gimbal_correction(telemetry[4])
        drone_info = (
            telemetry[0],  # lat
            telemetry[1],  # lon
            telemetry[2],  # alt
            telemetry[3],  # heading
            pitch,
        )

        rows: list[tuple] = []
        for i in range(len(centers)):
            cx, cy = float(centers[i, 0]), float(centers[i, 1])
            lat, lon = pixel_to_gps(
                (cx, cy),
                (model_w, model_h),
                DJI_MAVIC_FOV,
                drone_info,
            )
            label = CLASS_NAMES[int(class_ids[i])]
            track_id = int(tracker_ids[i])
            bbox = json.dumps({
                "x1": float(xyxy[i, 0]), "y1": float(xyxy[i, 1]),
                "x2": float(xyxy[i, 2]), "y2": float(xyxy[i, 3]),
            })
            conf = float(confidences[i])
            rows.append((lat, lon, label, track_id, 0.0, bbox, conf))

        db.save_detected_objects_batch(
            rows, operation_id, drone_id, session_id, frame_id,
        )

    db.submit(_write)


def _schedule_crowd_db_write(
    detections: sv.Detections, count: int, state: DroneState, frame_count: int
) -> None:
    """Offload the crowd-localization DB write, sampled at `_DB_WRITE_INTERVAL`.

    Unlike `_schedule_db_writes`, this does not assume track_id/label/
    bounding_boxes/confidence per-object semantics -- it writes one
    aggregate row (all point coordinates + count) per sampled frame.

    `count` is the caller's `len(detections)` (already computed once for the
    HUD overlay) -- passed in rather than recomputed here.
    """
    if count == 0:
        return
    if frame_count % _DB_WRITE_INTERVAL != 0:
        return

    drone_id = state.drone_id
    session_id = state.session_id
    operation_id = state.operation_id
    model_w, model_h = crowd_detector.input_size

    xyxy = detections.xyxy
    centers = np.column_stack([
        (xyxy[:, 0] + xyxy[:, 2]) / 2,
        (xyxy[:, 1] + xyxy[:, 3]) / 2,
    ])

    def _write() -> None:
        telemetry = db.get_drone_telemetry(drone_id)
        if telemetry is None:
            return

        frame_id = db.save_frame(session_id, drone_id)
        if frame_id is None:
            return

        pitch = apply_gimbal_correction(telemetry[4])
        drone_info = (
            telemetry[0],  # lat
            telemetry[1],  # lon
            telemetry[2],  # alt
            telemetry[3],  # heading
            pitch,
        )

        coordinates: list[list[float]] = []
        for i in range(len(centers)):
            cx, cy = float(centers[i, 0]), float(centers[i, 1])
            lat, lon = pixel_to_gps(
                (cx, cy),
                (model_w, model_h),
                DJI_MAVIC_FOV,
                drone_info,
            )
            coordinates.append([lat, lon])

        db.save_crowd_localization_result(
            session_id, operation_id, drone_id, frame_id,
            json.dumps(coordinates), count,
        )

    db.submit(_write)


# Legacy threshold: only persist a row when the frame's strongest disaster
# class exceeds this confidence -- preserved intentionally (unlike
# CROWD_LOCALIZATION's unconditional per-interval write) to avoid flooding
# the table with "Normal" frames.
_DISASTER_ALERT_THRESHOLD = 50.0


def _schedule_disaster_db_write(
    probabilities: dict[str, float], state: DroneState, frame_count: int
) -> None:
    """Offload the disaster-classification DB write, sampled at
    `_DB_WRITE_INTERVAL` like the other detection types.

    Unlike `_schedule_db_writes`/`_schedule_crowd_db_write`, a sampled tick
    alone does not guarantee a write: the interval controls *when* we check,
    `_DISASTER_ALERT_THRESHOLD` controls *whether* we actually write --
    matching legacy's ``max(result_probabilities.values()) > 50`` gate.
    """
    if frame_count % _DB_WRITE_INTERVAL != 0:
        return
    if not probabilities or max(probabilities.values()) <= _DISASTER_ALERT_THRESHOLD:
        return

    drone_id = state.drone_id
    session_id = state.session_id
    operation_id = state.operation_id

    def _write() -> None:
        telemetry = db.get_drone_telemetry(drone_id)
        if telemetry is None:
            return

        frame_id = db.save_frame(session_id, drone_id)
        if frame_id is None:
            return

        db.save_detected_disaster(
            session_id, operation_id, drone_id, frame_id,
            telemetry[0], telemetry[1],  # lat, lon
            probabilities["Earthquake"], probabilities["Fire"], probabilities["Flood"],
        )

    db.submit(_write)
