from types import SimpleNamespace
from enum import Enum
import logging
import numpy as np
import cv2
import torch
from .models.yolo_opencv import Yolo_detector_ocv
if torch.cuda.is_available():
    from .models.tensorrt import YOLODetector, YOLODetector_batch
# from .feature_extractor import FeatureExtractor\
from .tracker import MultiTracker
from .utils import profiler,dynamic_lines
from .utils.visualization import Visualizer
from .utils.numba import bisect_right
import numba as nb
import math
from collections import Counter
from .models.yolov5 import *

LOGGER = logging.getLogger(__name__)

class DetectorType(Enum):
    SSD = 0
    YOLO = 1
    PUBLIC = 2
    YOLOOCV = 3
    YOLOV5 = 4
    YOLO_BATCH = 6
    # YOLO_VEH
    # YOLO_COCO
class MOT:
    def __init__(self, size, export, stream_fps,
                 detector_type='YOLO',
                 detector_frame_skip=5,
                 class_ids=(1,),
                 live_drone=None,
                 ssd_detector_cfg=None,
                 yolo_detector_cfg=None,
                 yolo_detector_tile=None,
                 yolo_detector_ocv=None,
                 yolov5_cfg=None,
                 yolov5_custom=None,
                 tracker_cfg=None,
                 visualizer_cfg=None,
                 draw=False,
                 det_type_platf=None):
        """Top level module that integrates detection, feature extraction,
        and tracking together.

        Parameters
        ----------
        size : tuple
            Width and height of each frame.
        detector_type : {'SSD', 'YOLO', 'public'}, optional
            Type of detector to use.
        detector_frame_skip : int, optional
            Number of frames to skip for the detector.
        class_ids : sequence, optional
            Class IDs to track. Note class ID starts at zero.
        ssd_detector_cfg : SimpleNamespace, optional
            SSD detector configuration.
        yolo_detector_cfg : SimpleNamespace, optional
            YOLO detector configuration.
        yolov5_cfg : SimpleNamespace, optional
            YOLOv5 detector configuration.
        public_detector_cfg : SimpleNamespace, optional
            Public detector configuration.
        feature_extractor_cfgs : List[SimpleNamespace], optional
            Feature extractor configurations for all classes.
            Each configuration corresponds to the class at the same index in sorted `class_ids`.
        tracker_cfg : SimpleNamespace, optional
            Tracker configuration.
        visualizer_cfg : SimpleNamespace, optional
            Visualization configuration.
        draw : bool, optional
            Draw visualizations.
        det_type_platf : int, optional
            Forcing a different detector type than the one selected in the config file
        """
        # import pdb; pdb.set_trace()
        self.size = size

        self.detector_type = DetectorType[detector_type.upper()] if det_type_platf is None \
                            else DetectorType[det_type_platf.upper()]
        assert detector_frame_skip >= 1
        self.detector_frame_skip = detector_frame_skip
        self.class_ids = tuple(np.unique(class_ids))
        self.draw = draw
        self.dji_drone = None
        self.export = export
        if ssd_detector_cfg is None: #never used
            ssd_detector_cfg = SimpleNamespace()
        if yolo_detector_ocv is None: #for CPU (can also be ran in GPU). Detects cars from high alt
            yolo_detector_ocv = SimpleNamespace()
        if yolov5_cfg is None: # Detects cars from high alt. Works with GPU or CPU
            yolov5_cfg = SimpleNamespace()
        if yolov5_custom is None: # ...
            yolov5_custom = SimpleNamespace()
        if yolo_detector_cfg is None: # Detects cars from high alt. Works ONLY with GPU (faster on GPU only)
            yolo_detector_cfg = SimpleNamespace()
        if yolo_detector_tile is None: # People + vehicles. Works on GPU only
            yolo_detector_tile = SimpleNamespace()
        if tracker_cfg is None:
            tracker_cfg = SimpleNamespace()

        if visualizer_cfg is None:
            visualizer_cfg = SimpleNamespace()
        #if len(feature_extractor_cfgs) != len(class_ids):
        #    raise ValueError('Number of feature extractors must match length of class IDs')

        LOGGER.info('Loading detector model...')
        # if self.detector_type == DetectorType.SSD:
            # self.detector = SSDDetector(self.size, self.class_ids, **vars(ssd_detector_cfg))
        if self.detector_type == DetectorType.YOLO:
            self.detector = YOLODetector(self.size, self.class_ids, **vars(yolo_detector_cfg))
        elif self.detector_type == DetectorType.YOLOOCV:
            self.detector = Yolo_detector_ocv(self.size, **vars(yolo_detector_ocv))
        elif self.detector_type == DetectorType.YOLOV5:
            self.detector = yolov5_torch(**vars(yolov5_cfg))
        elif self.detector_type == DetectorType.YOLO_BATCH:
            self.detector = YOLODetector_batch(self.size, self.class_ids, **vars(yolo_detector_tile))

        # elif self.detector_type == DetectorType.PUBLIC:
        #     self.detector = PublicDetector(self.size, self.class_ids, self.detector_frame_skip,
        #                                    **vars(public_detector_cfg))

        LOGGER.info('Loading feature extractor models...')
        # self.extractors = [FeatureExtractor(**vars(cfg)) for cfg in feature_extractor_cfgs]
        print("stream fps",stream_fps)
        self.stream_fps = stream_fps
        self.tracker = MultiTracker(self.size, stream_fps, **vars(tracker_cfg))
        self.visualizer = Visualizer(**vars(visualizer_cfg))
        self.frame_count = 0
        self.lines = dynamic_lines.DynaLines(export=export)
        self.avg_vel = []
        self.avg_density = []
        self.avg_flow = []

        # MD
        self.frame_has_detected_object = False

    def visible_tracks(self):
        """Retrieve visible tracks from the tracker

        Returns
        -------
        Iterator[Track]
            Confirmed and active tracks from the tracker.
        """
        return (track for track in self.tracker.tracks.values()
                if track.confirmed and track.active)

    def mean_velo(self):
        """Retrieve average velocity of tracks from the tracker

        Returns
        -------
        Iterator[Track]
            Confirmed and active tracks from the tracker.
        """
        velocities = [track.veloc for track in self.tracker.tracks.values()
                if track.confirmed and track.active]
        if (len(velocities)==0) or (math.isnan(np.array(velocities).mean())):
            return 0
        else:
            return np.array(velocities).mean()

    def labels(self):
        """Retrieve labels of tracks from the tracker
       Returns
       -------
       Iterator[Track]
           Confirmed and active tracks from the tracker.
       """
        labels = [track.lbl_str for track in self.tracker.tracks.values()
                  if track.confirmed and track.active]
        label_array = np.array(labels)
        return dict(Counter(label_array))

    def reset(self, cap_dt):
        """Resets multiple object tracker. Must be called before `step`.

        Parameters
        ----------
        cap_dt : float
            Time interval in seconds between each frame.
        """
        self.frame_count = 0
        self.tracker.reset(cap_dt)

    def step(self, frame, dji_drone):
        """Runs multiple object tracker on the next frame.

        Parameters
        ----------
        frame : ndarray
            The next frame.
        """
        # cv2.setMouseCallback("Video", self.lines.get_mouse_clicks,param=self.frame_count)  # mouse callback in order to add the counter lines
        detections = []
        self.dji_drone = dji_drone
        if self.frame_count == 0:
            if self.detector_type == DetectorType.YOLO:
                detections = self.detector(frame)
            elif self.detector_type == DetectorType.YOLOOCV:
                detections = self.detector.detect(frame)
            elif self.detector_type == DetectorType.YOLOV5:
                # detections = self.detector.detect(frame)
                detections = self.detector.detect(np.expand_dims(frame, axis=0))
            elif self.detector_type == DetectorType.YOLO_BATCH:
                detections = self.detector.detect_async(frame)

            self.tracker.init(frame, detections, self.dji_drone)
            if self.lines != None:
                self.tracker.set_dyna_lines(self.lines)

        elif self.frame_count % self.detector_frame_skip == 0:
            with profiler.Profiler('preproc'):
                if self.detector_type == DetectorType.YOLO:
                    self.detector.detect_async(frame)
            with profiler.Profiler('track'):
                self.tracker.compute_flow(frame)
            with profiler.Profiler('detect'):

                if self.detector_type == DetectorType.YOLO:
                    detections = self.detector.postprocess()
                elif self.detector_type == DetectorType.YOLOOCV:
                    detections = self.detector.detect(frame)
                elif self.detector_type == DetectorType.YOLOV5:
                    # detections = self.detector.detect(frame)
                    detections = self.detector.detect(np.expand_dims(frame, axis=0))
                elif self.detector_type == DetectorType.YOLO_BATCH:
                    detections = self.detector.detect_async(frame)

            with profiler.Profiler('extract'):
                # cls_bboxes = self._split_bboxes_by_cls(detections.tlbr, detections.label,
                #                                        self.class_ids)
                # for extractor, bboxes in zip(self.extractors, cls_bboxes):
                #     extractor.extract_async(frame, bboxes)

                with profiler.Profiler('track', aggregate=True):
                    self.tracker.apply_kalman(frame_id=self.frame_count)

                embeddings = []
                # for extractor in self.extractors:
                #     embeddings.append(extractor.postprocess())
                #embeddings = np.concatenate(embeddings) if len(embeddings) > 1 else embeddings[0]
                embeddings = np.ones((len(detections), 512))
                embeddings /= np.linalg.norm(embeddings, axis=1, keepdims=True)

            with profiler.Profiler('assoc'):
                self.tracker.update(self.frame_count, detections, embeddings,self.dji_drone)
        else:
            with profiler.Profiler('track'):
                self.tracker.track(frame,self.frame_count)

        if self.draw:
            if self.tracker.homography is not None:
                if (np.mean(self.tracker.homography)>0.0):
                    self.lines.correct_lines(self.tracker.homography)
                    self.lines.correct_queues(self.tracker.homography)
            self.lines.draw_count_lines(frame)
            self.lines.draw_queues(frame)
            self._draw(frame, detections)
        self.frame_count += 1

        # MD
        self.frame_has_detected_object = False
        for track in self.tracker.tracks.values():
            if track.confirmed and track.active:
                self.frame_has_detected_object = True
                break


    @staticmethod
    def print_timing_info(frame_count):
        LOGGER.debug('=================Timing Stats=================')
        LOGGER.debug(f"{'track time:':<37}{profiler.Profiler.get_avg_millis('track'):>6.3f} ms")
        LOGGER.debug(f"{'preprocess time:':<37}{profiler.Profiler.get_avg_millis('preproc'):>6.3f} ms")
        LOGGER.debug(f"{'detect/flow time:':<37}{profiler.Profiler.get_avg_millis('detect'):>6.3f} ms")
        LOGGER.debug(f"{'feature extract/kalman filter time:':<37}"
                     f"{profiler.Profiler.get_avg_millis('extract'):>6.3f} ms")
        LOGGER.debug(f"{'association time:':<37}{profiler.Profiler.get_avg_millis('assoc'):>6.3f} ms")
        LOGGER.debug(f"{'Total duration:':<37}{profiler.Profiler.get_duration('app'):>6.3f} s")
        LOGGER.debug(f"{'Total frames:':<37}{frame_count}")
        LOGGER.debug('==============================================')


    @staticmethod
    @nb.njit(cache=True)
    def _split_bboxes_by_cls(bboxes, labels, class_ids):
        cls_bboxes = []
        begin = 0
        for cls_id in class_ids:
            end = bisect_right(labels, cls_id, begin)
            cls_bboxes.append(bboxes[begin:end])
            begin = end
        return cls_bboxes

    @staticmethod
    def get_total_duration():
        return f"{profiler.Profiler.get_duration('app'):>6.3f}"

    def _draw(self, frame, detections):
        visible_tracks = list(self.visible_tracks())
        size = int(5*self.stream_fps)
        if self.frame_count % size == 0 :
            self.velo = []
            self.avg_density = []
            self.avg_flow = []

        # calculate avg velocity
        self.avg_vel.append(self.mean_velo())
        # calculate avg density
        self.avg_density.append(len(visible_tracks))
        # calculate avg flow
        if len(self.lines.queues)>0:
            self.avg_flow.append(len(self.lines.queues[0].track_ids))

        self.visualizer.render(frame, visible_tracks, detections, self.tracker.klt_bboxes.values(),
                               self.tracker.flow.prev_bg_keypoints, self.tracker.flow.bg_keypoints)

        # vizualize calculations
        density = int(np.mean(np.array(self.avg_density)))
        velocity = np.mean(np.array(self.avg_vel)) if np.mean(np.array(self.avg_vel))>0  else 0

        velo_lbl = f'Mean Velocity: {velocity:.2f}km/h'
        (text_widthv, text_heightv), _ = cv2.getTextSize(velo_lbl, cv2.FONT_HERSHEY_DUPLEX, 0.7, 1)
        tl = (30,330)
        # cv2.rectangle(frame, (tl[0], tl[1]-int(text_heightv*1.1)), (tl[0] + int(text_widthv), 390 + int(text_heightv)+1),
        #               (255.0,255.0,255.0), cv2.FILLED)
        # cv2.putText(frame, f'Density: {density}', (30, 330),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2, cv2.LINE_AA)
        # cv2.putText(frame, velo_lbl, (30, 360),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2, cv2.LINE_AA)
        # if len(self.lines.queues)>0:
        #     flow = int(np.mean(np.array(self.avg_flow)))
        #     cv2.putText(frame, f'Mean Flow: {flow}', (30, 390),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2, cv2.LINE_AA)