"""Non-blocking database repository for StreamCV.

Wraps the legacy synchronous ``mysql.connector`` pool in a
:class:`concurrent.futures.ThreadPoolExecutor` so that every write
operation can be awaited from an ``async`` context (or fired-and-forgotten)
without stalling the video frame loop.

Read operations that the frame loop depends on (e.g. telemetry fetch) are
also available as blocking helpers for use inside the executor.
"""

from __future__ import annotations

import logging
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any

import mysql.connector
import pytz
from mysql.connector import pooling
from mysql.connector.errors import PoolError

logger = logging.getLogger(__name__)

_TZ = pytz.utc

# ---------------------------------------------------------------------------
# Connection pool (mirrors _shared_code/database/connection.py)
# ---------------------------------------------------------------------------

_DB_CONFIG = {
    "host": os.environ.get("DB_HOST", "localhost"),
    "port": int(os.environ.get("DB_PORT", 3306)),
    "user": os.environ.get("DB_USER", "root"),
    "password": os.environ.get("DB_PASSWORD", ""),
    "database": os.environ.get("DB_DATABASE", "aiders"),
    "connect_timeout": 30,
}

_POOL_SIZE = 32
_pools: list[pooling.MySQLConnectionPool] = []
_pool_count = 0
_pool_idx = 0


def init_pools(num_pools: int = 2, prefix: str = "streamcv") -> None:
    """Create *num_pools* MySQL connection pools (each of size 32).

    Blocks until the database is reachable, retrying every second — exactly
    like the legacy ``connection.init()`` behaviour.
    """
    global _pool_count
    _pool_count = num_pools
    while True:
        try:
            for i in range(1, num_pools + 1):
                pool = pooling.MySQLConnectionPool(
                    pool_name=f"{prefix}_{i}",
                    pool_size=_POOL_SIZE,
                    **_DB_CONFIG,
                )
                _pools.append(pool)
            logger.info("MySQL connection pools ready: %d", num_pools)
            break
        except Exception as exc:
            logger.warning("DB not ready, retrying in 1 s … (%s)", exc)
            time.sleep(1)


def _get_connection() -> mysql.connector.MySQLConnection:
    global _pool_idx
    try:
        conn = _pools[_pool_idx].get_connection()
        _pool_idx = (_pool_idx + 1) % _pool_count
        return conn
    except PoolError:
        logger.error("All connections in pool exhausted!")
        raise


def _execute(
    query: str,
    params: tuple[Any, ...] | None = None,
    *,
    fetch_one: bool = False,
) -> Any:
    """Run a single SQL statement and return its result.

    * ``SELECT`` → fetched rows (or single row if *fetch_one*).
    * ``INSERT`` → ``lastrowid``.
    * Everything else → ``None``.
    """
    conn = _get_connection()
    try:
        cursor = conn.cursor()
        cursor.execute(query, params)
        if query.lstrip().upper().startswith("SELECT"):
            result = cursor.fetchone() if fetch_one else cursor.fetchall()
        else:
            conn.commit()
            result = cursor.lastrowid if query.lstrip().upper().startswith("INSERT") else None
        cursor.close()
        return result
    except mysql.connector.Error as exc:
        logger.error("MySQL query error: %s", exc)
        return None
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# ThreadPoolExecutor – the non-blocking bridge
# ---------------------------------------------------------------------------

_executor = ThreadPoolExecutor(max_workers=8, thread_name_prefix="db-writer")


def submit(fn, *args, **kwargs):
    """Fire-and-forget a blocking DB function onto the thread-pool.

    Returns a :class:`~concurrent.futures.Future` the caller can optionally
    ``await`` via ``asyncio.wrap_future`` if it needs the result.
    """
    return _executor.submit(fn, *args, **kwargs)


# ---------------------------------------------------------------------------
# Query functions  (exact SQL preserved from cvn/app/database/queries.py)
# ---------------------------------------------------------------------------

# -- Session management -----------------------------------------------------

def get_active_session(drone_id: int) -> tuple | None:
    """``SELECT id FROM aiders_detectionsession …``"""
    return _execute(
        "SELECT id FROM aiders_detectionsession "
        "WHERE drone_id = %s AND is_active = 1 LIMIT 1",
        (drone_id,),
        fetch_one=True,
    )


def deactivate_sessions_and_create_new(
    drone_id: int,
    operation_id: int,
    user_id: int,
) -> int:
    """Deactivate all active sessions for *drone_id*, then create a fresh one.

    The ``latest_frame_url`` is set to the StreamCV live endpoint instead of
    a static placeholder image — no JPG on disk.
    """
    now = datetime.now(_TZ)
    _execute(
        "UPDATE aiders_detectionsession "
        "SET is_active = 0, end_time = %s "
        "WHERE drone_id = %s AND is_active = 1",
        (now, drone_id),
    )
    session_id: int = _execute(
        "INSERT INTO aiders_detectionsession "
        "(operation_id, user_id, drone_id, start_time, is_active, latest_frame_url) "
        "VALUES (%s, %s, %s, %s, %s, %s)",
        (
            operation_id,
            user_id,
            drone_id,
            now,
            1,
            f"/api/streamcv/live/{drone_id}",
        ),
    )
    frame_url = f"/api/streamcv/live/{drone_id}?s={session_id}"
    _execute(
        "UPDATE aiders_detectionsession "
        "SET latest_frame_url = %s WHERE id = %s",
        (frame_url, session_id),
    )
    return session_id


def end_session(session_id: int) -> None:
    _execute(
        "UPDATE aiders_detectionsession "
        "SET is_active = 0, end_time = %s WHERE id = %s",
        (datetime.now(_TZ), session_id),
    )


# -- Detection frames -------------------------------------------------------

def save_frame(session_id: int, drone_id: int) -> int:
    """Insert a detection-frame record **without writing a JPG to disk**.

    The ``frame`` column and ``latest_frame_url`` are pointed at the live
    StreamCV endpoint so the existing Web UI can still resolve a URL.
    The ``?s=`` cache-buster is session-unique so the browser reconnects
    the MJPEG stream when a new session starts.
    """
    frame_url = f"/api/streamcv/live/{drone_id}?s={session_id}"
    now = datetime.now(_TZ)

    frame_id: int = _execute(
        "INSERT INTO aiders_detectionframe "
        "(detection_session_id, frame, time) "
        "VALUES (%s, %s, %s)",
        (session_id, frame_url, now),
    )
    _execute(
        "UPDATE aiders_detectionsession "
        "SET latest_frame_url = %s WHERE id = %s",
        (frame_url, session_id),
    )
    return frame_id


# -- Telemetry --------------------------------------------------------------

def get_drone_telemetry(drone_id: int) -> tuple | None:
    """``SELECT lat, lon, alt, heading, gimbal_angle …``

    Returns ``(lat, lon, alt, heading, gimbal_angle)`` or ``None``.
    """
    return _execute(
        "SELECT lat, lon, alt, heading, gimbal_angle "
        "FROM aiders_telemetry "
        "WHERE drone_id = %s ORDER BY id DESC LIMIT 1",
        (drone_id,),
        fetch_one=True,
    )


# -- Detected objects -------------------------------------------------------

def save_detected_object(
    lat: float,
    lon: float,
    label: str,
    track_id: int,
    distance: float,
    operation_id: int,
    drone_id: int,
    session_id: int,
    frame_id: int,
) -> None:
    _execute(
        "INSERT INTO aiders_detectedobject "
        "(lat, lon, label, track_id, distance_from_drone, "
        "operation_id, drone_id, detection_session_id, frame_id, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
        (
            lat, lon, label, track_id, distance,
            operation_id, drone_id, session_id, frame_id,
            datetime.now(_TZ),
        ),
    )


def save_detected_objects_batch(
    rows: list[tuple],
    operation_id: int,
    drone_id: int,
    session_id: int,
    frame_id: int,
) -> None:
    """Bulk-insert detected objects with bounding boxes and confidence.

    Each element of *rows* is
    ``(lat, lon, label, track_id, distance, bounding_boxes_json, confidence)``.
    """
    if not rows:
        return
    now = datetime.now(_TZ)
    conn = _get_connection()
    try:
        cursor = conn.cursor()
        cursor.executemany(
            "INSERT INTO aiders_detectedobject "
            "(lat, lon, label, track_id, distance_from_drone, "
            "bounding_boxes, confidence, "
            "operation_id, drone_id, detection_session_id, frame_id, time) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
            [
                (
                    lat, lon, label, track_id, distance,
                    bbox_json, conf,
                    operation_id, drone_id, session_id, frame_id, now,
                )
                for lat, lon, label, track_id, distance, bbox_json, conf in rows
            ],
        )
        conn.commit()
        cursor.close()
    except mysql.connector.Error as exc:
        logger.error("Batch insert error: %s", exc)
    finally:
        conn.close()


# -- Crowd localization results ---------------------------------------------

def save_crowd_localization_result(
    session_id: int,
    operation_id: int,
    drone_id: int,
    frame_id: int,
    coordinates_json: str,
    count: int,
) -> None:
    """Insert one aggregate crowd-localization row for a sampled frame.

    Unlike ``aiders_detectedobject`` (per-object rows with track_id/label/
    confidence), this table holds one row per sampled frame containing all
    detected point coordinates as a JSON blob plus the point count -- there
    is no tracking/identity concept for crowd localization.
    """
    _execute(
        "INSERT INTO aiders_detectioncrowdlocalizationresults "
        "(drone_id, operation_id, detection_session_id, frame_id, coordinates, count, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s)",
        (drone_id, operation_id, session_id, frame_id, coordinates_json, count, datetime.now(_TZ)),
    )


# -- Disaster classification -------------------------------------------------

def save_detected_disaster(
    session_id: int,
    operation_id: int,
    drone_id: int,
    frame_id: int,
    lat: float,
    lon: float,
    earthquake: float,
    fire: float,
    flood: float,
) -> None:
    """Insert one disaster-classification row.

    Unlike ``aiders_detectedobject``/``aiders_detectioncrowdlocalizationresults``,
    lat/lon here come directly from drone telemetry, not a pixel-to-GPS
    conversion -- whole-frame classification has no pixel-space location to
    convert ("the disaster is somewhere near the drone", not "at this pixel").

    Whether to call this at all (the legacy 50%-confidence threshold gate)
    is the caller's decision, not this function's -- this is a pure
    "write these values" primitive, matching the rest of this module.
    """
    _execute(
        "INSERT INTO aiders_detecteddisaster "
        "(lat, lon, earthquake_probability, fire_probability, flood_probability, "
        "operation_id, drone_id, detection_session_id, frame_id, time) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
        (
            lat, lon, earthquake, fire, flood,
            operation_id, drone_id, session_id, frame_id,
            datetime.now(_TZ),
        ),
    )


def save_video_path(session_id: int, video_path: str) -> None:
    """Store the path to the recorded detection video in the session record."""
    _execute(
        "UPDATE aiders_detectionsession "
        "SET video_recording_path = %s WHERE id = %s",
        (video_path, session_id),
    )


# -- Detection entry status --------------------------------------------------

def update_detection_entry(
    drone_id: int,
    status: str,
    detection_type: str,
    model: str,
) -> None:
    _execute(
        "UPDATE aiders_detection "
        "SET detection_status = %s, detection_type_str = %s, "
        "detection_model = %s WHERE drone_id = %s",
        (status, detection_type, model, drone_id),
    )
