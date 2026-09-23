from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_RECONNECT_DELAY = 1.0
_MAX_RECONNECT_DELAY = 30.0


class ThreadedCamera:
    """Continuously reads an RTMP stream in a background thread, keeping only
    the most recent frame.  Old frames are silently discarded so downstream
    consumers always get the lowest-latency image available.

    Parameters
    ----------
    rtmp_url:
        Full RTMP URI, e.g. ``rtmp://host/live/drone1``.
    reconnect_delay:
        Initial seconds to wait before reopening the stream after a read
        failure.  Doubles on each consecutive failure up to *max_reconnect_delay*.
    max_reconnect_delay:
        Upper bound for the exponential back-off.
    """

    def __init__(
        self,
        rtmp_url: str,
        reconnect_delay: float = _DEFAULT_RECONNECT_DELAY,
        max_reconnect_delay: float = _MAX_RECONNECT_DELAY,
    ) -> None:
        self._url = rtmp_url
        self._reconnect_delay = reconnect_delay
        self._max_reconnect_delay = max_reconnect_delay

        self._capture: cv2.VideoCapture | None = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = True

        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            logger.info("Opened RTMP stream: %s", self._url)
        else:
            logger.warning("Failed to open RTMP stream: %s", self._url)
        return cap

    def _reader(self) -> None:
        self._capture = self._open()
        delay = self._reconnect_delay

        while self._running:
            if self._capture is None or not self._capture.isOpened():
                logger.info(
                    "Reconnecting to %s in %.1fs ...", self._url, delay,
                )
                time.sleep(delay)
                delay = min(delay * 2, self._max_reconnect_delay)
                self._capture = self._open()
                continue

            ret, frame = self._capture.read()
            if not ret:
                logger.warning("Read failure on %s, will reconnect.", self._url)
                self._capture.release()
                self._capture = None
                continue

            delay = self._reconnect_delay

            with self._lock:
                self._frame = frame

    def read(self) -> np.ndarray | None:
        """Return a *copy* of the latest frame, or ``None`` if no frame has
        been captured yet."""
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    @property
    def is_opened(self) -> bool:
        return self._capture is not None and self._capture.isOpened()

    def release(self) -> None:
        """Signal the reader thread to stop and release the capture device."""
        self._running = False
        self._thread.join(timeout=5.0)
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        logger.info("Released ThreadedCamera for %s", self._url)
