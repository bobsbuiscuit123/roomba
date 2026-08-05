import os
import threading
import time
from typing import Optional


class CameraStream:
    def __init__(self) -> None:
        self.device = os.environ.get("ROOMBA_CAMERA_DEVICE", "0")
        self.width = int(os.environ.get("ROOMBA_CAMERA_WIDTH", "640"))
        self.height = int(os.environ.get("ROOMBA_CAMERA_HEIGHT", "480"))
        self.fps = int(os.environ.get("ROOMBA_CAMERA_FPS", "15"))
        self.lock = threading.Lock()
        self.capture = None
        self.cv2 = None
        self.last_error = ""
        self.next_open_attempt = 0.0

    def _load_cv2(self):
        if self.cv2 is not None:
            return self.cv2

        try:
            import cv2
        except Exception as error:
            self.last_error = "OpenCV is not installed: " + str(error)
            return None

        self.cv2 = cv2
        return cv2

    def _open_locked(self) -> bool:
        if self.capture is not None and self.capture.isOpened():
            return True

        if time.monotonic() < self.next_open_attempt:
            return False

        cv2 = self._load_cv2()

        if cv2 is None:
            self.next_open_attempt = time.monotonic() + 5
            return False

        try:
            source: int | str = int(self.device)
        except ValueError:
            source = self.device

        capture = cv2.VideoCapture(source)

        if not capture.isOpened():
            self.last_error = "Could not open camera " + str(self.device)
            self.next_open_attempt = time.monotonic() + 5
            capture.release()
            self.capture = None
            return False

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture = capture
        self.last_error = ""
        self.next_open_attempt = 0.0
        return True

    def get_jpeg(self) -> Optional[bytes]:
        with self.lock:
            if not self._open_locked() or self.capture is None:
                return None

            ok, frame = self.capture.read()

            if not ok:
                self.last_error = "Could not read camera frame"
                self.capture.release()
                self.capture = None
                self.next_open_attempt = time.monotonic() + 2
                return None

            ok, encoded = self.cv2.imencode(
                ".jpg",
                frame,
                [int(self.cv2.IMWRITE_JPEG_QUALITY), 78],
            )

            if not ok:
                self.last_error = "Could not encode camera frame"
                return None

            return encoded.tobytes()

    def frames(self):
        delay = 1 / max(1, self.fps)

        while True:
            frame = self.get_jpeg()

            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )

            time.sleep(delay)

    def status(self) -> dict:
        with self.lock:
            ok = self._open_locked()
            return {
                "ok": ok,
                "device": self.device,
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "error": "" if ok else self.last_error,
            }

    def close(self) -> None:
        with self.lock:
            if self.capture is not None:
                self.capture.release()
                self.capture = None
