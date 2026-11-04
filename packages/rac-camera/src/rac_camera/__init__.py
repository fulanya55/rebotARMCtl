"""Single camera capture module with shared timestamped frames."""

from .shm import CameraReader, CameraWriter, Frame, segment_name

__all__ = ["CameraReader", "CameraWriter", "Frame", "probe_cameras", "segment_name"]


def probe_cameras(max_index: int = 6) -> list[dict]:
    """USB cameras that open and return a frame. Skips anything already captured."""
    import cv2

    found = []
    for index in range(max_index):
        capture = cv2.VideoCapture(index, cv2.CAP_DSHOW if _is_windows() else cv2.CAP_ANY)
        try:
            if capture.isOpened() and capture.read()[0]:
                found.append(
                    {
                        "source": index,
                        "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                        "fps": capture.get(cv2.CAP_PROP_FPS),
                    }
                )
        finally:
            capture.release()
    return found


def _is_windows() -> bool:
    import os

    return os.name == "nt"
