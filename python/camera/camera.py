import cv2
import numpy as np

from numpy.typing import NDArray

Frame = NDArray[np.uint8]

class Camera:
    """Basic OpenCV camera management with read and close properties"""

    def __init__(self,
        device: int,
        width: int,
        height: int,
        fps: int
    ) -> None:
        self._capture = cv2.VideoCapture(device)

        if not self._capture.isOpened():
            raise RuntimeError(
                f"CAM_ERROR: Could not open camera {device}"
            )
        
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)
        
    def read(self) -> Frame:
        """Return one singular image frame"""
        success, frame = self._capture.read(self)

        if not success:
            raise RuntimeError(
                "CAM_ERROR: Could not return frame from camera"
            )
        
        return frame
        
    def close(self) -> None:
        """Free camera usage by clearing capture property"""
        self._capture.release()