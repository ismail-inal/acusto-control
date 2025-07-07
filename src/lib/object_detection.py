import cv2
from ultralytics.models.yolo import YOLO
import numpy as np
from typing import Optional


def get_bounding_boxes(
    model: YOLO, img_path: str, buffer_size: int
) -> Optional[np.ndarray]:
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    results = model(img)
    bboxes = results[0].boxes.xyxy

    if bboxes.shape[0] == 0:
        return None

    bboxes = np.round(bboxes.cpu().numpy()).astype(int)

    bboxes[:, 0] -= buffer_size
    bboxes[:, 1] -= buffer_size
    bboxes[:, 2] += buffer_size
    bboxes[:, 3] += buffer_size

    return bboxes
