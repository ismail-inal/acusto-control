from cv2 import imread
from ultralytics.models.yolo import YOLO
from numpy import ndarray
from typing import Optional


def get_bounding_boxes(model: YOLO, img_path: str) -> Optional[ndarray]:
    img = imread(img_path)
    results = model(img)
    bboxes = results[0].boxes.xyxy

    if bboxes.shape[0] == 0:
        return None

    return bboxes.cpu().numpy()
