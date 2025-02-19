from torch.nn import Module
from cv2 import imread
from numpy import ndarray
from typing import Optional


def get_bounding_boxes(model: Module, img_path: str) -> Optional[ndarray]:
    img = imread(img_path)
    results = model(img)
    boxes = results.xyxy[0]
    if len(boxes) == 0:
        return None
    boxes = boxes.cpu().numpy()
    return boxes
