import torch
import numpy

model = torch.hub.load(
    "WongKinYiu/yolov7",
    "custom",
    "./models/bead.pt",
    force_reload=True,
    trust_repo=True,
)


print(type(model))
