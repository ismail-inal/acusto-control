from dataclasses import dataclass
from typing import List

import tomllib


@dataclass
class CameraConfig:
    exposure: float
    fps: float
    img_num: int


@dataclass
class MotorConfig:
    controllername: str
    stages: List[str]
    refmodes: List[str]
    serialnum: str


@dataclass
class AxesConfig:
    x: int
    y: int
    z: int


@dataclass
class FocusConfig:
    z_min: float
    z_max: float
    step_coarse: int
    step_fine: int
    step_finer: int
    step_num: int


@dataclass
class VertexConfig:
    pt1: List[float]
    pt2: List[float]


@dataclass
class ODConfig:
    buffer_size: int
    d_cells: int
    d_boundary: int


@dataclass
class MovementConfig:
    dx: float
    dy: float
    dz: float
    z_max_step: int
    x_step_num: int
    y_step_num: int


@dataclass
class FileConfig:
    save_dir: str
    model_path: str


@dataclass
class EnConfig:
    auto_focus: bool
    object_detection: bool
    dynamic_exposure: bool
    depth: bool


@dataclass
class Config:
    camera: CameraConfig
    motor: MotorConfig
    axes: AxesConfig
    vertex: VertexConfig
    movement: MovementConfig
    file: FileConfig
    en: EnConfig
    focus: FocusConfig
    od: ODConfig

    @classmethod
    def from_dict(cls, config_dict: dict) -> "Config":
        return cls(
            camera=CameraConfig(**config_dict["CAMERA"]),
            motor=MotorConfig(**config_dict["MOTOR"]),
            axes=AxesConfig(**config_dict["AXES"]),
            vertex=VertexConfig(**config_dict["VERTEX"]),
            movement=MovementConfig(**config_dict["MOVEMENT"]),
            file=FileConfig(**config_dict["FILE"]),
            en=EnConfig(**config_dict["EN"]),
            focus=FocusConfig(**config_dict["FOCUS"]),
            od=ODConfig(**config_dict["OD"]),
        )


def load_config(config_path="config.toml"):
    with open(config_path, "rb") as file:
        config = tomllib.load(file)

    config["MOVEMENT"]["x_step_num"] = int(
        abs(config["VERTEX"]["pt1"][0] - config["VERTEX"]["pt2"][0])
        // config["MOVEMENT"]["dx"]
    )

    config["MOVEMENT"]["y_step_num"] = int(
        abs(config["VERTEX"]["pt1"][1] - config["VERTEX"]["pt2"][1])
        // config["MOVEMENT"]["dy"]
    )

    config["FOCUS"]["step_coarse"] *= config["MOVEMENT"]["dz"]
    config["FOCUS"]["step_fine"] *= config["MOVEMENT"]["dz"]
    config["FOCUS"]["step_finer"] *= config["MOVEMENT"]["dz"]

    return Config.from_dict(config)
