import os
import sys

import lib.config as cnf
import lib.camera as cmr
import lib.motor as mtr
from ultralytics import YOLO


class AppContext:
    def __init__(self, logger, config_path="config.toml"):
        self.logger = logger
        self.logger.info("Loading configuration...")
        self.config = cnf.load_config(config_path)
        self.logger.debug(f"Config details:\n{self.config}")

        self._connect_motor()
        self._connect_camera()
        self._load_model()
        self._fetch_camera_limits()
        self._prepare_directories()

    def _connect_motor(self):
        try:
            self.logger.info("Connecting to the motor controller...")
            self.pidevice = mtr.connect_pi(
                self.config.motor.controllername,
                self.config.motor.serialnum,
                self.config.motor.stages,
                self.config.motor.refmodes,
            )
        except Exception as e:
            self.logger.critical(
                f"Could not connect to the motor controller: {e}\nTerminating operation."
            )
            sys.exit(1)

    def _connect_camera(self):
        try:
            self.logger.info("Connecting to the camera...")
            self.camera = cmr.connect_camera(
                500, self.config.camera.exposure, self.config.camera.fps
            )
        except Exception as e:
            self.logger.critical(
                f"Could not connect to the camera: {e}\nTerminating operation."
            )
            sys.exit(1)

    def _load_model(self):
        self.logger.info("Loading the object detection model...")
        self.model = YOLO(self.config.file.model_path)

    def _fetch_camera_limits(self):
        self.logger.info("Retrieving camera limits.")
        try:
            cam = self.camera
            self.offset_x_inc = cam.OffsetX.GetInc()
            self.offset_y_inc = cam.OffsetY.GetInc()
            self.width_inc = cam.Width.GetInc()
            self.height_inc = cam.Height.GetInc()
            self.width_max = cam.Width.GetMax()
            self.height_max = cam.Height.GetMax()

            self.logger.debug(
                f"Width Increment: {self.width_inc}, Height Increment: {self.height_inc}\n"
                f"OffsetX Increment: {self.offset_x_inc}, OffsetY Increment: {self.offset_y_inc}\n"
                f"Maximum Width: {self.width_max}, Maximum Height: {self.height_max}"
            )
        except Exception as e:
            self.logger.critical(
                f"Error retrieving increment and maximum values: {e}\n"
                + "Terminating operation gracefully..."
            )
            self.camera.Close()
            self.pidevice.CloseConnection()
            sys.exit(1)

    def _prepare_directories(self):
        os.makedirs(self.config.file.save_dir, exist_ok=True)

    def close_all(self):
        self.logger.info("Shutting down...")
        try:
            self.camera.Close()
            self.pidevice.CloseConnection()
        except Exception as e:
            self.logger.warning(f"Error during shutdown: {e}")
