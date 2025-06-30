import os
import logging

from pipython import pitools
from pypylon import genicam, pylon

import lib.cmr as cmr
import lib.cnf as cnf
import lib.mtr as mtr
import lib.fcs as fcs

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(), logging.FileHandler("acusto_control.log")],
)

logger = logging.getLogger(__name__)


def main():
    logger.info("Loading configuration...")
    config = cnf.load_config("config.toml")

    logger.debug("config details")

    try:
        logger.info("Connecting to the motor controller...")
        pidevice = mtr.connect_pi(
            config.motor.controllername,
            config.motor.serialnum,
            config.motor.stages,
            config.motor.refmodes,
        )
    except Exception as e:
        logger.critical(
            f"Could not connect to the motor controller: {e}\n"
            + "Terminating operation."
        )
        return

    logger.info("Connecting to the camera...")
    try:
        camera = cmr.connect_camera(500, config.camera.exposure, config.camera.fps)
    except Exception as e:
        logger.critical(
            f"Could not connect to the camera: {e}\n" + "Terminating operation."
        )
        return

    os.makedirs(config.file.save_dir, exist_ok=True)

    logger.info("Starting scanning process...")
    for y in range(config.movement.num_steps_y + 1):
        for x in (
            range(config.movement.num_steps_x + 1)
            if y % 2 == 0
            else range(config.movement.num_steps_x + 1)[::-1]
        ):
            target_x = config.vertex.pt1[0] + x * config.movement.dx
            target_y = config.vertex.pt1[1] + y * config.movement.dy
            logger.debug(f"\nMoving to position: X={target_x}, Y={target_y}")

            try:
                pidevice.MOV([config.axes.x, config.axes.y], [target_x, target_y])
                pitools.waitontarget(pidevice, axes=(config.axes.x, config.axes.y))
                logger.debug("Stage movement complete.")
            except Exception as e:
                logger.error(f"Error during stage movement: {e}\n")
                continue

            try:
                logger.info("Adjusting focus...")
                # make kernel 91, 3
                fcs.move_to_focus(pidevice, camera, config)
            except Exception as e:
                logger.error(f"Error during focusing: {e}")
                continue

            camera.StartGrabbingMax(1)
            while camera.IsGrabbing():
                with camera.RetrieveResult(2000) as result:
                    if result.GrabSucceeded():
                        img = pylon.PylonImage()
                        img.AttachGrabResultBuffer(result)
                        filename = os.path.join(
                            config.file.save_dir,
                            f"position({target_x:.2f},{target_y:.2f}).tiff",
                        )
                        img.Save(pylon.ImageFileFormat_Tiff, filename)
                        img.Release()
                        logger.debug(f"Saved image: {filename}")
                    else:
                        logger.error(f"Grab failed with error: {result.ErrorCode}")

        camera.StopGrabbing()
        logger.info("Image capture complete.")

    logger.info("Closing connections...")
    try:
        pidevice.CloseConnection()
    except Exception as e:
        logger.error(f"Error closing motion controller connection: {e}")

    try:
        camera.Close()
    except genicam.GenericException as e:
        logger.error(f"Error closing camera connection: {e}")

    logger.info("Process complete.")


if __name__ == "__main__":
    main()
