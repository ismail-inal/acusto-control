import os
import logging

from pipython import pitools
from pypylon import genicam
from ultralytics import YOLO

import lib.cmr as cmr
import lib.cnf as cnf
import lib.mtr as mtr
from lib.cell_detection import get_bounding_boxes

import lib.fcs as fcs


# TODO: histogram/roi auto exposure
# TODO: autofocus algorithm based on https://opg.optica.org/oe/fulltext.cfm?uri=oe-29-7-10285&id=449327
# TODO: cell yolo model

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(), logging.FileHandler("acusto_control.log")],
)
logger = logging.getLogger(__name__)


def main():
    logger.info("Loading configuration...")
    config = cnf.load_config("config.toml")

    logger.debug(f"Config details:\n{config}")

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

    logger.info("Loading the object detection model...")
    model = YOLO(config.file.model_path)

    try:
        offset_x_increment = camera.OffsetX.GetInc()
        offset_y_increment = camera.OffsetY.GetInc()
        width_increment = camera.Width.GetInc()
        height_increment = camera.Height.GetInc()
        max_width = camera.Width.GetMax()
        max_height = camera.Height.GetMax()

        logger.debug(
            f"Width Increment: {width_increment}, Height Increment: {height_increment}\n"
            + f"OffsetX Increment: {offset_x_increment}, OffsetY Increment: {offset_y_increment}\n"
            + f"Maximum Width: {max_width}, Maximum Height: {max_height}"
        )

    except Exception as e:
        logger.critical(
            f"Error retrieving increment and maximum values: {e}\n"
            + "Terminating operation gracefully..."
        )
        camera.Close()
        pidevice.CloseConnection()
        return

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

            logger.info("Capturing original image...")
            temp_dir = os.path.join(config.file.save_dir, "temp")
            try:
                cmr.save_images(camera, 1, temp_dir)
            except Exception as e:
                logger.error(f"Error capturing image: {e}")
                continue

            logger.info("Detecting objects...")
            temp_file = os.path.join(temp_dir, "0.tiff")
            bboxes = get_bounding_boxes(model, temp_file, 40)

            if bboxes is None:
                logger.warning(
                    "There are no objects detected. Skipping current position..."
                )
                continue

            logger.debug(f"Detected {len(bboxes)} objects.")
            for idx, bbox in enumerate(bboxes):
                frame_dir = os.path.join(
                    config.file.save_dir,
                    f"position({target_x:.2f},{target_y:.2f})_cell{idx}",
                )

                x_min, y_min, x_max, y_max = bbox
                logger.debug(
                    f"\nProcessing Circle {idx}: top left corner ({x_min}, {y_min}), bottom right corner({x_max}, {y_max})"
                )

                try:
                    # NOTE: if you want a specific size uncomment this and comment the other paragraph
                    # camera.Width.Value = config.camera.kernel_size[0]
                    # camera.Height.Value = config.camera.kernel_size[1]

                    adjusted_width = adjust(
                        abs(x_max - x_min), width_increment, max_width
                    )
                    adjusted_height = adjust(
                        abs(y_max - y_min), height_increment, max_height
                    )

                    camera.Width.Value = adjusted_width
                    camera.Height.Value = adjusted_height

                    current_max_offset_x = camera.OffsetX.GetMax()
                    current_max_offset_y = camera.OffsetY.GetMax()

                    adjusted_offset_x = adjust(
                        x_min, offset_x_increment, current_max_offset_x
                    )
                    adjusted_offset_y = adjust(
                        y_min, offset_y_increment, current_max_offset_y
                    )

                    camera.OffsetX.Value = adjusted_offset_x
                    camera.OffsetY.Value = adjusted_offset_y

                    logger.debug(
                        f"Adjusted Camera ROI: Width={camera.Width.Value}, Height={camera.Height.Value}, "
                        f"OffsetX={camera.OffsetX.Value}, OffsetY={camera.OffsetY.Value}"
                    )

                    try:
                        logger.info("Adjusting focus...")
                        fcs.move_to_focus(pidevice, camera, config)
                    except Exception as e:
                        logger.error(f"Error during focusing: {e}")
                        continue

                    logger.info("Starting image capture...")
                    cmr.save_images(camera, config.camera.num_of_images, frame_dir)
                    logger.info("Image capture complete.")

                except Exception as e:
                    logger.error(f"Error processing circle {idx}: {e}")
                    continue

                finally:
                    try:
                        logger.info("Resetting camera settings")
                        camera.OffsetX.Value = 0
                        camera.OffsetY.Value = 0
                        camera.Width.Value = max_width
                        camera.Height.Value = max_height
                        logger.info("Camera settings reset.")
                    except genicam.GenericException as e:
                        logger.critical(
                            f"Error resetting camera settings: {e}\n"
                            + "terminating operation gracefully..."
                        )
                        camera.Close()
                        pidevice.CloseConnection()
                        return

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


def adjust(value, increment, max_value):
    raw_value = int(max(0, value))
    adjusted_value = raw_value - (raw_value % increment)
    final_value = min(adjusted_value, max_value)
    return final_value


if __name__ == "__main__":
    main()
