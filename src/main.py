import os
import signal
import sys
from functools import partial
import logging

from pipython import pitools
from pypylon import genicam

from lib.context import AppContext
import lib.camera as cmr
import lib.focus as fcs
from lib.cell_detection import get_bounding_boxes


# TODO: histogram/roi auto exposure
# TODO: wrap processes in modules that can be enabled/disabled (via a boolean in config.toml)
# TODO: cell yolo model


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("acusto_control.log"),
        ],
    )
    logger = logging.getLogger(__name__)

    ctx = AppContext(logger=logger)

    handler = partial(
        cleanup,
        camera=ctx.camera,
        width_max=ctx.width_max,
        height_max=ctx.height_max,
        pidevice=ctx.pidevice,
        logger=logger,
    )
    signal.signal(signal.SIGINT, handler)

    logger.info("Starting scanning process...")
    for y in range(ctx.config.movement.num_steps_y + 1):
        for x in (
            range(ctx.config.movement.num_steps_x + 1)
            if y % 2 == 0
            else range(ctx.config.movement.num_steps_x + 1)[::-1]
        ):
            # scanning
            target_x = ctx.config.vertex.pt1[0] + x * ctx.config.movement.dx
            target_y = ctx.config.vertex.pt1[1] + y * ctx.config.movement.dy
            logger.debug(f"\nMoving to position: X={target_x}, Y={target_y}")

            try:
                ctx.pidevice.MOV(
                    [ctx.config.axes.x, ctx.config.axes.y], [target_x, target_y]
                )
                pitools.waitontarget(
                    ctx.pidevice, axes=(ctx.config.axes.x, ctx.config.axes.y)
                )
                logger.debug("Stage movement complete.")
            except Exception as e:
                logger.error(f"Error during stage movement: {e}\n")
                continue

            # object detection
            if ctx.config.en.object_detection:
                try:
                    bboxes = object_detection(ctx, logger)
                except Exception:
                    continue
            else:
                bboxes = [[0, 0, ctx.width_max, ctx.height_max]]

            logger.debug(f"Detected {len(bboxes)} objects.")
            for idx, bbox in enumerate(bboxes):
                frame_dir = os.path.join(
                    ctx.config.file.save_dir,
                    f"position({target_x:.2f},{target_y:.2f})_cell{idx}",
                )

                # roi
                if ctx.config.en.object_detection:
                    try:
                        roi(ctx, logger, bbox, idx)
                    except Exception as e:
                        logger.error(f"Error processing object {idx}: {e}")
                        continue

                # focus
                if ctx.config.en.auto_focus:
                    try:
                        logger.info("Adjusting focus...")
                        fcs.move_to_focus(ctx.pidevice, ctx.camera, ctx.config)
                    except Exception as e:
                        logger.error(f"Error during focusing: {e}")

                # image capture
                logger.info("Starting image capture...")
                if ctx.config.en.depth:
                    fcs._capture_focus_range(
                        ctx.pidevice, ctx.camera, ctx.config, frame_dir
                    )
                else:
                    cmr.save_images(
                        ctx.camera, ctx.config.camera.img_num, frame_dir, logger
                    )
                logger.info("Image capture complete.")

                # resetting camera settings
                try:
                    reset_camera(ctx, logger)
                except Exception as _:
                    ctx.camera.Close()
                    ctx.pidevice.CloseConnection()
                    sys.exit(1)

    logger.info("Closing connections...")
    try:
        ctx.pidevice.CloseConnection()
    except Exception as e:
        logger.error(f"Error closing motor controller connection: {e}")

    try:
        ctx.camera.Close()
    except genicam.GenericException as e:
        logger.error(f"Error closing camera connection: {e}")

    logger.info("Process complete.")


def adjust(value, increment, max_value):
    raw_value = int(max(0, value))
    adjusted_value = raw_value - (raw_value % increment)
    final_value = min(adjusted_value, max_value)
    return final_value


def object_detection(ctx: AppContext, logger):
    logger.info("Capturing original image...")
    temp_dir = os.path.join(ctx.config.file.save_dir, "temp")
    try:
        cmr.save_images(ctx.camera, 1, temp_dir, logger)
    except Exception as e:
        logger.error(f"Error capturing image: {e}")
        raise e

    logger.info("Detecting objects...")
    temp_file = os.path.join(temp_dir, "0.tiff")
    bboxes = get_bounding_boxes(ctx.model, temp_file, 40)

    if bboxes is None:
        logger.warning("There are no objects detected. Skipping current position...")
        raise ValueError("No objects detected.")

    logger.debug(f"Detected {len(bboxes)} objects.")

    return bboxes


def roi(ctx, logger, bbox, idx):
    x_min, y_min, x_max, y_max = bbox
    logger.debug(
        f"\nProcessing object {idx}: top left corner ({x_min}, {y_min}), bottom right corner({x_max}, {y_max})"
    )

    try:
        current_max_offset_x = ctx.camera.OffsetX.GetMax()
        current_max_offset_y = ctx.camera.OffsetY.GetMax()
    except Exception as e:
        logger.error(f"Error fetching camera offsets: {e}")
        raise e

    adjusted_width = adjust(abs(x_max - x_min), ctx.width_inc, ctx.width_max)
    adjusted_height = adjust(abs(y_max - y_min), ctx.height_inc, ctx.height_max)
    adjusted_offset_x = adjust(x_min, ctx.offset_x_inc, current_max_offset_x)
    adjusted_offset_y = adjust(y_min, ctx.offset_y_inc, current_max_offset_y)

    try:
        ctx.camera.Width.Value = adjusted_width
        ctx.camera.Height.Value = adjusted_height
        ctx.camera.OffsetX.Value = adjusted_offset_x
        ctx.camera.OffsetY.Value = adjusted_offset_y
    except Exception as e:
        logger.error(f"Error setting camera dimensions/offsets: {e}")
        raise e

    logger.debug(
        f"Adjusted camera ROI: Width={ctx.camera.Width.Value}, Height={ctx.camera.Height.Value}, "
        f"OffsetX={ctx.camera.OffsetX.Value}, OffsetY={ctx.camera.OffsetY.Value}"
    )


def reset_camera(ctx, logger):
    try:
        logger.info("Resetting camera settings")
        ctx.camera.OffsetX.Value = 0
        ctx.camera.OffsetY.Value = 0
        ctx.camera.Width.Value = ctx.width_max
        ctx.camera.Height.Value = ctx.height_max
        logger.info("Camera settings reset.")
    except Exception as e:
        logger.critical(f"Error resetting camera settings: {e}")
        raise e


def cleanup(signum, frame, *, camera, width_max, height_max, pidevice, logger):
    logger.info("SIGINT received: resetting camera settings & closing connections…")
    try:
        camera.OffsetX.Value = 0
        camera.OffsetY.Value = 0
        camera.Width.Value = width_max
        camera.Height.Value = height_max
        camera.Close()
    except Exception as e:
        logger.error(f"Error resetting/closing camera: {e}")
    try:
        pidevice.CloseConnection()
    except Exception as e:
        logger.error(f"Error closing motor controller: {e}")
    sys.exit(0)


if __name__ == "__main__":
    main()
