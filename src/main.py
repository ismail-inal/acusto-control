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
import lib.object_detection as od


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    if not logger.handlers:
        stream = logging.StreamHandler()
        file = logging.FileHandler("control.log")

        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        stream.setFormatter(formatter)
        stream.setLevel(logging.INFO)
        file.setFormatter(formatter)
        file.setLevel(logging.DEBUG)

        logger.addHandler(stream)
        logger.addHandler(file)

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
    for y in range(ctx.config.movement.y_step_num + 1):
        for x in (
            range(ctx.config.movement.x_step_num + 1)
            if y % 2 == 0
            else range(ctx.config.movement.x_step_num + 1)[::-1]
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
                logger.error(f"Error during stage movement: {e}\n", exc_info=True)
                continue

            # object detection
            if ctx.config.en.object_detection:
                logger.info("Starting object detection")
                try:
                    bboxes = od.object_detection(ctx, logger)
                except Exception as e:
                    logger.error(f"Error during object detection: {e}")
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
                        cmr.roi(ctx, logger, bbox, idx)
                    except Exception as e:
                        logger.error(
                            f"Error processing object {idx}: {e}", exc_info=True
                        )
                        continue

                # focus
                if ctx.config.en.auto_focus:
                    try:
                        logger.info("Adjusting focus...")
                        fcs.autofocus_golden(ctx, fcs.measure_std_dev)
                    except Exception as e:
                        logger.error(f"Error during focusing: {e}", exc_info=True)

                # image capture
                logger.info("Starting image capture...")
                if ctx.config.en.depth:
                    cmr.save_range(ctx, frame_dir, logger)
                else:
                    cmr.save_images(
                        ctx.camera, ctx.config.camera.img_num, frame_dir, logger
                    )
                logger.info("Image capture complete.")

                # resetting camera settings
                try:
                    cmr.reset_camera(ctx, logger)
                except Exception as e:
                    logger.fatal(f"Could not reset camera: {e}", exc_info=True)
                    ctx.camera.Close()
                    ctx.pidevice.CloseConnection()
                    sys.exit(1)

    logger.info("Closing connections...")
    try:
        ctx.pidevice.CloseConnection()
    except Exception as e:
        logger.error(f"Error closing motor controller connection: {e}", exc_info=True)

    try:
        ctx.camera.Close()
    except genicam.GenericException as e:
        logger.error(f"Error closing camera connection: {e}", exc_info=True)

    logger.info("Process complete.")


def cleanup(signum, frame, *, camera, width_max, height_max, pidevice, logger):
    logger.info("SIGINT received: resetting camera settings & closing connections…")
    try:
        camera.OffsetX.Value = 0
        camera.OffsetY.Value = 0
        camera.Width.Value = width_max
        camera.Height.Value = height_max
        camera.Close()
    except Exception as e:
        logger.error(f"Error resetting/closing camera: {e}", exc_info=True)
    try:
        pidevice.CloseConnection()
    except Exception as e:
        logger.error(f"Error closing motor controller: {e}", exc_info=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
