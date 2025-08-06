from os import makedirs, path

import numpy as np
from pypylon import pylon
from pipython import pitools


def connect_camera(exposure, fps) -> pylon.InstantCamera:
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    # camera.MaxNumBuffer.Value = config.buffer_val
    camera.ExposureTime.Value = exposure
    camera.AcquisitionFrameRateEnable.Value = True
    camera.AcquisitionFrameRate.Value = fps
    return camera


def return_image(camera: pylon.InstantCamera) -> np.ndarray:
    camera.StartGrabbingMax(1)

    with camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException) as result:
        if result.GrabSucceeded():
            img = result.GetArray()
        else:
            raise RuntimeError("Image grab failed")

    camera.StopGrabbing()

    return img


def save_images(
    camera: pylon.InstantCamera,
    num: int,
    file_dir: str,
    logger,
    grab_idx: int = 0,
) -> None:
    makedirs(file_dir, exist_ok=True)

    camera.StartGrabbingMax(num)

    while camera.IsGrabbing():
        with camera.RetrieveResult(2000) as result:
            if result.GrabSucceeded():
                img = pylon.PylonImage()
                img.AttachGrabResultBuffer(result)
                filename = path.join(file_dir, f"{grab_idx}.tiff")
                img.Save(pylon.ImageFileFormat_Tiff, filename)
                img.Release()
                grab_idx += 1
            else:
                logger.error(f"Grab failed with error: {result.ErrorCode}")

    camera.StopGrabbing()


def save_range(
    ctx,
    frame_dir: str,
    logger,
):
    org_z = ctx.pidevice.qPOS(ctx.config.axes.z)[ctx.config.axes.z]
    for step_num in range(
        -ctx.config.movement.z_max_step, ctx.config.movement.z_max_step + 1, 1
    ):
        target_z = org_z + ctx.config.movement.dz * step_num
        ctx.pidevice.MOV(ctx.config.axes.z, target_z)
        pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)
        save_images(ctx.camera, 1, frame_dir, logger, step_num)

    ctx.pidevice.MOV(ctx.config.axes.z, org_z)
    pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)
    return


def return_range(ctx, l_pos, r_pos):
    img_arr = []
    pos_range = np.arange(
        l_pos, r_pos + ctx.config.focus.step_finer, ctx.config.focus.step_finer
    )
    pos_range = np.clip(pos_range, l_pos, r_pos)
    for pos in pos_range:
        ctx.pidevice.MOV(ctx.config.axes.z, pos)
        pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)
        img_arr.append(return_image(ctx.camera))

    img_arr = np.array(img_arr)
    return img_arr


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


def roi(ctx, logger, bbox, idx):
    def _adjust(value, increment, max_value):
        raw_value = int(max(0, value))
        adjusted_value = raw_value - (raw_value % increment)
        final_value = min(adjusted_value, max_value)
        return final_value

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

    adjusted_width = _adjust(abs(x_max - x_min), ctx.width_inc, ctx.width_max)
    adjusted_height = _adjust(abs(y_max - y_min), ctx.height_inc, ctx.height_max)
    adjusted_offset_x = _adjust(x_min, ctx.offset_x_inc, current_max_offset_x)
    adjusted_offset_y = _adjust(y_min, ctx.offset_y_inc, current_max_offset_y)

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
