from os import makedirs, path

import numpy as np
from pypylon import pylon
from pipython import pitools

import lib.context as ctx


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
    ctx: ctx.AppContext,
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


def return_range(ctx: ctx.AppContext, l_pos, r_pos):
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
