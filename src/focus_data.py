import os

from pipython import pitools
from pypylon import genicam

import lib.cmr as cmr
import lib.cnf as cnf
import lib.mtr as mtr
import lib.fcs as fcs


def main():
    print("Loading configuration...")
    config = cnf.load_config("config.toml")

    print("Connecting to the motor controller...")
    pidevice = mtr.connect_pi(
        config.motor.controllername,
        config.motor.serialnum,
        config.motor.stages,
        config.motor.refmodes,
    )

    print("Connecting to the camera...")
    camera = cmr.connect_camera(500, config.camera.exposure, config.camera.fps)

    print("Starting scanning process...")
    for y in range(config.movement.num_steps_y + 1):
        for x in (
            range(config.movement.num_steps_x + 1)
            if y % 2 == 0
            else range(config.movement.num_steps_x + 1)[::-1]
        ):
            target_x = config.vertex.pt1[0] + x * config.movement.dx
            target_y = config.vertex.pt1[1] + y * config.movement.dy
            print(f"\nMoving to position: X={target_x}, Y={target_y}")

            try:
                pidevice.MOV([config.axes.x, config.axes.y], [target_x, target_y])
                pitools.waitontarget(pidevice, axes=(config.axes.x, config.axes.y))
                print("Stage movement complete.")
            except Exception as e:
                print(f"Error during stage movement: {e}")
                continue

            output_dir = os.path.join(
                config.file.save_dir,
                f"position({target_x:.2f},{target_y:.2f})",
            )
            print("Starting Image capture.")
            fcs.capture_focus_range(pidevice, camera, config, output_dir)
            print("Image capture complete.")

    print("Closing connections...")
    try:
        pidevice.CloseConnection()
    except Exception as e:
        print(f"Error closing motion controller connection: {e}")

    try:
        camera.Close()
    except genicam.GenericException as e:
        print(f"Error closing camera connection: {e}")

    print("Process complete.")


if __name__ == "__main__":
    main()
