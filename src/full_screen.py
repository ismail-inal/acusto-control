import os

from pipython import pitools
from pypylon import genicam

import lib.cmr as cmr
import lib.cnf as cnf
import lib.mtr as mtr


def main():
    print("Loading configuration...")
    config = cnf.load_config("config.json")

    print("Connecting to the motor controller...")
    pidevice = mtr.connect_pi(
        config.motor.controllername,
        config.motor.serialnum,
        config.motor.stages,
        config.motor.refmodes,
    )

    print("Connecting to the camera...")
    camera = cmr.connect_camera(500, config.camera.exposure)
    output_dir = config.file.save_dir

    os.makedirs(output_dir, exist_ok=True)

    org_width = camera.Width.Value
    org_height = camera.Height.Value
    print(f"Original Camera Resolution: {org_width}x{org_height}")

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

            frame_dir = os.path.join(output_dir, f"position({target_x},{target_y})")
            cmr.save_images(camera, 10, frame_dir)
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


def adjust_offset(value, increment):
    return value - (value % increment)


if __name__ == "__main__":
    main()
