from pipython import GCSDevice, pitools
from pypylon import pylon, genicam
import os
import lib.cmr as cmr
import lib.circle_detection as cdt
import lib.mtr as mtr
import lib.fcs as fcs
import lib.cnf as cnf


def adjust_offset(value, increment):
    """
    Adjusts the given value down to the nearest multiple of the increment.
    """
    return value - (value % increment)


def main():
    print("Loading configuration...")
    config = cnf.load_config("config.json")

    print("Connecting to motion controller...")
    pidevice = mtr.connect_pi(
        config["CONTROLLERNAME"],
        config["SERIALNUM"],
        config["STAGES"],
        config["REFMODES"],
    )

    print("Connecting to camera...")
    camera = cmr.connect_camera(500, config["EXPOSURE"])
    output_dir = config["DIR"]

    os.makedirs(output_dir, exist_ok=True)

    org_width = camera.Width.Value
    org_height = camera.Height.Value
    print(f"Original Camera Resolution: {org_width}x{org_height}")

    print("Starting scanning process...")
    for y in range(config["STEP_NUM"][1] + 1):
        for x in (
            range(config["STEP_NUM"][0] + 1)
            if y % 2 == 0
            else range(config["STEP_NUM"][0] + 1)[::-1]
        ):
            target_x = config["VERTEX"]["0,0"][0] + x * config["DX"]
            target_y = config["VERTEX"]["0,0"][1] + y * config["DY"]
            print(f"\nMoving to position: X={target_x}, Y={target_y}")

            try:
                pidevice.MOV(
                    [config["AXES"]["x"], config["AXES"]["y"]], [target_x, target_y]
                )
                pitools.waitontarget(
                    pidevice, axes=(config["AXES"]["x"], config["AXES"]["y"])
                )
                print("Stage movement complete.")
            except Exception as e:
                print(f"Error during stage movement: {e}")
                continue  # Skip to the next position

            try:
                print("Starting image capture...")
                camera.StartGrabbingMax(1)  # Grabbing 10 images

                grab_idx = 0  # Initialize grab index
                while camera.IsGrabbing():
                    with camera.RetrieveResult(2000) as result:
                        if result.GrabSucceeded():
                            img = pylon.PylonImage()
                            img.AttachGrabResultBuffer(result)
                            # Include grab index to ensure unique filenames
                            filename = os.path.join(
                                output_dir, f"position({target_x},{target_y})"
                            )
                            img.Save(pylon.ImageFileFormat_Tiff, filename)
                            img.Release()
                            print(f"Saved image: {filename}")
                            grab_idx += 1  # Increment grab index
                        else:
                            print(f"Grab failed with error: {result.ErrorCode}")

                camera.StopGrabbing()
                print("Image capture complete.")

            except Exception as e:
                print(f"Error processing pos ({x}, {y}): {e}")

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
