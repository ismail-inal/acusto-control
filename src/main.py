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
    config = cnf.load_config('config.json')
    
    print("Connecting to motion controller...")
    pidevice = mtr.connect_pi(config['CONTROLLERNAME'], config['SERIALNUM'], config['STAGES'], config['REFMODES'])
    
    print("Connecting to camera...")
    camera = cmr.connect_camera(500, config['EXPOSURE'])
    output_dir = config['DIR']

    os.makedirs(output_dir, exist_ok=True)
    
    org_width = camera.Width.Value
    org_height = camera.Height.Value
    print(f"Original Camera Resolution: {org_width}x{org_height}")

    # Retrieve increment values
    try:
        offset_x_increment = camera.OffsetX.GetInc()
        offset_y_increment = camera.OffsetY.GetInc()
        width_increment = camera.Width.GetInc()
        height_increment = camera.Height.GetInc()
        
        print(f"Width Increment: {width_increment}, Height Increment: {height_increment}")
        print(f"OffsetX Increment: {offset_x_increment}, OffsetY Increment: {offset_y_increment}")
    except genicam.GenericException as e:
        print(f"Error retrieving increment values: {e}")
        camera.Close()
        pidevice.CloseConnection()
        return

    print("Starting scanning process...")
    for y in range(config['STEP_NUM'][1] + 1):
        for x in (range(config['STEP_NUM'][0] + 1) if y % 2 == 0 else range(config['STEP_NUM'][0] + 1)[::-1]):
            target_x = config['VERTEX']["0,0"][0] + x * config['DX']
            target_y = config['VERTEX']["0,0"][1] + y * config['DY']
            print(f"\nMoving to position: X={target_x}, Y={target_y}")

            try:
                pidevice.MOV([config['AXES']["x"], config['AXES']["y"]], [target_x, target_y])
                pitools.waitontarget(pidevice, axes=(config['AXES']["x"], config['AXES']["y"]))
                print("Stage movement complete.")
            except Exception as e:
                print(f"Error during stage movement: {e}")
                continue  # Skip to the next position

            print("Capturing original image...")
            try:
                org_image = cmr.capture_single_image(camera)
            except Exception as e:
                print(f"Error capturing image: {e}")
                continue  # Skip to the next position
            
            print("Detecting circles...")
            circles = cdt.get_circle(org_image)

            # Skip processing if no circles detected
            if circles is None or len(circles) == 0:
                print("No circles detected, skipping this position.\n")
                # Reset camera settings
                try:
                    camera.OffsetX.Value = 0
                    camera.OffsetY.Value = 0
                    camera.Width.Value = org_width
                    camera.Height.Value = org_height
                    print("Camera settings reset.")
                except genicam.GenericException as e:
                    print(f"Error resetting camera settings: {e}")
                continue  # Skip to the next (x, y) position

            print(f"Detected {len(circles)} circles.")


            for idx, circle in enumerate(circles):
                frame_dir = os.path.join(output_dir, f"position({target_x},{target_y})_cell{idx}")
                os.makedirs(frame_dir, exist_ok=True)  # Directory is created only if circles are detected
                try:
                    x_c, y_c, r_c = circle
                    print(f"\nProcessing Circle {idx}: Center=({x_c}, {y_c}), Radius={r_c}")

                    # Adjusting camera region of interest
                    camera.Width.Value = config['KERNEL_SIZE'][0]
                    camera.Height.Value = config['KERNEL_SIZE'][1]

                    # Calculate the raw offsets
                    raw_offset_x = int(max(0, x_c - config['KERNEL_SIZE'][0] // 2))
                    raw_offset_y = int(max(0, y_c - config['KERNEL_SIZE'][1] // 2))

                    # Adjust offsets to be multiples of the increment
                    adjusted_offset_x = adjust_offset(raw_offset_x, offset_x_increment)
                    adjusted_offset_y = adjust_offset(raw_offset_y, offset_y_increment)

                    # Ensure the adjusted offsets do not exceed the camera's maximum allowed values
                    max_offset_x = camera.OffsetX.GetMax()
                    max_offset_y = camera.OffsetY.GetMax()

                    adjusted_offset_x = min(adjusted_offset_x, max_offset_x)
                    adjusted_offset_y = min(adjusted_offset_y, max_offset_y)

                    # Set the adjusted offsets
                    camera.OffsetX.Value = adjusted_offset_x
                    camera.OffsetY.Value = adjusted_offset_y

                    print(f"Adjusted Camera ROI: Width={camera.Width.Value}, Height={camera.Height.Value}, "
                          f"OffsetX={camera.OffsetX.Value}, OffsetY={camera.OffsetY.Value}")

                    print("Performing autofocus...")
                    fcs.move_to_focus(pidevice, camera, config, (x_c, y_c, r_c))

                    print("Starting image capture...")
                    camera.StartGrabbingMax(10)  # Grabbing 10 images

                    grab_idx = 0  # Initialize grab index
                    while camera.IsGrabbing():
                        with camera.RetrieveResult(2000) as result:
                            if result.GrabSucceeded():
                                img = pylon.PylonImage()
                                img.AttachGrabResultBuffer(result)
                                # Include grab index to ensure unique filenames
                                filename = os.path.join(frame_dir, f"{grab_idx}.tiff")
                                img.Save(pylon.ImageFileFormat_Tiff, filename)
                                img.Release()
                                print(f"Saved image: {filename}")
                                grab_idx += 1  # Increment grab index
                            else:
                                print(f"Grab failed with error: {result.ErrorCode}")

                    camera.StopGrabbing()
                    print("Image capture complete.")

                except Exception as e:
                    print(f"Error processing circle {idx}: {e}")
                finally:
                    # Reset camera settings after processing each circle
                    try:
                        camera.OffsetX.Value = 0
                        camera.OffsetY.Value = 0
                        camera.Width.Value = org_width
                        camera.Height.Value = org_height
                        print("Camera settings reset.")
                    except genicam.GenericException as e:
                        print(f"Error resetting camera settings: {e}")

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
