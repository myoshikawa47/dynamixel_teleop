import subprocess
import argparse

def set_camera_parameters(device_path, parameters):
    for param, value in parameters.items():
        command = f"v4l2-ctl -d {device_path} --set-ctrl={param}={value}"
        # print(command)
        subprocess.run(command, shell=True)

def read_parameters_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        parameters = dict(line.strip().split('=') for line in lines)
    return parameters

def main():
    parser = argparse.ArgumentParser(description="Set camera parameters using V4L2.")
    parser.add_argument("--device", required=True, help="Path to the camera device (e.g., /dev/video0)")
    parser.add_argument("--params_file", required=True, help="Path to the file containing camera parameters")

    args = parser.parse_args()

    camera_parameters = read_parameters_from_file(args.params_file)
    set_camera_parameters(args.device, camera_parameters)

if __name__ == "__main__":
    main()
