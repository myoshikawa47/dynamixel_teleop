# Set camera parameter
execute camset and save settings, eg. C922 Pro Stream Webcam.camset.

## Load defalut parameter
$ bash set_default_parameter.sh


## Load custom parameter
python3 ./utils/camera_parameters.py --device "$device" --params_file "$params_file"
python3 ./utils/camera_parameters.py --device /dev/CAM_HIGH --params_file ./config/
