#!/bin/bash

# デバイスとファイルのリストを定義
devices=(/dev/CAM_HIGH /dev/CAM_LOW /dev/CAM_LEFT_WRIST /dev/CAM_RIGHT_WRIST)
params_file="./config/camera_default.camset"

# forループで各デバイスに対してコマンドを実行
for device in "${devices[@]}"; do
    python3 ./utils/camera_parameters.py --device "$device" --params_file "$params_file"
done
