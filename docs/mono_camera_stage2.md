# 階段 2：單目相機 ROS 2 Driver

Driver 位於 `ros2_ws/src/mono_camera_driver`。預定相機是 Teledyne FLIR Firefly S `FFY-U3-16S2M-S`：USB 3.1、USB3 Vision、黑白、1440×1080、全域快門。這是工業相機協定，不應預設會出現 `/dev/video0`；正式取像使用 Spinnaker／PySpin 後端。V4L2 與 GStreamer 後端保留供其他相機或測試影像使用。輸出 `/camera/image_raw` 和 `/camera/camera_info`，兩者使用相同時間戳與 `camera_optical_frame`。

目前尚無實體相機，因此無法驗證相機識別、實拍 FPS 或內參。

## Spinnaker 準備

Jetson 目前可匯入 `PySpin`，系統 Spinnaker 與 Python 套件版本均為 `3.2.0.62`。接上相機後仍需確認該 SDK 版本能辨識 Firefly；若需更新，從 [Teledyne 官方下載頁](https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads)取得 Ubuntu 22.04 ARM64 的 Spinnaker SDK 與**相同版本**的 Python 3.10 PySpin wheel。官方下載需要登入；請勿只替換其中一項。相機連接時也需確認 USB3 傳輸與使用者權限。

## 建置

```bash
cd /home/user/mono-cuvslam-ros2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mono_camera_driver
source install/setup.bash
```

## 啟動

Firefly S 正式啟動方式（接上相機並完成標定後）：

```bash
ros2 run mono_camera_driver mono_camera_node --ros-args \
  -p source_type:=spinnaker \
  -p width:=1440 -p height:=1080 -p fps:=30.0 \
  -p calibration_file:=/絕對路徑/camera.yaml
```

若有多台 Spinnaker 相機，可再加 `-p camera_serial:=序號`。`fps` 是 ROS 發布的目標頻率；相機的實際曝光、輸出速率及其他 GenICam 參數，接上相機後需確認。官方規格的最高影格率為 60 FPS，不代表 Jetson 到 ROS 的實測 FPS。

無相機時可使用 GStreamer 測試影像驗證 ROS 訊息路徑：

```bash
ros2 run mono_camera_driver mono_camera_node --ros-args \
  -p source_type:=gstreamer \
  -p pipeline:='videotestsrc is-live=true ! videoconvert ! video/x-raw,format=BGR,width=1440,height=1080,framerate=30/1 ! appsink drop=true sync=false' \
  -p fps:=30.0
```

`calibration_file` 讀取 ROS `camera_calibration` 產生的 YAML，必須有 `image_width`、`image_height`、`camera_matrix`、`distortion_coefficients`、`rectification_matrix`、`projection_matrix`。標定解析度必須等於實際影像解析度，否則 Driver 會丟棄該影格。尚未標定時可省略 `calibration_file` 先驗證影像，此時 `/camera/camera_info` 的 `K[0] = 0` 表示未標定，不能當作 cuVSLAM 內參使用。

1.8 mm 焦距和標稱 100° FOV 只能用來選鏡頭，不能直接推算可供 cuVSLAM 使用的內參與畸變；鏡頭裝上後必須以實際的 1440×1080 影像完成標定。

## 驗收

在另一個終端：

```bash
source /opt/ros/humble/setup.bash
source /home/user/mono-cuvslam-ros2/ros2_ws/install/setup.bash
ros2 topic hz /camera/image_raw --qos-reliability best_effort
ros2 topic echo --once /camera/camera_info --qos-reliability best_effort
```

階段 2 通過條件：實體相機連續出圖、FPS 接近設定值、`camera_info.k[0]` 和 `camera_info.k[4]` 為正、影像與內參尺寸相同。因目前沒有實體相機，依使用者要求暫緩這項驗收，改用現成圖片進行階段 3；日後接上相機時仍須補驗。
