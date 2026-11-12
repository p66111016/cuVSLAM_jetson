# 階段 3：圖片序列 + PyCuVSLAM 單目 ROS 2 Node

目前不需要實體相機。`image_sequence_node` 讀取 `/home/user/2026-06-22VTOL/firefly` 的 PNG 與 [提供內參轉換的 ROS YAML](../config/firefly_20260622_assumed.yaml)，發布 `/camera/image_raw` 與 `/camera/camera_info`；自製 `mono_cuvslam_node` 直接呼叫 PyCuVSLAM Core 的 `OdometryMode.Mono`。沒有使用 Isaac ROS cuVSLAM Node。

若提供的 `p1/p2` 是 Metashape Frame 模型原始係數，應使用[交換切向係數的 YAML](../config/firefly_20260622_metashape.yaml)：Metashape 的 P1 對應 OpenCV/ROS 的 p2，Metashape 的 P2 對應 OpenCV/ROS 的 p1。從 `firefly_008011.png` 起測前 300 張，舊版與修正版皆為 281 張 `TRACKING`、19 張 `WARMING_UP`；此修正尚未改善追蹤連續性。`xp/yp` 的原始輸出格式仍需確認，不能單憑 JSON 欄位名稱推定主點方向。

資料夾內有 10,968 張 1440×1080 的 8-bit 灰階 PNG，檔名從 `firefly_006799.png` 起。轉換內參時暫採 `cx = 1440/2 + xp = 717.1616`、`cy = 1080/2 + yp = 576.7608`，並視 `f = 517.6153` 為像素焦距。這與 1.8 mm 鏡頭和 0.00345 mm 像素尺寸相符，但 `xp/yp` 的原始座標方向仍須由標定來源確認；若原始定義不同，追蹤結果需重跑。

建議從 `start_index:=1`（`firefly_006800.png`）開始。第一張 `firefly_006799.png` 與第二張的影像內容約旋轉 180°；以第一張起跑，連續 300 張都停在 `WARMING_UP` 且位姿不動。略過第一張後，第 11 張進入 `TRACKING`。

輸出：

- `/mono_cuvslam/odometry`（`nav_msgs/Odometry`，追蹤有效時才發布）
- `/mono_cuvslam/path`（`nav_msgs/Path`，追蹤有效時才更新，最多保留 2000 個姿態）
- `/mono_cuvslam/tracking_status`（`std_msgs/String`：`WAITING_FOR_INPUT`、`WARMING_UP`、`TRACKING`、`LOST` 或錯誤狀態）
- `/mono_cuvslam/processed_stamp_ns`（`std_msgs/UInt64`，離線圖片發布器用來確認處理完成）

姿態座標系 `cuvslam_odom_optical` 在起始相機光學座標方向，x 向右、y 向下、z 向前。單目平移尺度不確定；數值不能當作公尺直接餵給 PX4。TF、RViz2 與 rosbag 屬於階段 4。

## 建置與啟動

```bash
cd /home/user/mono-cuvslam-ros2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mono_camera_driver mono_cuvslam
```

終端 A：

```bash
cd /home/user/mono-cuvslam-ros2
source scripts/ros_env.sh
ros2 run mono_cuvslam mono_cuvslam_node --ros-args -p input_reliability:=reliable
```

離線 PNG 解碼成 `mono8` ROS 影像後，每張約 1.5 MB；這組資料跨程序測試時，追蹤端使用 `reliable` QoS 才能穩定收到完整影像。`image_sequence_node` 的發布端已設定為 `reliable`。日後使用目前的實體相機 Driver 時，其發布端為 `best_effort`，追蹤端應使用預設的 `input_reliability:=best_effort`。

終端 B：

```bash
cd /home/user/mono-cuvslam-ros2
source scripts/ros_env.sh
ros2 run mono_camera_driver image_sequence_node --ros-args \
  -p image_dir:=/home/user/2026-06-22VTOL/firefly \
  -p calibration_file:=/home/user/mono-cuvslam-ros2/config/firefly_20260622_assumed.yaml \
  -p glob:='firefly_*.png' -p start_index:=1 \
  -p fps:=30.0 -p replay_fps:=0.0
```

圖片依檔名中的數字自然排序，會轉成 `mono8`。`fps` 是原始拍攝時間戳的假設頻率，不是播放速度；因資料沒有逐張時間戳，目前暫用 30 FPS。`replay_fps:=0` 代表不限制播放速度，每張影像在 cuVSLAM 完成處理後立即送下一張；不會因 ROS 訂閱端忙碌而跳幀。離線節點預設在背景預先解碼下一張圖片，可用 `-p prefetch:=false` 關閉。可設 `start_index` 與 `max_frames` 做短跑，例如 `-p max_frames:=300`。

若原始拍攝頻率已知，請把 `fps` 改成實際值；不需要為了加速播放而調大它。追蹤品質仍需看這組實際圖片的連續性與內參定義。

## 實際圖片驗證

在 Jetson AGX Orin 本機，以 `start_index:=1`、`max_frames:=300`、`replay_fps:=0` 測試，300 張全部收到處理確認；前 10 張為 `WARMING_UP`，後 290 張為 `TRACKING`，最後位姿相對起點有變化。修正 ROS 影像封裝並加入預先解碼後，跨程序整合測試為 30.159 張／秒；詳見[階段 4 的效能驗證](mono_cuvslam_stage4.md)。資料缺少逐張拍攝時間戳，30 FPS 只用於合成來源時間戳，不能拿來推定實際速度或速度估計。此結果證明單目追蹤能啟動，尚不能證明軌跡的絕對尺度或定量精度。
