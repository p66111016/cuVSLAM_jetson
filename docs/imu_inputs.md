# IMU 與真實影像時間戳輸入

目前完成的是離線感測資料輸入：相機按 CSV 中的實際時間戳發布，原始 IMU 按時間切成每張影像以前的樣本發布到 `/imu/data_raw`。**現有 cuVSLAM Mono 不會融合 IMU，軌跡仍是無絕對尺度的單目 VO。**

## cuVSLAM 限制

專案安裝的 cuVSLAM 17.0.1 只在 `Inertial`（雙目＋IMU）和含 IMU 的 `Multisensor` 模式做融合；後者仍需要至少一組雙目或 RGB-D。單張 Firefly 影像沒有深度，不能直接切換模式，也不能複製同一張影像當成雙目。

來源：[版本 17.0.1 tracking modes](https://github.com/nvidia-isaac/cuVSLAM/blob/v17.0.1/README.md#tracking-modes)、本機 `.cache/pycuvslam/cuvslam/pycuvslam.pyi`、`tracker.py`。

真正的單目 VIO 需要支援單目＋IMU 的後端，或補上 cuVSLAM 支援的感測器組合。融合之前還需要相機與 IMU 的外參、軸向、單位、時間偏移及 IMU 雜訊／bias 模型；目前不假設相機光學軸與飛控 IMU 軸相同。

## 此次資料檢查

`/home/user/2026-06-22VTOL/EO_opk_new_with_timestamp.csv` 有 9,756 筆，其中 1,309 筆時間戳空白，其餘 8,447 筆全部是 `1.78209E+15`，已失去逐影像時間精度，不能從中恢復 IMU 同步。

改用既有 `IMU/EOP_firefly_unix_aligned_imu.csv` 的 `Label` 與完整整數微秒欄 `unix_utc+8`。僅讀檔名及時間，不把 EO 位置或姿態當作估計輸入。有效範圍是 `firefly_008011.png` 至 `firefly_016457.png`，影像時間間隔中位數約 99.947 ms（約 10 Hz），不是舊流程假設的 30 Hz。

使用 `IMU/imu_200hz.csv` 原始高頻樣本，而非每張影像一筆的對齊插值 IMU。全有效影像區間加 1 秒前置資料的最大 IMU 取樣間隔為 10.561 ms；前 300 張含 6,157 筆 IMU。完整 IMU 檔案在影像區間之外有長時間斷點，因此只檢查選取區間及前置資料。

時間以 Decimal 讀取整數微秒再轉 nanosecond，不經浮點數。兩檔使用原有同一數值時間基準，不因 `unix_utc+8` 名稱額外加減 8 小時。數值重疊與時間單調不等於已驗證硬體同步或曝光時間偏移。

## 執行同步發布＋既有單目追蹤

```bash
cd /home/user/mono-cuvslam-ros2
source /opt/ros/humble/setup.bash
(cd ros2_ws && colcon build --symlink-install --packages-select mono_camera_driver mono_cuvslam)
source scripts/ros_env.sh

# start_index 按實際圖片檔案排序計算，避免用 CSV 行號或假設圖片連續。
start_index=$(/usr/bin/python3 - <<'PY'
from pathlib import Path
from mono_camera_driver.sequence_node import natural_key
files = sorted(Path('/home/user/2026-06-22VTOL/firefly').glob('firefly_*.png'), key=natural_key)
print(next(i for i, p in enumerate(files) if p.name == 'firefly_008011.png'))
PY
)
ros2 launch mono_cuvslam stage4_offline.launch.py \
  start_index:="$start_index" max_frames:=300 \
  timestamp_file:=/home/user/2026-06-22VTOL/IMU/EOP_firefly_unix_aligned_imu.csv \
  imu_file:=/home/user/2026-06-22VTOL/IMU/imu_200hz.csv \
  calibration_file:=/home/user/mono-cuvslam-ros2/config/firefly_20260622_metashape.yaml
```

`timestamp_file` 與 `imu_file` 預設空白，保留舊的純影像流程。指定 IMU 時必須提供真實影像時間戳；缺值、倒序、重複、涵蓋不足或區間內 IMU 間斷超過 `max_imu_gap_s`（預設 0.05 秒）會直接停止。`max_frames:=8447` 可選完整有效影像範圍；超出對應表範圍會報錯。

`imu_warmup_s` 預設 1 秒；第一張之前的樣本隨第一張發布，後續逐張發布 `(上一張時間, 這張時間]` 的所有樣本。回放速度由原有 `replay_fps` 控制，預設等待 tracker ack 後盡快發布；這是離線批次回放，不是依壁鐘維持 IMU 200 Hz，也沒有發布 `/clock`。ROS 不保證跨 topic callback 接收順序；後續 VIO 後端必須緩存 IMU 並按時間排序，等足夠 IMU 到齊才處理影像。

`/imu/data_raw` 使用 reliable QoS、depth 1000。`imu_frame_id` 預設 `imu_link`，保留原始 IMU 軸及含重力的加速度，不產生未標定的 camera-to-IMU TF。不提供姿態（orientation covariance 第一項為 -1）；gyro／accel covariance 全零表示未知。預設假設原始角速度為 rad/s、加速度為 m/s²，使用前需核對記錄來源；若是 deg/s 或 g，分別設定 `imu_gyro_scale:=0.017453292519943295` 或 `imu_accel_scale:=9.80665`。

舊 `run_with_ground_truth.py`、Sim(3) 對齊和影片工具仍以固定 `source_fps` 推算影像索引，**不要用它們解讀真實時間戳回放結果**。這次沒有把真實時間戳結果接進舊的固定 FPS 比較工具。

驗證：`/usr/bin/python3 -m unittest discover -s tests -v`。建置與資料讀取不需要 GPU；完整 cuVSLAM 追蹤需要可使用 `/dev/nvmap` 的 Jetson 環境。

本次驗證：兩個 ROS 套件 colcon 建置成功、4 項單元測試通過；`tests/verify_imu_ros_replay.py` 的 ROS 回放收到 3 張影像、239 筆原始 IMU，時間戳與數值均和來源一致。沙箱 UDP transport 出現權限警告，但同程序 executor 的訂閱發布測試通過；這不代表已驗證跨程序 DDS、rosbag 或 VIO 融合。
