# Mono Camera + ROS 2 + PyCuVSLAM

Jetson AGX Orin 專案，使用 cuVSLAM Core 的 Python API；ROS 2 Node 會自行實作。

## 分階段進行

1. ROS 2 Humble、PyCuVSLAM、CUDA 啟動驗證：已通過。見 [階段 1](docs/mono_cuvslam_stage1.md)。
2. FFY-U3-16S2M-S 單目相機 ROS 2 Driver：程式已建立，等待相機與標定資料做實機驗收。見 [階段 2](docs/mono_camera_stage2.md)。
3. 自製 `mono_cuvslam_node` 與圖片序列輸入：使用 `/home/user/2026-06-22VTOL/firefly` 實測，從第二張起跑的 300 張中有 290 張進入 `TRACKING`。內參主點方向和原始時間戳仍待確認。見 [階段 3](docs/mono_cuvslam_stage3.md)。
4. RViz2、TF、rosbag、FPS 與追蹤測試：已通過；修正 ROS 影像封裝並預先解碼後，300 張追蹤 290 張，無 RViz 平均 30.159 張／秒、有 RViz 平均 28.423 張／秒。TF、rosbag 錄製／回放與 RViz 畫面均已驗證。見 [階段 4](docs/mono_cuvslam_stage4.md)。

架構：相機／離線圖片 → `/camera/image_raw`、`/camera/camera_info` → PyCuVSLAM Mono → `/mono_cuvslam/odometry`、`/mono_cuvslam/path`、追蹤狀態與 `/tf`。單目平移沒有絕對尺度；PX4 EKF2 串接需在未來另行解決尺度後進行。

真值比較：可在追蹤完成後自動用 Sim(3) 對齊 `EO_opk_new.csv`，產生有圖例的 XY/Z 疊圖及相機影像同步比較影片。用法見 [Sim(3) 真值比較](docs/sim3_comparison.md)。

IMU 與真實時間戳：已加入高頻 IMU 同步回放與資料檢查；目前 cuVSLAM 單目模式不支援 IMU 融合，詳見 [IMU 輸入與限制](docs/imu_inputs.md)。
