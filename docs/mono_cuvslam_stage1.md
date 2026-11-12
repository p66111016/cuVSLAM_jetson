# Mono cuVSLAM：階段 1

目標：在 Jetson AGX Orin Industrial、JetPack 6.2.1、Ubuntu 22.04 上，讓 ROS 2 Humble 和 PyCuVSLAM Core 使用同一個系統 Python 3.10，並確認 CUDA 可以啟動。這個階段不建立相機 driver 或 ROS node。

在 Jetson **本機終端**（可輸入 sudo 密碼且可看到 `/dev/nvmap`）執行：

```bash
cd /home/user/mono-cuvslam-ros2
bash scripts/install_mono_cuvslam_stage1.sh
```

腳本會將 ROS 2 Humble 裝到 `/opt/ros/humble`，把 NVIDIA cuVSLAM 17.0.1 的 Jetson Orin／CUDA 12／Python 3.10 wheel 裝到本專案的 `.cache/pycuvslam`，並驗證 ROS、單目模式 API 與 CUDA 暖機。既有 PyTorch 不會被安裝或修改。

重跑驗證：

```bash
bash scripts/check_mono_cuvslam_stage1.sh
```

通過時應看到 `Mono mode: OdometryMode.Mono`、`cuVSLAM CUDA warm-up: OK` 和 `Stage 1 passed`。若在容器或 IDE sandbox 看不到 `/dev/nvmap`，請回到 Jetson 本機終端執行；光是 `import cuvslam` 成功還不足以證明 CUDA 可用。

單目模式可以估計軌跡，但平移的絕對尺度未知；因此未來傳給 PX4 EKF2 前，必須先加入尺度來源或處理。參見 [NVIDIA cuVSLAM tracking modes](https://github.com/nvidia-isaac/cuVSLAM#tracking-modes)。
