# Sim(3) 真值對齊、疊圖與影片

真值來源是 `/home/user/2026-06-22VTOL/EO_opk_new.csv` 的 `Label,X,Y,Z`。程式先用圖片序列時間戳算出來源影格索引，再以實際檔名配對 `Label`；不會把有效姿態筆數當成影格索引，因此中途暖機缺口不會導致後續配對錯位。

## 一次執行追蹤與成果製作

```bash
cd /home/user/mono-cuvslam-ros2
source scripts/ros_env.sh
/usr/bin/python3 scripts/run_with_ground_truth.py \
  --start-image firefly_008011.png \
  --output-dir results/new_run_with_truth
```

輸出資料夾必須尚未存在。預設從指定圖片跑到最後，結束後自動停止 ROS launch，依序執行 Sim(3)、繪圖和製片。可用 `--max-frames 300` 短跑、`--skip-video` 只產生數據與圖片。`--image-dir`、`--ground-truth`、`--calibration-file`、`--source-fps` 可更換來源；目前 launch 的圖片篩選仍為 `firefly_*.png`。`--source-fps` 是來源合成時間戳頻率，不是效能上限。

## 重新處理既有結果

```bash
cd /home/user/mono-cuvslam-ros2
/usr/bin/python3 scripts/align_trajectory_sim3.py \
  --image-dir /home/user/2026-06-22VTOL/firefly \
  --start-image firefly_008011.png \
  --trajectory-csv results/from_008011/trajectory.csv \
  --metrics-json results/from_008011/metrics.json \
  --ground-truth /home/user/2026-06-22VTOL/EO_opk_new.csv \
  --first-pose-frame 9 \
  --output-dir results/from_008011/comparison

/usr/bin/python3 scripts/plot_trajectory.py \
  results/from_008011/comparison/trajectory_aligned.csv

/usr/bin/python3 scripts/render_trajectory_video.py \
  --image-dir /home/user/2026-06-22VTOL/firefly \
  --start-image firefly_008011.png \
  --trajectory-csv results/from_008011/trajectory.csv \
  --metrics-json results/from_008011/metrics.json \
  --alignment-dir results/from_008011/comparison \
  --output results/from_008011/comparison/camera_trajectory_comparison.mp4
```

新結果的 `metrics.json` 有 `first_ack_stamp_ns`，不需要 `--first-pose-frame`。上例舊結果的 9 是依日誌與短序列重跑推定，並非舊測試實際記錄的第一張時間戳；其誤差值與影片同步受這個假設影響。

## 詳細結果與 Excel

`run_with_ground_truth.py` 現在會在 Sim(3) 完成後自動執行 `export_run_report.py`。也可以重新處理舊結果：

```bash
cd /home/user/mono-cuvslam-ros2
/usr/bin/python3 scripts/export_run_report.py --run-dir results/from_008011
```

`comparison/results_report.json` 與 `results_report.txt` 包含既有全部量測、追蹤覆蓋率／連續段、軌跡長度、Sim(3) 和位置誤差的 mean/median/std/min/max/P90/P95/P99/RMSE。相鄰有效影格才計算一步相對平移向量誤差，缺口不連接。這些誤差不包含無姿態影格，需同時查看覆蓋率。

`comparison/per_frame_results.xlsx` 有 Summary、Frames、Definitions 三頁，並附同內容的 `per_frame_results.csv`。Frames 每張影像一列，含圖片檔名、合成奈秒時間戳（以文字保存完整精度）、狀態及其來源、是否有姿態／真值、原始與對齊後 XYZ／四元數／RPY、真值 XYZ／OPK、XYZ 分量差、3D／水平／垂直位置誤差與一步相對平移誤差。

新追蹤節點另發布帶時間戳的 `/mono_cuvslam/frame_diagnostics`，由量測節點寫入 `trajectory.frames.csv`，保存每張圖片的確切狀態、callback 處理毫秒、cuVSLAM track API 呼叫毫秒與完成時的系統時間戳。前者不含 PNG 解碼／ROS 傳輸等待；後者不是純 GPU kernel 時間。報告包含這些時間的統計。影像來源時間戳依然是 source_fps 合成，並非相機硬體時間戳。

舊結果沒有逐張診斷時，不捏造各張 WARMING_UP/LOST 或耗時；有 Odometry 的影格標示 TRACKING，其餘標示 NO_POSE_STATUS_UNAVAILABLE。EO 的 OPK 軸向／旋轉順序未確認，故僅保留真值 OPK 與對齊後 RPY，orientation_error_deg 留空；不能直接逐項相減。Excel 使用標準 OOXML 格式，不需額外安裝 Excel Python 套件。

## 輸出與座標

- `trajectory_aligned.csv`：每筆姿態的來源影格索引、檔名、對齊後 XYZ 與四元數。
- `ground_truth.csv`：同一播放範圍的完整真值，含無有效追蹤的影格。
- `alignment.json`：尺度、旋轉矩陣、平移、成功配對數、RMSE/median/P95/max 位置誤差及同步方式。
- `trajectory_xy_comparison.png`、`trajectory_z_comparison.png`：橘色真值、藍色 cuVSLAM，含圖例。
- `camera_trajectory_comparison.mp4`：完整真值為背景，cuVSLAM 逐格疊上，包含原圖、XY、Z、XYZ、Roll/Pitch/Yaw 與四元數。

對齊使用 Umeyama 最小平方解，求一組全域 `p_gt ≈ scale * R * p_cuvslam + t`，限制正尺度及 `det(R)=+1`，不允許鏡射。所有有效位置配對共同擬合；不對不同追蹤段分別擬合，不插值填補缺口。至少需要三個不共線的位置。

XYZ 使用真值的座標與單位；XY 靜態圖為便於閱讀減去第一筆真值 XY，Z 保留真值的 Z 座標。影片數值是對齊後的絕對 XYZ。姿態旋轉只左乘 Sim(3) 的旋轉部分，四元數不套用尺度；顯示的是該真值座標系中的 Roll/Pitch/Yaw，沒有把 EO 的 Omega/Phi/Kappa 當成同一角度慣例進行比較。

位置誤差是在用來擬合的相同配對上計算；真值只供離線評估與視覺化，沒有回饋 cuVSLAM 追蹤，也不能修正尺度隨時間漂移或形狀變形。

已驗證：已知 Sim(3) 的合成數據可恢復尺度、旋轉與平移；旋轉後四元數與禁止鏡射的檢查通過；60 張實際圖片的自動流程產生了精確時間戳配對、疊圖與可讀影片。
