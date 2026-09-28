# 9月28日 IMU ±2g/±4g 比較

QDDの40VをOff、JetsonだけOn、胴体を箱で支持し、取付を変えずに加速度レンジを`2g→4g→2g`の順で各4秒測定した。計測用コードは[`runtime/singularitydog_hw/imu_range_crosscheck.py`](../runtime/singularitydog_hw/imu_range_crosscheck.py)。CANを開かず、校正値も適用しない。各セッション終了時、変更したIMUレジスタは元の値へ復元・読戻しした。

| レンジ | 新規標本 | 加速度ノルム平均 | 生カウントノルム平均 | 復元 |
|---|---:|---:|---:|---|
| ±2g（前） | 386 | 10.67583m/s² | 17836.14 | 確認 |
| ±4g | 381 | 10.67639m/s² | 8918.54 | 確認 |
| ±2g（後） | 392 | 10.67070m/s² | 17827.57 | 確認 |

±4gで生カウントがほぼ半分になり、SI単位での結果は±2gと一致した。レジスタ読戻し、静止性、2g前後の安定性も通過。したがって、今回の約`+8.84%`は**レンジ切替に依存する**設定・換算の取り違えだけでは説明できない。両レンジに共通する絶対倍率の誤差は、この比較だけでは否定できない。**加速度のbias/scaleと機体座標の承認は未完了**。固定した1姿勢だけではbiasとscaleを分離できない。既存の鼻先上げ・左側上げ記録も静止角度の広がりが小さく、6面の代用にしない。外部の水平・重力方向基準、または安全に複数の既知姿勢を作れる条件を先に用意する。

生ログはGit外のJetson `~/singularitydog-logs/imu-range-crosscheck-20260928-r1` とMac `/Users/akira/.codex/private-robotdog-kits/diagnostics-20260928/imu-range-crosscheck-20260928-r1` に保存した。`status=CONFIG_CONSISTENT_ACCEL_NORM_UNRESOLVED`、`approved_for_runtime=false`。診断コードとドライバの単体試験は24件通過した。モデル入力に`10.67→9.81`の比率を無検証で掛けることや、±4gの結果を校正完了として使うことはしない。
