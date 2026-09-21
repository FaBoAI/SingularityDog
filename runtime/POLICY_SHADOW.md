# 学習済みモデルの無出力診断

`policy_shadow.py`は、保存済みの実センサーログを入力して第28回Aのモデルを調べる。
シリアルやI2Cを開く処理、モーターの有効化・角度送信は持たない。
このモジュールが終了コード0で終わっても、実機駆動の準備完了とは扱わない。

## 配置するもの

- 第28回Aの `model_149.pt` と、対応する `swing_core.py` / `swing_deployment.py`。3ファイルの固定SHA256を照合する。
- PyTorchを導入した独立したPython環境。既存の診断環境から分離する。
- `diagnose`の `events.jsonl` / `summary.json`、12個体と対応する手動校正候補。これらはGit管理外に保存する。

モデルは74入力・12残差出力。SwingCoreを含む対応処理を通して診断用の関節目標を計算する。
入力の `h` は高さではなく、12関節の負荷蓄積量である。

## 実行

以下のパスは実際の非公開ディレクトリへ置き換える。

```sh
python -m singularitydog_hw.policy_shadow \
  --bundle /path/to/verified-model-bundle \
  --capture /path/to/recorded-diagnose \
  --calibration /path/to/calibration-candidates.json \
  --output /path/to/new-private-result \
  --assume-sensor-aligned
```

`--assume-sensor-aligned`は、取付回転を未確認の単位行列、ジャイロ補正を未確認の0とする**診断上の仮定を明示する指定**。
向きの校正を済ませる指定ではない。加速度を正規化しても、元の大きさと偏差を記録する。
推定重力が直立時の向きと逆になる場合も自動反転せず、結果へ警告を残す。

取付方向の候補がある場合は、`--assume-sensor-aligned` を
`--imu-mount-candidate /path/to/private-imu-mount-candidate.json` に置き換える。
2つの指定は排他で、どちらかが必要。候補ファイルの形式は次のとおり。

```json
{
  "schema_version": 1,
  "status": "IMU_MOUNT_CANDIDATE_ONLY",
  "input_frame": "sensor",
  "output_frame": "body_x_forward_y_left_z_up",
  "R_body_from_sensor": [[0, 1, 0], [1, 0, 0], [0, 0, -1]],
  "raw_driver_axes_verified": false,
  "approved_for_runtime": false,
  "provenance": {
    "source": "Operator placement observation and CAD orientation candidate",
    "observation": "Printed X points left; component face points down; raw IC axes unverified"
  }
}
```

この例は「基板のX矢印が犬の左、部品面が床向き」という観察に基づく候補。
体軸はXが前、Yが左、Zが上で、候補は `v_body = R_body_from_sensor × v_sensor` として
加速度と角速度の両方へ適用する。推定重力は `-R × accel / |accel|`。
行列は有限な3×3、直交、行列式+1を許容誤差 `1e-6` で確認する。鏡映、スケール、せん断は拒否し、自動修正しない。

**基板の印字とドライバーが返す生IC軸・回転符号の対応は未検証**。
候補を指定しても校正済み・駆動可能にはならない。候補ファイルの絶対パス、元バイト列のSHA256、
`provenance` と未検証フラグを結果に残す。元の加速度・角速度、加速度ノルム、
9.80665m/s²に対する相対偏差も保持する。加速度やジャイロのバイアス・スケールは補正しない。
`provenance` は非空文字列の辞書とし、schemaにない補正値などの追加フィールドは拒否する。
観測だけで取付回転を確定させず、既存ログの無出力診断で仮説を比較するために使う。

負荷蓄積量は未接続なので、各姿勢に対し `h=0` と `h=1` の2条件を明記して計算する。
いずれも実測負荷ではない。各条件で状態を独立にリセットするため、初回の小さい出力差は継続運転中の負荷感度を意味しない。

取得がCANの無応答で終了したログは既定で拒否する。
`--allow-incomplete-capture`を明示すると、最初のタイムアウトより前の、送受信照合が成立した部分だけをファイル診断に利用できる。
元ログの `INCOMPLETE`、エラー内容、切り取り時刻を結果へ保持し、完全な計測と表示しない。

## 判定と制限

- 送受信バイト、個体、単位、12軸の位置・速度の順序、有限値を照合する。
- 12軸が揃った記録から最大10姿勢を選び、その時刻より前に読み終えたIMUを組み合わせる。
- 関節は `q_model = sign × raw + offset`、速度は `dq_model = sign × raw_velocity`。学習範囲外の入力を隠して切り詰めない。
- CAN値の取得時刻差とIMUの古さを記録する。この記録の再生は、50Hzの実時間制御試験ではない。
- 各姿勢・各負荷仮定で状態を初期化するため、確認できるのは起動時の推論接続であり、連続した閉ループ動作ではない。
- 現在姿勢とモデル目標の差、学習範囲外の軸、IMU仮定を結果に明示する。これらの目標値をモーターへ送らない。

結果は `OFFLINE_COLD_START_SHADOW_ONLY`。モーター出力、校正承認、電源再投入後の原点保証、50Hz制御確認はすべてfalseを維持する。

[実機への20分計画](../docs/deploy-20-minute-plan.md) · [現在の確認状況](../docs/hardware-readiness-20260921.md)
