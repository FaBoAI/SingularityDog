# 実機校正の準備

ここで扱う姿勢保存・校正ツールは、校正候補を自動適用せず、モーター有効化や位置・トルク指令を送信しません。別モジュールの支持付き単関節試験は[専用手順](SINGLE_JOINT_TRIAL.md)で扱います。

12関節をポーズごとにまとめて保存し、基準との差分と再現性を確認する手順は、[ポーズを使った関節校正](POSE_CALIBRATION.md)を参照してください。

## IMUの六面測定

一姿勢の重力ノルムだけでは、加速度の感度誤差とゼロ点ずれを分離できません。六面測定ではセンサーの各軸を上下へ向けて静止データを収集します。姿勢を変える作業ではモーター電源を切り、JetsonとIMUだけを通電します。必要ならIMUを取付台から外して安定した台へ固定し、配線に力をかけない状態で測定します。

ラベル`x+`は「センサーの+Xが上向きで、静止時のX加速度が正」、`x-`はその逆です。Y/Zも同様です。ロボットの前後左右ではなく、センサー出力の軸を表します。基板の印刷と取付方向の対応は別途検証します。

各姿勢で固定してから次を実行します。最初の10秒は安定待ち、その後15秒を保存します。

```sh
cd runtime  # このリポジトリーのルートから実行

# --executeなしなら計画表示のみ。
python3 -m singularitydog_hw.imu_capture --execute \
  --face x+ --settle-seconds 10 --seconds 15 \
  --output "$HOME/singularitydog-logs/imu-six-face/xplus"
```

姿勢と`--face`・保存先を変更して、`x- / xminus`、`y+ / yplus`、`y- / yminus`、`z+ / zplus`、`z- / zminus`も別々に測定します。保存先はGit管理外の新しいディレクトリーです。撮り直す場合は別名を使います。ソフトウェアは指定された向きを物理的に保証できないため、向きの確認と固定が必要です。

各`summary.json`で`RECORDED_NOT_CALIBRATED`、`restore_status: restored`、エラーなしを確認します。値のばらつき、温度、取得周期も記録します。内部のトリム・補正レジスターは読み取るだけで、変更前後の一致を検査します。

6個の実測データが揃ったら、次を実行します。

```sh
python3 -m singularitydog_hw.imu_calibration \
  --face "x+=$HOME/singularitydog-logs/imu-six-face/xplus/events.jsonl" \
  --face "x-=$HOME/singularitydog-logs/imu-six-face/xminus/events.jsonl" \
  --face "y+=$HOME/singularitydog-logs/imu-six-face/yplus/events.jsonl" \
  --face "y-=$HOME/singularitydog-logs/imu-six-face/yminus/events.jsonl" \
  --face "z+=$HOME/singularitydog-logs/imu-six-face/zplus/events.jsonl" \
  --face "z-=$HOME/singularitydog-logs/imu-six-face/zminus/events.jsonl" \
  --output "$HOME/singularitydog-logs/imu-six-face/candidate.json"
```

補正候補は、加速度の各軸オフセットと倍率、ジャイロの静止時バイアスです。各記録の前75%で計算し、最後25%で確認します。データ量・時刻・静止時ばらつき・ラベルの向き・補正量・重力ベクトルの誤差を検査し、同じ記録の使い回しや大きな揺れは拒否します。受入れしきい値は診断用の仮基準です。

出力は`candidate`、`approved_for_runtime: false`のままです。検算に使った最後25%も同じ姿勢の記録なので、独立した物理検証の代わりにはなりません。別の姿勢・時間で再測定し、取付軸も確かめてから採用します。対角倍率以外の軸間誤差・温度補償やジャイロ感度は推定しません。

## 関節と停止の記録

[commissioning.example.json](config/commissioning.example.json)は全12個RS05、脚の足先→胴体順のIDを記録した雛形です。関節名・原点・正方向・実測可動域、IMU取付変換、電源遮断・支持、各モーターの通信断時停止は未検証値として残しています。

```sh
python3 -m singularitydog_hw.commissioning --config config/commissioning.example.json
```

未記入や未検証項目を`blockers`として表示します。重複ID、重複関節名、反転を含む不正な回転行列、NaNや不足した根拠も検出します。雛形で終了コード1となるのは想定どおりです。全項目が揃っても、根拠の内容や実機安全性をこの検査だけで保証するものではなく、駆動許可は出しません。

ID6・9・10の通信タイムアウト設定は読出し失敗のため不明です。他の9個の読み取り値0も通信断時の停止を保証しません。設定値の取得と、実際の通信断に対する停止試験は分けて記録します。

[実機計測結果](../docs/hardware-bringup-20260920.md) · [ランタイムの実行方法](README.md)
