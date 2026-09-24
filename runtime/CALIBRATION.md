# 実機校正の準備

ここで扱う姿勢保存・校正ツールは、校正候補を自動適用せず、モーター有効化や位置・トルク指令を送信しません。別モジュールの支持付き単関節試験は[専用手順](SINGLE_JOINT_TRIAL.md)で扱います。

12関節をポーズごとにまとめて保存し、基準との差分と再現性を確認する手順は、[ポーズを使った関節校正](POSE_CALIBRATION.md)を参照してください。

## 必要な関節だけを手で再確認

方向が曖昧な関節は、全脚の校正を繰り返さずに指定できます。次の例は左前・右後の足先側ID4・7だけを、各脚で「L字→足先を約2cm下げる→L字へ戻す」の計6姿勢で記録します。

```sh
python3 -m singularitydog_hw.manual_calibration --execute --legs FL RR --joint calf
```

`--execute`を外すと案内表示だけです。支持台上・脱力状態で手で合わせ、各位置でEnterを押します。L字の下脚は後脚も顔側へ水平に向けます。`--joint`は`calf`・`thigh`・`hip`、`--legs`は記録する脚の順番です。従来の`--leg FR`による5姿勢と、指定なしの20姿勢も使用できます。

選択した関節だけの候補を出力します。同じ脚の他2軸が動きすぎていないか、戻り姿勢、全12台の個体対応も検査します。「完了」は選択範囲の観測完了を意味し、他関節の校正や実機設定は変更しません。

## 現在のIMU：取付位置を維持して調整

2026-09-22のユーザー指定により、GY-ICM20948V2は現在の固定位置を維持します。[固定マウントの調整手順](../docs/imu-fixed-mount-20260922.md)に従い、まず静止基準を2回収録し、ジャイロのゼロ点候補と加速度の再現性を確認します。IMUを取り外す六面測定は、今回の必須手順にしません。

固定一姿勢では加速度の各軸biasとscaleを分離できません。現在姿勢を自動的に水平とみなさず、取付方向と傾き応答は、IMUを装着したまま機体の小さな傾きで別に検証します。補正候補は自動適用しません。

収録2本の比較は`imu_fixed_mount_baseline`で行います。完了・復元・raw換算・設定・トリム・時刻・再現性を検査し、操作者による静止確認と診断基準が揃った場合にのみジャイロbias候補を保存します。どの結果も`approved_for_runtime: false`です。[9月22日の実測比較](../docs/hardware-readonly-20260922.md)では候補の保存まで完了し、加速度の偏差は残っています。

`imu_orientation_replay`では、Aで求めたbias候補を別記録Bに適用し、ジャイロのみの相対積分を未補正と比較できます。絶対姿勢や動作時の校正は検証せず、加速度も補正しません。実行例は[固定マウント手順](../docs/imu-fixed-mount-20260922.md)を参照してください。

## 参考：別途六面測定を行う場合

六面ツールは、別途各軸を上下へ向けられる条件で使うために残しています。現在の固定マウント手順とは別です。一姿勢の記録を6種類のラベルへ複製して使うことはできません。姿勢を変える作業ではモーター電源を切り、JetsonとIMUだけを通電します。配線に力をかけず、センサーを安定して固定できる方法を事前に決めます。

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

9月23日のID6・9・10交換後は、全12台の通信タイムアウト設定を読み取れています。初回の取得値は全台0（無効）でした。その後の左前脚試験でID4・5・6のみ4000ticksへ設定・読戻ししています。通信断時の物理停止時間は未検証です。設定値の取得と、実際の通信断に対する停止試験は分けて記録します。[交換後の記録](../docs/hardware-replacement-check-20260923.md)

[実機計測結果](../docs/hardware-bringup-20260920.md) · [ランタイムの実行方法](README.md)
