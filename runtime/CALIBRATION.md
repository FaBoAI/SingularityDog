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

6個の実測データが揃ったら、完了した収録ディレクトリーを指定して次を実行します。すべての向きと収録中の静止を現物で確認した場合だけ、`--operator-confirmed-stationary`を付けます。JSONLだけを渡す旧`--face`経路は診断用として残りますが、取得設定・raw/SI換算・トリム・終了復元を監査した結果ではありません。

```sh
python3 -m singularitydog_hw.imu_calibration \
  --capture-face "x+=$HOME/singularitydog-logs/imu-six-face/xplus" \
  --capture-face "x-=$HOME/singularitydog-logs/imu-six-face/xminus" \
  --capture-face "y+=$HOME/singularitydog-logs/imu-six-face/yplus" \
  --capture-face "y-=$HOME/singularitydog-logs/imu-six-face/yminus" \
  --capture-face "z+=$HOME/singularitydog-logs/imu-six-face/zplus" \
  --capture-face "z-=$HOME/singularitydog-logs/imu-six-face/zminus" \
  --operator-confirmed-stationary \
  --output "$HOME/singularitydog-logs/imu-six-face/candidate.json"
```

補正候補は、加速度の各軸オフセットと倍率、ジャイロの静止時バイアスです。各記録の前75%で計算し、最後25%で確認します。データ量・時刻・静止時ばらつき・ラベルの向き・補正量・重力ベクトルの誤差を検査し、同じ記録の使い回しや大きな揺れは拒否します。受入れしきい値は診断用の仮基準です。

出力は`candidate`、`approved_for_runtime: false`のままです。検算に使った最後25%も同じ姿勢の記録なので、独立した物理検証の代わりにはなりません。監査付き経路は、6記録の設定・ソース・デバイス・トリムの一致、raw/SI換算、時刻と連番、復元、温度安定を追加検査します。`capture_audit_verified=true`はこの入力監査を意味し、校正の物理承認ではありません。

補正係数を変えずに検算する場合は、いったん元の支持姿勢へ戻してから6方向を再設置し、別名の`check-xplus`等へ新しく収録します。40VはOff、IMUの取付は固定したままです。機体全体を安全に支持できる治具と既知姿勢がない場合、この追加測定は行わず、加速度bias/scaleは未識別と記録します。センサーを取り外す指示ではありません。

上のコマンドへ次の6指定を追加し、出力を新しい`candidate-independent.json`へ変更します。

```sh
  --validation-capture-face "x+=$HOME/singularitydog-logs/imu-six-face/check-xplus" \
  --validation-capture-face "x-=$HOME/singularitydog-logs/imu-six-face/check-xminus" \
  --validation-capture-face "y+=$HOME/singularitydog-logs/imu-six-face/check-yplus" \
  --validation-capture-face "y-=$HOME/singularitydog-logs/imu-six-face/check-yminus" \
  --validation-capture-face "z+=$HOME/singularitydog-logs/imu-six-face/check-zplus" \
  --validation-capture-face "z-=$HOME/singularitydog-logs/imu-six-face/check-zminus"
```

独立6収録は係数の計算へ混ぜず、各標本の重力ノルムと3成分、向き、ジャイロ残差を固定した係数で検算します。収録や測定列の使い回し、重なった時間区間、温度・設定・トリムの変更を拒否します。合格時は`validation.independent_capture_gates_passed=true`となりますが、基準器の精度、取付角、動的加速度と温度変化の影響、実機への採用は別に確認します。対角倍率以外の軸間誤差・温度補償やジャイロ感度は推定しません。

### 最終reviewのある補正を明示して実入力へ使う場合

独立6収録も通過した候補から、既存のgyro-bias候補を変えずに未審査の拡張ファイルを作ります。リポジトリー直下から実行し、各変数にGit外の実ファイルを設定します。

```sh
PYTHONPATH=runtime python3 -m singularitydog_hw.imu_calibration_review \
  --bias "$DOG_IMU_GYRO_BIAS" --candidate "$DOG_IMU_ACCEL_CANDIDATE" \
  --mount "$DOG_IMU_MOUNT" --output "$DOG_IMU_BIAS_REVIEW_TEMPLATE"
```

出力は外側の`approved_for_runtime=false`を維持し、内側の`accel_calibration_review`は`UNREVIEWED`、現物確認はすべてfalseです。基準器の精度、6方向の静止・再設置、取付不変を現物で確認した人が、`review`の氏名・時刻・理由・`ACCEPT_DIAGONAL_ACCEL_CALIBRATION_INPUT`、4つの現物確認、`external_reference_uncertainty_rad`、補正後ノルムの監視上下限を記入します。不確かさは0より大きく3°以下の明示値が必要です。既存の水平器の精度は未記録なので、この記録だけから不確かさを0へ置き換えません。

加速度の採用はV3の`run_settings.apply_reviewed_accel_calibration=true`と審査済み拡張biasファイルの明示指定を必要とします。正式候補は`prepare_supported_profile.py --bias`へこのファイルを渡し、通常どおり未承認のprofileを作ります。無出力`native_pipeline_benchmark`にも`--gyro-bias`で同じファイルと`--apply-reviewed-accel-calibration`を明示します。診断と正式設定でこの指定が違えば拒否し、既存のbias SHA固定・最終hardware reviewへ結び付けます。デフォルトはrawで、未審査の候補だけでは有効になりません。

ロード時に、候補SHA、元のfit6収録と独立6収録、補正係数と検算の再計算、6つの実入力ソースのSHAを再検査します。候補と全収録は、記録された絶対パスでロード先から読める必要があります。記録を移す場合はその保存先から候補を再作成し、reviewとbias SHAも結び直します。ソースを変更した場合も再審査が必要です。ロードは補正候補自体や生ログを書き換えません。

実入力はセンサー座標で`(raw-bias)*scale`を計算してから機体座標へ回します。取得時刻・鮮度・20ms条件とrawノルム監視は維持し、補正後ノルムにも別の審査済み上下限を使います。モデルと診断の記録へraw/補正後ノルム、補正係数とreview SHAを両方残します。加速度の大きさを毎周期gへ置き換える処理ではありません。重力方向を作る正規化は従来の方向仮説のままで、動的加速度下の融合・温度補償・ジャイロ感度の保証は追加しません。

9月28日の水平器記録はセンサーZ平均−10.6867m/s²、ノルム偏差+8.9745%です。1姿勢では、Zのbiasを−0.8801m/s²・scaleを1とする説明と、biasを0・scaleを0.91765とする説明を区別できません。X/Yを含む3軸bias/scaleや軸間誤差も識別できません。固定行列による回転はノルムを変えず、±2g/±4gのreadbackと換算の一致も絶対校正の完了にはなりません。既存の水平・3方向の記録は取付方向と符号の証拠として再利用し、偏差を一律正規化で隠しません。外部水平器の精度は未記録です。[レンジ比較](../docs/imu-range-crosscheck-20260928.md)・[水平比較](../evidence/imu-level-reference-20260928.json)。

ICM-20948の公称感度は±2gで16,384LSB/g、±4gで8,192LSB/gです。現ドライバーは設定readbackのFSビットからこの換算を選びます。[TDK DS-000189 v1.5、Table 2](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf)。この仕様との一致は、装着した個体の絶対校正の証明ではありません。

## 関節と停止の記録

[commissioning.example.json](config/commissioning.example.json)は全12個RS05、脚の足先→胴体順のIDを記録した雛形です。関節名・原点・正方向・実測可動域、IMU取付変換、電源遮断・支持、各モーターの通信断時停止は未検証値として残しています。

```sh
python3 -m singularitydog_hw.commissioning --config config/commissioning.example.json
```

未記入や未検証項目を`blockers`として表示します。重複ID、重複関節名、反転を含む不正な回転行列、NaNや不足した根拠も検出します。雛形で終了コード1となるのは想定どおりです。全項目が揃っても、根拠の内容や実機安全性をこの検査だけで保証するものではなく、駆動許可は出しません。

9月23日のID6・9・10交換後は、全12台の通信タイムアウト設定を読み取れています。初回の取得値は全台0（無効）でした。その後の左前脚試験でID4・5・6のみ4000ticksへ設定・読戻ししています。通信断時の物理停止時間は未検証です。設定値の取得と、実際の通信断に対する停止試験は分けて記録します。[交換後の記録](../docs/hardware-replacement-check-20260923.md)

[実機計測結果](../docs/hardware-bringup-20260920.md) · [ランタイムの実行方法](README.md)
