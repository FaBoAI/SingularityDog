# 固定IMUの一括確認と翌日の測定

IMUを取り外さずに、静止時のジャイロ偏差、センサーから機体への回転、前後・左右・旋回の符号を一度の手順で確認する。既存の生ADC、SI換算、設定readback、トリム不変、終了時復元の監査を再利用した。設定候補を自動適用する機能やCAN送信は持たない。

## 今回追加したプログラム

| プログラム | 実施内容 |
|---|---|
| `singularitydog_hw.imu_commissioning_capture` | 端末の合図で静止A/B、鼻先上げ、左側上げ、左旋回の5収録を連続実施。既存`imu_capture`を子プロセスとして使い、毎回の設定復元を確認する |
| `singularitydog_hw.imu_commissioning_audit` | 保存記録だけを再解析。Aだけでジャイロbias候補を求め、Bで検算。3動作の往路・復路を機体座標で比較し、軸・符号・軸外成分・不完全記録を一覧化する |

回転候補は次のとおり。機体座標はX前、Y左、Z上、ユーザー申告はセンサーXが犬の左、基板部品面が下向き。

```text
R_body_from_sensor = [[0,1,0], [1,0,0], [0,0,-1]]
gyro_body = R_body_from_sensor * (gyro_sensor - bias_sensor)
specific_force_body = R_body_from_sensor * raw_accel_sensor
```

行列の直交性と行列式+1を検査する。反転・scale混入を拒否する。ただし、数学的に正しい行列であることと現物の取付が正しいことは別であり、後者を動作記録と対応付ける。

## 保存済みログで確認したこと

2026-09-22の2本の静止記録と別々の鼻先上げ・左側上げ記録を再解析した。元ファイルは変更していない。公開するのは以下の集計だけで、生ログ・個別時刻・Macの保管先は公開しない。

| 項目 | 再解析結果 | 扱い |
|---|---|---|
| ジャイロbias候補、sensor XYZ | `[+0.00675946, +0.02270928, −0.00266844] rad/s` | 静止Aだけで推定、Bは推定に使わない |
| Bの補正後ジャイロ平均、body XYZ | `[−0.00017287, −0.00041148, −0.00004744] rad/s` | 同じ取付・温度付近での再現性を支持 |
| 鼻先上げ→戻す | body Yの成分積分 `−16.35° → +22.04°` | 期待符号に整合。軸外成分があるので正確なピッチ角ではない |
| 左側上げ→戻す | body Xの成分積分 `+17.23° → −17.32°` | 期待符号に整合。正確なロール角ではない |
| 左旋回 | 要約ファイルが空 | 完全な取得・復元記録として使えず、未確認 |
| Bの生加速度ノルム | `10.6987 m/s²`、標準重力比約`+9.10%` | 偏差は残る。座標回転でも正規化でも校正済みとしない |

9月27日の別起動ではジャイロX平均が約`+0.0111 rad/s`の記録もある。9月22日のbiasを永久固定するのではなく、翌日の起動・温度で静止A/Bを取り直す。単一姿勢から加速度3軸のbiasとscaleを分離することはできない。

## 翌日の一括操作（約3〜4分）

QDDの40Vは**Off**、JetsonはOn。胴体を安全に支え、IMUの取付を変えない。抵抗や接触がなく小さく傾けられる動作だけを行い、できない動作は`q`で中止する。支持台を外す指示ではない。

以下は配置済みソースの`runtime`がPythonの検索パスに入った環境で実行する。保存先はGitの外で毎回新しい名前にする。

```sh
# まず計画だけを表示。I2CにもCANにも接続しない。
python3 -m singularitydog_hw.imu_commissioning_capture \
  --output "$HOME/singularitydog-logs/imu-commissioning-tomorrow-r1"

# 準備後、同じ一つのコマンドで5収録と集計まで進む。
python3 -m singularitydog_hw.imu_commissioning_capture \
  --execute --motor-power-off --body-supported \
  --output "$HOME/singularitydog-logs/imu-commissioning-tomorrow-r1"
```

1. 静止Aを20秒、静止Bを20秒収録する。各収録前に3秒整定する。両方静止していた場合だけ`y`を入力する。
2. 鼻先上げ、左側上げ、左旋回を1回ずつ収録する。最初の3秒は静止、合図から4秒かけて5〜15°程度動かし、3秒保持、4秒かけて戻し、最後は静止する。大きく持ち上げたり裏返したりしない。
3. 各動作後、指定の方向へ動かして戻した場合だけ`y`を入力する。各23秒、計115秒の収録に、位置決め・Enter操作の時間が加わる。
4. `audit.json`の`checks`と`unresolved`を読む。途中で不完全な収録や復元失敗があれば中断し、後続の動作を自動実行しない。

静止候補だけ先に必要なら`--static-only`を加える。収録は計46秒。これだけでは3軸の動作方向は確認済みにならない。

合図は最初の実IMU標本の時刻に同期し、実際に表示した時刻を保存する。250msを超えて遅れた合図をまとめて発行せず、その収録を中断する。表示どおり手が動いたこと自体は操作者の確認であり、ソフトから観測できるものではない。

## 出力と推論への接続

| 出力 | 用途 |
|---|---|
| `static-a/`、`static-b/`、各動作ディレクトリ | 元のADC・SI・取得設定・復元結果。変更しない |
| `manifest.json` | 使用記録、方向申告、往路・復路の時刻を一括指定 |
| `audit.json` | 各チェック、残件、生加速度ノルム、補正後body角速度 |
| `gyro-bias-candidate.json` | 静止判定を通った場合だけ作成。既存の無出力`PolicyObserver`の`gyro_bias_candidate`へ渡せる形式 |
| `imu-mount-candidate.json` | 既存の無出力推論が受け取れる取付回転候補 |
| `session-incomplete.json` | 中断時の理由と保存済み範囲 |

全出力は`approved_for_runtime: false`の候補である。まず保存ログ上の推論または無出力の実入力診断に渡し、ジャイロbias有無による推論入力・出力の差を比較する。新しい設定を無断で実駆動へ昇格させない。

保存後の再集計はハードウェアを開かない。

```sh
python3 -m singularitydog_hw.imu_commissioning_audit \
  --manifest "$HOME/singularitydog-logs/imu-commissioning-tomorrow-r1/manifest.json" \
  --output "$HOME/singularitydog-logs/imu-commissioning-tomorrow-r1/audit-replay.json"
```

`raw_accel_norm_within_3_percent`は既存の約9%偏差ではfalseになる。falseを消すために一律scaleを掛けない。正規化した`−R a / |a|`は静止時の重力**方向候補**としてのみ比較し、生ノルムの異常を併記する。動作時には並進・振動の加速度が重なるため、フィルターの遅れや採用条件を検証した姿勢推定が必要になる。加速度bias/scaleを特定する場合は、別途複数の既知姿勢（既存の六面校正など）が必要。

### 現起動のbiasと不足する左旋回だけを測る短縮手順

9月22日の鼻先上げ・左側上げの完全記録は別の証拠として保存済みで、取付を変更していなければ物理レビューに再利用できる。biasは起動・温度で変わり得るため、現起動では静止A/Bを取り直す。9月22日の左旋回記録は不完全なので、左旋回だけ追加する。40Vを**Off**、胴体を支持したまま、抵抗・干渉なしで機体を小さく旋回して元へ戻せる場合に限る。

```sh
python3 -m singularitydog_hw.imu_commissioning_capture \
  --yaw-only --output "$HOME/singularitydog-logs/imu-static-yaw-NEW"

python3 -m singularitydog_hw.imu_commissioning_capture \
  --execute --motor-power-off --body-supported --yaw-only \
  --output "$HOME/singularitydog-logs/imu-static-yaw-NEW"
```

新規保存先を一度だけ指定する。実収録は静止A/B各23秒と左旋回23秒の計69秒で、操作時間が別途かかる。`audit.json`で`gyro_bias_candidate`と`turn_left_axis_sign`を確認し、元データの設定復元・ハッシュ・静止Bの補正後ジャイロを監査する。`nose_up_axis_sign`と`left_side_up_axis_sign`はこの短縮記録では**falseのまま**で、9月22日の記録を新しい静止Bと同一起動で測ったように混ぜない。旧記録のレビューと取付不変の確認が別に必要。

現行の実出力プロファイル雛形のIMU加速度ノルム範囲は9.4〜10.2m/s²であり、旧静止実測の10.654〜10.737m/s²を通さない。新A/Bの生ノルム最小・最大を確認し、約9%偏差の理由と監視範囲を明示してレビューする。範囲を広げるだけで加速度bias/scaleや重力方向の精度が確定するわけではない。実出力レビューは別途、機体の水平基準と時刻対応した独立の重力方向計測を必要とする。既存の`prepare_imu_review.py`は新一式から未承認の数値雛形を作れるが、物理確認フラグを自動でtrueにしない。

実出力モデルは生ノルムを監視し、`gravity_body = -R a_sensor / |a_sensor|`を入力する。正の一律倍率ならこの方向計算で打ち消し合うため、**重力方向入力だけを目的にした一律scaleの適用は不要**。一方、biasや軸別倍率の誤差は方向にも残る。9月22日の鼻先上げ・左側上げログは動作時にノルムが約9.72〜11.60m/s²、約9.20〜12.00m/s²まで動き、独立に測った既知角度もない。3つの姿勢名だけから3軸biasと等方scaleを同時に決めたり、動作時の加速度を静止重力として採用したりしない。

左旋回の符号が確認できても絶対方位は決まらない。今回のモデルで使うbody角速度と重力方向の確認、絶対yawの校正、六面加速度校正を同じ「完了」にまとめない。

## オフライン検証

`test_imu_commissioning_audit.py`は合成ADCからの換算・独立静止記録・body変換順序・反転行列拒否・逆方向動作・無確認記録・空の要約・窓の重複・生値改変・非静止bias・保存先の上書き拒否・合図遅延・計画時の無I/Oを検査する。実機での動作確認や姿勢推定の精度検証に代わるものではない。
