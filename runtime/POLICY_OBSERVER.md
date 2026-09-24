# 状態を保持する無出力方策observer

`policy_observer.py`は、時刻付きのセンサー記録を受け取り、20ms刻みで方策の内部状態を更新する純粋な処理です。デバイスの接続、ライブ収録、モーターへの送信機能はありません。`policy_shadow.py`の姿勢ごとのcold reset診断を残し、連続した状態更新を別に検証します。

**無出力の処理基盤と有限ライブ診断CLIを実装しています。実センサーとの連続接続、実時間50Hzの成立は実機での検証が必要です。** 起立・歩行・校正の承認にはなりません。[Deploy計画](../docs/deployment-roadmap-20260922.md)に沿って次段階を検証します。

## APIと状態

`StatefulPolicyObserver`には、独立したpolicy instance、校正候補、明示したIMU取付候補、速度指令、`h=0`または`h=1`、有限tick数、観測年齢・取得幅の上限を渡します。実方策のロードには、固定したモデル一式のハッシュを検証する`policy_shadow.load_policy()`を利用できます。ロード時のmanifestは呼出側の記録へ保存します。

```python
observer = StatefulPolicyObserver(
    policy, calibration,
    imu_mount_candidate=mount_candidate,
    h_hypothesis=0,
    command=[0., 0., 0.],
    max_ticks=100,
    max_age_ns=10_000_000,
    max_spread_ns=5_000_000,
    gyro_bias_candidate=None,
    torch_module=torch,
)
observer.reset_run(first_tick_ns, warmup_completed=True)
result = observer.consume(snapshot.as_dict())
summary = observer.finish()
```

上の年齢・取得幅は診断例の値で、実機出力の許容値ではありません。`max_ticks`は1〜30000、tick間隔は固定20msです。呼出側でpolicyをウォームアップしてから、`reset_run()`を1回呼びます。各tickで`consume()`を1回呼び、phase・elapsed・指令／yawフィルター・heading error・前回残差を保持します。同じpolicy instanceを複数observerで共有できません。

欠けたtickの追いつき実行、重複tick、非有限値、範囲違反、欠測、古い入力、時刻後退はrunを`INCOMPLETE`にします。再開には明示した新しい`reset_run()`が必要です。予定tick数の前に`finish()`しても`INCOMPLETE`です。完了時の`COMPLETE_NO_OUTPUT_DIAGNOSTIC`は、有限の診断処理が完了した意味です。

出力の`output_allowed`・`motor_output_available`・`approved_for_runtime`・`live_50hz_verified`は常にfalseです。このクラスは壁時計のスケジューラーを持たず、20msのtick番号を守ったことだけで実時間50Hzを認定しません。速度指令ゼロでも内部状態と方策目標は更新されるため、固定姿勢保持やモーター停止の代用にもなりません。

### 区間計測と単独仮説の診断

`StatefulPolicyObserver(..., profile_consume=True)`で、入力コピー・時刻等の検査・入力変換・出典情報のJSON化とハッシュ・tensor生成・モデル呼出し・出力検査と変換・結果作成の8区間を測定します。返り値の`consume_profile`へホスト時間を記録し、途中の失敗は`summary().last_consume_profile`にも保持します。既定は無効で、従来の返り値形式を維持します。センサー取得、呼出側のJSON／ファイル書込み、CAN送信はこの計測に含みません。

ライブCLIの`--hypothesis both`（既定）は独立したh=0/1を共通の20ms期限で計算します。`--hypothesis 0`または`--hypothesis 1`では指定した固定仮説だけを計算し、ログも選択したhで記録します。`--profile-consume`で上記8区間を記録できます。計画のみの表示でも選択と計測の有無が出ます。単独計測を選んでも、入力鮮度・範囲・20ms期限・追いつき禁止は変わりません。

固定hは感度診断の入力で、本番の負荷履歴推定ではありません。従来の2仮説同時試験の失敗を単独条件の結果で置き換えず、両者を別記録にします。[20ms化の方針](../docs/control-20ms-strategy-20260924.md)

この汎用CLIのセンサー収集部分は従来の単一CAN構成です。現在の前後2バス機体で使うには、別途保存している2バス実行ツールへ上記オプションを接続し、配置・検証する必要があります。今回の修正はまだ実機へ配置していません。

## 入力snapshot

`TelemetrySnapshotBuffer.snapshot(tick_ns).as_dict()`を入力します。バッファは検証済みType17の位置・速度と生IMUを保持し、tickまでに受信が完了した値だけを選びます。raw CANフレームや最新個体の照合は上流の責任で、observerは`fresh_identity_match_verified=false`を維持します。

| フィールド | 内容 |
|---|---|
| `status` / `output_allowed` / `blocked_reasons` | `DIAGNOSTIC_READY` / false / 空リストを要求。`BLOCKED`は処理しない |
| `tick_ns` | 20msずつ増加する整数ナノ秒時刻 |
| `motors` | 12軸×位置・速度の24行。各行は`motor_id`・`parameter`・`value`・`unit`・`request_ns`・`received_ns`・`age_upper_bound_ns` |
| `imu` | `frame=raw_sensor`、`accel_m_s2`と`gyro_rad_s`各3値、`read_started_ns`・`read_finished_ns`・`age_upper_bound_ns` |
| 時刻集計 | `oldest_observation_age_ns`・`acquisition_spread_ns`・`receive_spread_ns`を各行から再検算 |
| 上限 | `max_age_ns`・`max_spread_ns`はobserverに渡した固定値と一致が必要 |
| `source_flags` | 任意の出典フラグ。診断のprovenanceへ保持するが、出力許可へ昇格しない |

位置の単位は`rad`、速度は`rad_s`。別IDの同時刻受信と行順の違いは許容します。同じ取得区間の据え置きは値が同一で年齢上限内の場合だけ許容し、新しい取得区間は開始・終了時刻の両方が前進する必要があります。時刻はホスト側の要求／読取区間であり、モーターやIMUが内部で測定した瞬間の証明ではありません。

位置は`sign × raw + offset`、速度は`sign × raw_velocity`で候補順に並べます。範囲外の位置を切り詰めて入力せず、forward前に中断します。観測74値、actor残差12値、SwingCore通過後の目標12値の形状と有限性を確認します。目標境界はfloat32表現の端点と比較し、物理的な許容幅は追加しません。[74観測の列順と倍率](../docs/deployment-roadmap-20260922.md#方策と駆動)も参照してください。

IMU取付候補は[既存candidate形式](POLICY_SHADOW.md)のオブジェクトまたはファイルを指定します。直交行列・行列式+1・出典を検証します。任意の`gyro_bias_candidate`は`imu_fixed_mount_baseline`の未承認なA-fit比較結果だけを受け付け、`R × (raw_gyro − Aのbias候補)`の順に仮適用します。生値・生加速度ノルム・候補の出典を保持し、加速度bias・scaleを補正しません。重力入力は負の正規化比力という仮説で、検証済み姿勢融合ではありません。

`h`は12関節の負荷履歴入力で、高さや温度ではありません。実機の負荷履歴更新器は未接続です。`h=0/1`を別instanceで固定して比較し、`h_measured=false`を保持します。この比較は物理的な出力の上下限保証ではありません。

## 合成入力だけでAPIを試す

以下は`runtime`ディレクトリーから実行します。テスト専用のfake policyと合成snapshotを使うため、PyTorch・モデルデータ・デバイスは不要です。fake backendにはコンパイルのウォームアップがありません。

```sh
PYTHONPATH=.:tests python3 - <<'PY'
from singularitydog_hw.policy_observer import StatefulPolicyObserver
from test_policy_observer import Policy, FakeTorch, calibration, mount, snapshot

observer = StatefulPolicyObserver(
    Policy(), calibration(), imu_mount_candidate=mount(),
    h_hypothesis=0, command=[0., 0., 0.], max_ticks=2,
    max_age_ns=10_000_000, max_spread_ns=5_000_000, torch_module=FakeTorch,
)
observer.reset_run(1_000_000_000, warmup_completed=True)
for tick in (1_000_000_000, 1_020_000_000):
    result = observer.consume(snapshot(tick))
    assert result['output_allowed'] is False
print(observer.finish())
PY
```

## 検証記録

### 保存済み関節位置を起立準備の観点で確認する

`prestand_check`は保存した`joint_snapshot`と元の校正候補から、全12個体の一致、電圧・電流・モデル角度範囲・サンプリング警告を一括表示します。

```sh
python3 -m singularitydog_hw.prestand_check \
  --capture "$HOME/singularitydog-logs/saved-joint-snapshot" \
  --calibration "$HOME/singularitydog-logs/calibration-candidate.json" \
  --output "$HOME/singularitydog-logs/prestand-report-new.json"
```

任意の`--rr-overlay`は、元の校正ハッシュと個体IDに結び付いた右後付け根ID9だけの補正候補です。元ファイルと制御設定を変更せず、符号・他関節も変更しません。名目姿勢との差は診断値で、移動量としてモーターへ送信しません。

出力先はGit外の新規ファイルです。終了コード0はレポートの保存成功を示し、`OFFLINE_SCREEN_BLOCKED`でも0です。`status`と`blockers`を確認してください。通過しても保存済み記録の検査に留まり、`live_readiness`と出力許可は常にfalseです。

### 保存した実センサー記録を再生する

`policy_observer_replay`は、完了した`diagnose`記録を`policy_shadow.load_capture()`で生CANまで検査し、12個体と校正候補を照合してから再生します。不完全な記録を許可するオプションはありません。元の要求開始・受信完了・IMU読取時刻を保持し、各tickまでに読取が完了した値だけをsnapshotへ渡します。

```sh
python3 -m singularitydog_hw.policy_observer_replay \
  --capture "$HOME/singularitydog-logs/complete-diagnose" \
  --calibration "$HOME/singularitydog-logs/calibration-candidate.json" \
  --bundle "$HOME/singularitydog-policy-bundle" \
  --imu-mount-candidate "$HOME/singularitydog-logs/imu-mount-candidate.json" \
  --max-age-ms 100 --max-spread-ms 100 --max-ticks 100 \
  --output "$HOME/singularitydog-logs/observer-replay-new"
```

年齢・取得幅は明示する診断値で、上の100msを出力許容基準にはしません。任意の`--gyro-bias-candidate`は未承認のA-fit候補を指定します。指令は既定でゼロ、`--command VX VY YAW`でも明示できます。

最初のtickは、記録にある各必須ソースの初回読取と12個体照合が終わった時刻を基準にします。以降は20ms刻みで、欠測、古い観測、取得幅超過、モデル範囲外を最初に見つけたtickでその仮説の再生を止めます。初めから欠けているソースも埋めず、最初のtickを拒否します。記録終端を越えて続けることもありません。`h=0/1`は独立instanceで、明示した合成入力によるウォームアップ後に各1回だけresetし、実記録に合成値を混ぜません。

Git外の新規ディレクトリーに`events.jsonl`と`summary.json`を保存します。最初の停止tickと理由、完了tick数、取得条件・モデル・候補・実行ソースのハッシュを残します。収録検査などの開始前の失敗も、新規出力先を確保できた場合は`INCOMPLETE`の要約に残します。途中停止は`INCOMPLETE`と終了コード2で報告し、診断tickを最後まで処理できた場合だけ終了コード0になります。時刻付きのオフライン再生であり、ライブ50Hzの成立や実機駆動の承認ではありません。

### 有限ライブ診断を準備する

`policy_observer_live`はモデルを先にロード・ウォームアップし、単一の`ReadOnlyCAN`で全12個体を校正候補と照合した後、Type17の位置・速度24項目を連続取得します。CAN送信はType0/17だけです。CAN所有threadが共通の2ファイルロックをserial終了まで保持し、921600baud・DTR/RTS=false・exclusiveで開きます。独立したI2C threadは既存ICM20948読取器を使います。IMUの設定レジスターを書き、終了時に復元するため、機器全体に対する純粋な読取だけの操作ではありません。

```sh
python3 -m singularitydog_hw.policy_observer_live \
  --calibration "$HOME/singularitydog-logs/calibration-candidate.json" \
  --bundle "$HOME/singularitydog-policy-bundle" \
  --imu-mount-candidate "$HOME/singularitydog-logs/imu-mount-candidate.json" \
  --max-age-ms 100 --max-spread-ms 100 --max-lateness-ms 5 \
  --max-ticks 100 --max-seconds 10 \
  --output "$HOME/singularitydog-logs/observer-live-new"
```

既定では計画を表示し、デバイスを開きません。実行には`--execute-no-output`を付けます。取得時間は最大10秒、準備待ちは最大2秒、tickは最大500回です。上の100msは診断の年齢・取得幅で、出力許容値ではありません。24要求の応答が揃っても古さ、モデル角度、20ms以内の処理時間を満たす保証はなく、最初の不成立を記録して終了します。

元の要求／読取開始・完了時刻を保存し、さらにキュー投入完了時刻を記録します。予定tickより後に到着した値をそのtickへ遡って使いません。20msの実時間スケジュールで独立した`h=0/1`を各1回resetしてから更新し、追いつき実行・欠測の補完・範囲の切詰めを行いません。どちらかの仮説、producer、キュー、時刻またはログ保存が失敗すれば全producerへ停止を要求します。カーネルI/Oで終了を確認できないthreadやIMU復元失敗は`INCOMPLETE`です。終了待ちと保存には取得予算とは別に有限の待機があります。

Git外の新規ディレクトリーへ生CAN TX/RX、デコード値、生IMU、各tick、最初の停止理由を`events.jsonl`に保存し、`summary.json`へ入力・モデル・実行ソースのハッシュ、boot、終了／復元状態を保存してfsyncします。途中失敗は終了コード2、有限診断の完了だけ0です。完了しても`approved_for_runtime=false`・`live_50hz_verified=false`・`motor_output_available=false`を維持します。このツールにはモーターのenable・停止・目標送信機能がありません。

### 基盤の検査

2026-09-22時点でobserver16件、snapshot buffer16件、既存shadow関連29件、保存記録replay10件の合計71件がPASSしました。状態保持、reset回数、時刻・欠測・範囲、補正順、float32端点、異常後の再利用禁止に加え、未来の読取を使わない再生、個体照合、不完全収録の拒否、停止tickの保存を検査しています。

```sh
PYTHONPATH=.:tests python3 -m unittest \
  test_policy_observer_replay test_policy_observer test_telemetry_snapshot test_policy_shadow test_policy_shadow_mount -q
```

固定した第28回Aの実モデルでも、変化する合成入力を使い、独立した直接呼出しとobserverを比較しました。`h=0/1`各100tick、仮想時間各2秒、ウォームアップ後のreset各1回で、74観測・12残差・12目標・全状態bufferの最大差はすべて0でした。比較許容値は`atol=rtol=1e-6`です。

この結果は合成入力によるオフライン検証です。ハードウェア接続、実センサーの連続入力、CAN負荷、起立・歩行、ライブ50Hzの成立は検証していません。

### ライブ実測で見つかった初回周期の遅延

9月23日の最初の無出力診断は全12台の識別とCAN・IMU取得まで到達したが、最初のtick開始期限を超えて中断した（予定tickから例外記録まで35.23ms。初版は実際の開始検査時刻を別記していない）。実センサーを使う方策forwardは0回、モーター指令も0回。IMU設定の復元と取得スレッドの終了・記録の同期保存を確認した。

初版には、最初のtick時刻を決めた後に2仮説分のpolicy resetを実行する構造があった。改訂版では `prepare_run()` で初期化を済ませた後、`arm_run()` で周期開始時刻を確定する。初期化の開始・終了、周期開始設定、各tickの開始検査を記録する。許容遅延5ms、入力鮮度・取得幅、20ms周期の判定値は変更しない。修正だけで実時間性能を達成したとは判定しない。

修正後の2回目の無出力診断は、初回tick開始遅延0.220659msで5ms以内だった。その後、校正候補から変換した関節角度がモデルの登録範囲外のため、実センサーによるforward前に中断した。全12台識別、57回のCAN問い合わせ、IMU復元・取得スレッド終了・記録保存を確認。実forwardは0回であり、継続50Hz性能の確認には至っていない。
