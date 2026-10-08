# 4バス boxed 実Type1 出力経路（追加実装）

4台のUSB2CANに各3軸のRS05を接続した構成で、boxed（箱で支持）の小振幅Type1を
送るための追加実装です。既存の2バス経路と `four_bus_diagnostic/`（STOP専用の診断）は
変更せず、importしてそのまま再利用します。設計は `DESIGN.md` に従います。

**このディレクトリにあるものはどれも出力許可になりません。** ライブラリ、プロファイル、
PLAN、テスト合格、タイミング結果、`COMPLETE_*` の報告は、いずれも承認ではありません。
実機での実行には、毎回次の三つが必要です。

- 現在の起動・電源区間でユーザー本人がチャットで述べた条件の記録（`conditions`）
- 新しい読み取り専用トポロジ取得
- 人が明示的に指定する `--execute`

## ファイル

| ファイル | 役割 |
| --- | --- |
| `subset_active.cpp`, `build.py` | マスク（7/56）に束縛した `sda_subset_validate` / `sda_subset_exchange` を追加します。通常の `transport.cpp` と `subset_stop.cpp` を変更せずに include します。受領記録のスコープは `four_bus_subset_active.v1` で、`output_allowed: false` です。 |
| `type1_transport.py` | 1ポートを担当する3軸の所有者です。<br>・Pythonの許可リストを通し、`sda_subset_exchange` だけを使います。<br>・失敗すると自身を汚染状態にし、`cancel_all` を呼びます。<br>・終了時は `sda_emergency_stop_subset` を最大3回／1秒繰り返します。曖昧な状態は解消せず保持します。 |
| `type1_profile.py` | ファイルだけで動く `prepare`（契約とプロファイルの作成）、人の条件記録 `conditions`、前段報告の検証、`admit()` を提供します。 |
| `type1_runner.py` | OR の A〜J 相を4ポート向けに移したものです。デバイス、モデル、時計はすべて外から注入します。`execute=True` を渡さない限りPLANを返すだけです。 |
| `type1_foreground.py` | CLIの入口です。ピン留め、起動時の読み戻し、ポートロック、キャンセル、シグナル処理、電流ガード、設定の復元、O_EXCL による報告の書き出しを、`four_bus_diagnostic/foreground.py` と同じ型で行います。 |
| `test_*.py` | ファイルだけで完結するテストです（Torch・実機・ネットワークは不要）。 |

## 統合で揃えた接続点

- **PACINGの一本化**: `type1_runner.PACING` と `POST_REPLY_V1` は、`type1_profile` の
  `PACING` と `POST_REPLY_POLICY` をそのまま使います。そのため、実行報告の `pacing` は
  次段が検証する契約の `pacing` と一致します。
- **admitted の変換**: `type1_foreground.runner_admitted(admitted, firmware_by_id)` が、
  `type1_profile.Admitted` を runner 用の平らなマッピングに変換します。
  - 有効境界は契約の値をそのまま使います。
  - オフセットは固定ブランチの値を使います。
  - 初回周期のpost-reply猶予は `False` に固定します。契約に審査済みの選択がないためです。
- **ファームウェアの指紋**: ピン留めした審査済みジオメトリの `artifacts.hardware_review` にある
  `device_watchdog[*].version_bytes_hex` から取ります。これは「現在の版バイトが、
  コマンド途絶ウォッチドッグを試験した版と一致するか」を見る拒否条件として使うだけです。
  旧審査の受理をこちらに流用するものではありません。
- **ネイティブの上限値**: `zero_gain_timing` ではネイティブの kp/kd 上限を 0 にします。
  `learned_boxed` では契約の値（3 / 0.15）を使います。各軸の生位置の範囲は、
  契約の `raw_lower_rad` / `raw_upper_rad` を使います。
- **observer**: `model_bridge.create_guarded_observer` は tick 数を 5 か 501 しか受け付けません。
  そのため `create_type1_observer` が同じ手順のまま、上限を `duration×50+2` に設定します。
  モデルの読み込みは、診断前面と同じ `checked.load(active=False)` です。
- **IMUの姿勢・角速度・重力ノルム検査**: 2バスの `policy.validate_inputs`（OM:243-271）のIMU部分を
  `type1_runner.check_imu_limits` に移しました。拒否するだけで、値を丸めません。
  - 検査内容：`frame == 'sensor'` で補正フラグがないこと、生加速度ノルムが
    `imu_accel_norm_min/max_m_s2` の範囲内、マウント回転後の傾きが `imu_tilt_limit_rad` 以下、
    バイアスを引いて回転した角速度の大きさが `imu_gyro_limit_rad_s` 以下。
  - 加速度入力仮説を選んだ計画では、仮説自身のノルム範囲と補正後の傾きも検査し、生の傾きも別に検査します。
  - 回転とバイアスは、モデル計画の `observer_kwargs` にあるインラインの候補（observer と同じもの）から取ります。
    ファイルは開きません。仮説の補正オブジェクトは前面の `load_model` が読み込み、`accel_correction` で渡します。
  - 実行箇所：有効化前（メタデータ検査の直後）と、すべての周期（ゲイン減少中を含む。OR:1957）。
    違反すると4ポートすべてに同時STOPを送り、その周期の出力は書きません。
  - `validate_admitted` は4つの限界値を必須とし、LP:2083-2087 と同じ範囲
    （傾き 0.01〜0.35、角速度 0.01〜1.0、ノルム 8.8〜11.2 で min<max、幅1.5以下）を要求します。
    契約側の「正の値」より厳しい条件です。
  - 結果は `pre_enable_imu_limit_check` と各周期の `imu_limit_check` に残します。
- **STOP回収の全域化**: `_Supervisor.finish()` は例外を出しません。失敗したポートの未確認IDは、
  アダプタではなく `ids_by_port` から取ります。`run()` の `finally` でも `finish()` を保護し、失敗時は
  全ポート未確認（電源遮断が必要）として、その後の復元とクローズを必ず実行します。
- **ホスト監視スレッドの配置**: 40 ms / 2 ms 周期の `OutputWatchdog` は、CPU4 に固定した
  メインスレッドからは作りません。新しいスレッドは作成元の CPU マスクとタイマースラックを
  引き継ぐためです。IMU 担当スレッド（CPU0..3）上で作り、実スレッドのマスクを読み戻して
  `watchdog_settings` に記録します。実バックエンドでは `[0, 1, 2, 3]`（CPU4 を含まない）でなければ拒否します。
  OR は主スレッドを固定する前に作るので（OR:1456/1633）、同じく制御CPUを避けます。
- **報告**: 前面の `report.json` は `REPORT_SCHEMA` に従い、`REPORT_REQUIRED_KEYS` をすべて持ちます。
  - 送信フラグは、runner のフラグと各 transport の `attempts` の論理和です。
  - `terminal_stop` は12軸すべてが確認でき、曖昧がなく、故障ビットがすべて0の場合に限り確認済みとします。
  - `cancel_requests` に、キャンセルを最初に要求したもの（どのポートか、シグナルか、runnerか）を残します。

## 使い方（`runtime/` で実行）

```sh
# ビルド（既定はPLANで、何もビルドしない）
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.build --output "$FRESH_DIR"
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.build --output "$FRESH_DIR" --build

# プロファイル（既定はPLAN。--prepare を付けたときだけ新しいファイルを1つ書く）
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.type1_profile prepare --mode zero_gain_timing --duration 2 ... --output /abs/fresh/profile.json
# 人の条件記録（ユーザー本人のチャット発言からのみ作る。--record を付けたときだけ書く）
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.type1_profile conditions --user-statement "..." ... --output /abs/fresh/conditions.json

# 前面（既定はPLAN。デバイス、ライブラリ、Torch、モデルは一切開かない）
PYTHONPATH=. python3 -B -m experiments.four_bus_type1.type1_foreground --profile ... --conditions ... --output /abs/fresh/run
# 実行は人だけが行う。外側の電力スコープで包む:
#   tools/jetson_latency_power_scope.py --supported-characterization -- ... --execute
```

前面の入力は、すべて「パス＋SHA256」の組でピン留めします。

- ソースキット：manifest、binding、元のモデルプロファイル
- 認可済みのプロファイルと条件記録
- 現在のトポロジ取得とイベント（プロファイルの `evidence.current_capture` と同一であること）
- Type1ライブラリと受領記録、および3つのソース
- 電流ガードのライブラリ、受領記録、ソース
- 告知用の音声（PCM、8秒以内）
- 任意で、C++ エンコーダのバイナリ

`--mode` と `--duration` は認可済みプロファイルの値と一致しなければなりません。SIGINT、SIGTERM、SIGHUP を
受けるとキャンセルし、STOPを送ります。SIGUSR1 を受けると、エンベロープのゲインを段階的に
下げて終了します。

タイミング検証の準備をする前に、`docs/jetson-best20-settings.md` と
`evidence/jetson-best20-settings.json` を読んでください（AGENTS.md の規則）。条件は次のとおりで、
PLANと報告に明記します。

- 要求間隔 900µs、window 3、release spin 500µs
- main は CPU4、CANの所有者は CPU0〜3、IMU のマスクは 0..3
- タイマースラック 1000ns、スイッチ間隔 100µs
- 周期中はGCを延期する

## テスト

```sh
cd runtime
PYTHONPATH=. python3 -B -m unittest experiments.four_bus_type1.test_subset_active \
  experiments.four_bus_type1.test_type1_transport experiments.four_bus_type1.test_type1_profile \
  experiments.four_bus_type1.test_type1_runner experiments.four_bus_type1.test_type1_foreground
```

`FOUR_BUS_TYPE1_TEST_LIBRARY` を設定すると、ビルド済みのライブラリでsocketpair試験を行います。
設定しなければ一時ディレクトリにビルドします。

`test_type1_foreground` は次の流れを、模擬モーターと仮想時計で通しで確認します。

1. 本物の `admit` を行う
2. 本物の `Type1Transport` クラスを、合成したサブセット交換とSTOPの上で動かす
3. runner を実行する
4. `report.json` を書く
5. `validate_type1_report` で検証する

この流れで、ゼロゲイン2秒から学習2秒への連鎖、モード0応答、曖昧STOP、SIGUSR1、SIGTERM、
ファームウェア不一致を試しています。macOS 上の模擬結果であり、実機のタイミングや出力の
適格性を示すものではありません。Jetson（aarch64）で再ビルドしてSHAを固定し、テストを再実行する
必要があります。模擬ネイティブは各セッションのキャンセルFDを書き込み前に調べ、読み取り可能なら書きません。

`test_type1_runner` は `check_current` と `check_cancelled` を別々に注入します。各安全ゲートには、
それだけが止める注入を個別に用意しています。

- 3つの境界電流検査
- 5か所の Type1 前電圧検査（書き込みとの順序台帳つき）
- 40 ms ホスト監視（主スレッド停止中に4ポートSTOP）
- 起動時と周期内の各 need()

## 実行のはしご（各段で新しい証拠が必要）

1. **zero_gain_timing**（2秒。10秒や20秒も可）：現在の起動での4バス STOP-proxy 501周期診断（`COMPLETE`）が前提です。
2. **learned_boxed 2秒**：同じ契約で `COMPLETE` になったゼロゲイン報告と、12軸すべてのSTOP確認が前提です。
3. **learned_boxed 10秒**：`COMPLETE` の learned 2秒が前提です。
4. **新しい4バス STOP-proxy 501周期診断**：10秒の実行のあとに取り直します。
5. **learned_boxed 20秒**：`COMPLETE` の learned 10秒と、手順4の診断が前提です。

どの段でも、次のものをそのつど用意し直します。

- 前段の終了STOPより後に取った読み取り専用トポロジ
- その取得に基づく新しいプロファイル
- 現在の条件記録

どれかが変わったら、人がその場で記録し直してください。

**この経路は、どの段階でも、立つこと、歩くこと、箱から外すこと、荷重を移すことを許可しません。**
