# SingularityDog 実機診断ランタイム

Jetson上で12個のRS05とGY-ICM20948V2を調べる初期立上げ用プログラムです。`diagnose`などの診断・校正ツールはモーターを有効化せず、IMUは計測用設定へ一時変更して終了時に復元します。`rs05_joint_trial`と`rs05_leg_trial`は、支持台と電源遮断担当者を必要とする、明示選択した関節の微小駆動試験です。[単関節試験](SINGLE_JOINT_TRIAL.md)・[脚1本の3関節同時試験](LEG_TRIAL.md)を参照してください。

`policy_shadow`は、取得済みの実センサーログで学習済みモデルの起動時推論を確認するファイル処理です。ハードウェアを開く機能やモーター出力はありません。[無出力診断の使い方と仮定](POLICY_SHADOW.md)を参照してください。

`policy_observer`と`telemetry_snapshot`は、時刻付き入力を選び、20ms刻みで方策の状態を保持する無出力の処理基盤です。合成入力で連続更新を検証済みですが、ライブproducerと実機50Hz検証は未完了です。[API・snapshot形式・合成入力の例](POLICY_OBSERVER.md)を参照してください。

[2026-09-21までの実機集計](../docs/hardware-results-20260921.md)：右前脚3関節の限定駆動・停止まで確認済み。公開時のランタイム全体は325件のオフラインテストを通過しています。試験に用いた個体情報・校正候補・生ログ・重みは同梱しません。

## 接続と取得内容

| 機器 | 接続 | 取得内容 |
|---|---|---|
| RobStride USB2CAN / CH340 | `/dev/robstride-usb2can`、UART 921600 8N1 | Type 0識別、Type 17パラメーター読み取り |
| RS05 ×12 | FR 1–3、FL 4–6、RR 7–9、RL 10–12。各脚の足先から胴体の順 | 出力軸位置rad・速度rad/s、電流A、電源V、run_mode、CAN timeout設定、zero_state |
| GY-ICM20948V2 | `/dev/i2c-7`、`0x68`、WHO_AM_I `0xEA` | 加速度m/s²・角速度rad/s、ホスト時刻、読み取り所要時間 |

機種と脚のID対応はユーザー申告です。ID1〜3は右前脚の足先側・中央・胴体側との対応を動作・目視で確認済みですが、他のIDとモデル上の関節名・原点・正方向は未検証です。`run_mode`は制御モードの設定であり、モーターが有効かどうかの判定には使いません。UART速度とCANバス速度も別です。

[ポーズごとの12関節一括保存・基準との比較](POSE_CALIBRATION.md)を追加しました。取得は`joint_snapshot`の読み取りだけ、名前付きの記録と比較は`pose_record`のオフライン処理です。

[Enterで進める手動校正ツール](MANUAL_CALIBRATION.md): Jetson上で位置の案内に従って手で脚を動かし、Enterで保存。各脚5姿勢を続けて記録できます。

IMUは±2g／±250dps、DLPF23.9Hz、約102Hzの設定を100Hzでポーリングします。センサー座標のまま記録し、取付回転・バイアス補正・姿勢推定・磁気センサーはまだ適用しません。最新値を読む方式で、取りこぼし数や変換の厳密な同期は保証しません。

## 実行

Linux、Python3.10以降、pyserial3.5以降が必要です。ユーザーにシリアルポートとI2Cデバイスのアクセス権を設定してください。実機の検証結果は別の立上げ記録を参照します。

[2026-09-20 Jetson実測結果](../docs/hardware-bringup-20260920.md): 60秒計測とSIGTERM時のIMU設定復元を確認。一部設定読取とIMU測定偏差の課題を記録しています。

```sh
cd runtime
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests

# 実行計画を表示するだけ。デバイスは開きません。
python3 -m singularitydog_hw.diagnose --seconds 60 --output "$HOME/singularitydog-logs/test-001"

# 実機計測。保存先はGit管理外の、まだ存在しないディレクトリーを指定します。
python3 -m singularitydog_hw.diagnose --execute --seconds 60 --output "$HOME/singularitydog-logs/test-001"
```

`--seconds`は1–120秒の収集時間です。初期化・復元処理と進行中の1リクエスト分は別に時間がかかります。二重起動はI2Cの協調ロックとシリアルの排他オープンで防ぎますが、別のI2Cツールを同時に動かさないでください。

新規ディレクトリーは0700で作り、`events.jsonl`と`summary.json`を保存します。生ログには個体識別情報を含むためGit管理外へ保存します。カメラ画像の取得・アップロードはこのプログラムにはありません。

初回に各IDの識別と設定を読み、続いて位置・速度・電流・電圧を全関節で読みます。全関節の走査は2Hzを目標としますが、応答時間に依存し、制御用サンプリング周期ではありません。タイムアウト後はそのCANセッションを打ち切り、遅れた応答を次の要求の結果として扱いません。

全12個の識別・基本データ・IMUデータが揃わない計測は`INCOMPLETE`です。`can_timeout`と`zero_state`だけは、正しい形式の失敗応答を受信した場合に「設定値不明」の警告として基本データの計測を続けます。この場合は`COMPLETE_WITH_WARNINGS`を返します。失敗応答中の0を有効な設定値として扱いません。無応答・壊れた応答・基本データの読み取り失敗は計測中断になります。いずれの完了表示も歩行可能・安全確認済みを意味しません。

## 停止と次の開発

Ctrl-CまたはSIGTERMで収集を止め、IMU設定の復元を試みます。SIGKILL・電源断・バス断では復元できない場合があります。終了コードと`imu_restore_status`を確認してください。CANの停止命令やtimeout設定変更は送信しません。

`safety.py`は期限切れセンサー、デッドマン解除、校正不足などで出力許可を拒否する純粋な判定器です。診断プログラムでは常に非武装です。物理停止、CAN出力、通信断時の実モーター停止を実装・実証したものではありません。

次は物理的な電源遮断手段と機体支持を確認し、IMUの取付軸と12関節の原点・正方向・可動域を校正します。その後、モーター出力を持たない推論を先に評価し、単関節の制限付き試験、支持付き立位へ進みます。

[校正の手順](CALIBRATION.md)に、IMUだけを記録する`imu_capture`、固定したままの2記録を比較する`imu_fixed_mount_baseline`、関節・停止の確認記録を検査する`commissioning`の使い方をまとめています。`imu_orientation_replay`は独立した静止記録Bを、Aで求めたジャイロbias候補の有無で積分比較するオフライン診断です。絶対姿勢や動作時の精度は保証せず、補正を自動適用しません。六面用`imu_calibration`は必要時の参考として残し、今回の固定IMUで必須にはしません。いずれもモーター出力機能はありません。

## 仕様根拠

- [RobStride RS05公式マニュアル](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf) のパラメーター読み取りとデータ型。
- [RobStride USB変換ソフト](https://github.com/RobStride/CAN-USB-data-conversion/blob/main/switch/mainwindow.cpp) のバイナリーATフレーム。
- [TDK ICM-20948データシート](https://invensense.tdk.com/wp-content/uploads/2024/03/DS-000189-ICM-20948-v1.6.pdf)。実装上の制限は[IMUの詳細](IMU_IMPLEMENTATION_NOTES.md)を参照。
- [停止判定器の詳細](SAFETY_IMPLEMENTATION_NOTES.md) · [実機開発方針](../docs/real-world-deployment.md)
