# 全12軸・同一起動の立位姿勢読取り

`singularitydog_hw.fixed_stance_readonly_capture` は、前後2台のUSB2CANから全12軸のUIDと出力軸の生角度を読みます。モーター有効化・STOP・原点変更・目標値送信は行いません。既定は計画表示だけです。

Jetsonで実行する前に、前後の `/dev/serial/by-path` が現在も正しい基板を指していることと、12 UIDの非公開ファイルを確認してください。以前の記録では前側が `...2.4:1.0-port0`、後側が `...2.2:1.0-port0` でしたが、USB差し替え後は変わり得ます。UID照合に不合格なら記録は未完了になります。

```bash
PYTHONPATH=/home/jetson/singularitydog-tests/current/runtime python3 -m singularitydog_hw.fixed_stance_readonly_capture \
  --front-port /dev/serial/by-path/FRONT_BY_PATH \
  --rear-port /dev/serial/by-path/REAR_BY_PATH \
  --expected-uids /home/jetson/singularitydog-tests/fr-toward-stance-r11/expected-uids.json \
  --output /home/jetson/singularitydog-logs/fixed-stance-capture-UNIQUE
```

読取りを実行する場合だけ、同じコマンドに `--execute-readonly` を追加します。非公開UIDと姿勢はGit外の新しい出力ディレクトリに保存されます。Jetsonの実際の配置に合わせて `PYTHONPATH`、ポート、UIDファイルを置き換えてください。

ID照合後、四脚を**同時に**一つの姿勢で保持できたときだけEnterを押します。重力で四脚を同時に保持できず、一脚ずつL字に合わせる場合は `q` で中止してください。一脚ずつの別時刻の角度を統合して「同時の立位姿勢」として扱うことはできません。取得後も姿勢の実測・写真と一緒にレビューが必要です。

成功時の `summary.json` は起動ID、12 UID、各位置/速度/電流/電圧/制御モードの3回の読取り値と時刻、最初から最後の取得時間を記録します。`capture-draft.json` は候補の12生角度を抜粋し、元のsummaryのSHA-256で結びます。どちらも現在のSTOP状態、全動作経路の干渉なし、校正済みモデル角度、荷重支持、自立を証明しません。`run_mode=0` もSTOPの証明にはなりません。状態フラグは未検証・出力不可のままです。
