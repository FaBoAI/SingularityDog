# 12軸CANの読み取り速度を測る

`singularitydog_hw.can_timing_probe`は、現在のUSB2CANと要求・返信方式で、12軸の位置・速度を読み取る時間を測ります。制御指令の送信、モーターの停止・有効化、CAN設定や自動報告周期の変更は行いません。`full_controller_50Hz_verified`は常に`false`です。

## 計測内容

最初にType0でID1〜12のUIDを非公開の照合ファイルと確認します。全12軸の一致後、各軸について位置(Type17/0x7019)、速度(Type17/0x701B)の順に読み、ID1〜12を1cycleとします。次の要求は、前の要求に対応する返信を受信・検証した後だけ送ります。別の軸への同時要求や人工的な送信間隔の比較はありません。

- 既定10cycle、指定範囲1〜100cycle。要求数は`12 + 24 × cycle数`、最大2412件です。
- 既定30秒、指定範囲1〜120秒の上限があります。期限後は新たな要求を送らず、処理中の要求も残り時間に合わせたtimeoutで打ち切ります。OS・USBの処理停止やファイル書込みの停止を含む厳密な実時間保証はありません。
- timeout、UID不一致、不正・重複・対応しない返信、途中の割り込み、受信データの欠損や残留があれば、その場で不成立として終了します。再送や自動再試行はありません。
- 途中で終了しても、完了した要求とcycle、失敗した要求、残留byte数、終了理由を保存します。不完全なcycleは成功の統計に混ぜません。

## 実行

UIDファイルは、既存の非公開記録から用意するID1〜12をキーとするJSONです。公開リポジトリへUIDやRAWログを置かないでください。

```sh
python3 -m singularitydog_hw.can_timing_probe \
  --expected-uids /private/path/expected-uids.json \
  --output /private/path/new-can-timing-capture \
  --cycles 20 --max-seconds 30
```

上記は計画表示だけで、ポート・lock・出力フォルダーを開きません。実測する場合だけ`--execute-readonly`を追加します。出力先はGitおよびruntimeの外側にある新規フォルダーを指定します。

実測時は他のCAN処理を終了し、既にモーターが無効な状態で実施してください。このツールはモーターを停止できません。既存の`ReadOnlyCAN`と同じ921600bps、DTR/RTS無効、serial exclusive接続を使用します。また`~/.cache/singularitydog/manual-calibration.lock`と`can-readonly.lock`を保持します。lockを使用しない別プログラムまで排除できるとは限りません。

## 結果の読み方

非公開出力フォルダーの`events.jsonl`には生の送受信byte列と時刻、`summary.json`には次を保存します。

| 項目 | 意味 |
|---|---|
| `round_trip_ms` | 既存ReadOnlyCANの要求準備開始から対応返信の復号完了まで。ログ処理も含む |
| `wire_round_trip_ms` | 実際のserial.write呼出直前から、対応フレームを含むread完了まで。USB・OS・ホスト処理を含み、CANバスだけの伝送時間ではない |
| `duration_ms` | 12軸×2readを1cycleとして取得する時間 |
| `oldest_newest_spread_ms` | 同じcycle内の最古・最新の返信受信時刻の差。24値は同時観測ではない |
| `oldest_request_to_newest_reply_ms` | cycle最初のwriteから最後の返信までの幅 |
| `residual_*` | serial受信待ち、parser未完了、parser破棄byte数 |
| `statistics_ms` | RTT、cycle期間、取得時刻差の件数・最小・中央値・平均・p95・最大 |
| `observed_complete_cycle_rate_hz` | 完了cycle全体の経過時間に基づく観測レート。cycle間のログ処理も含む |

50Hz制御の周期は20msですが、この計測に含まれるのは位置・速度の読み取りだけです。方策推論、IMU同期、制御出力、安全判定、スケジューラの余裕は評価しません。読み取りが20ms以内でも、実機の12軸50Hz制御が成立したとは判断しません。

## 2026-09-21の実測

再接続後に全12個体を照合し、20cycleを完了。識別12件・位置／速度480件、計492要求に全応答した。
送受信の生データを別処理で再解読し、Type0/17だけであること、値・対応・件数の一致、各巡終了時の残留／破棄byte数0を確認した。

| 指標 | 中央値 | p95 | 最大 |
|---|---:|---:|---:|
| 要求から復号完了まで | 3.164ms | 3.225ms | 3.503ms |
| 12軸の位置・速度一巡 | 81.204ms | 81.665ms | 81.843ms |
| 巡内の最古・最新の返信時刻差 | 77.741ms | 78.267ms | 78.430ms |

巡回速度は12.32Hzだった。現方式の読み取りだけで20msを超え、50Hz方策への直結は成立しない。
これは一度の短時間・無駆動計測であり、運転中の安定性や通信エラー率の保証にはしない。
