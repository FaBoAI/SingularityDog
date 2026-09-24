# 無駆動でCANの並行読み取りを計測する

`runtime/singularitydog_hw/can_pipeline_probe.py` は、1回の実行につき1組の `window`、`gap-ms`、`request-order` を計測する診断ツールです。モーター有効化・停止・設定書込み・CAN速度変更は実装していません。実行前に別のCAN利用者がいないこと、モーターがすでに無効であることを確認します。読み取りでは稼働中のモーターを停止できません。

```sh
# runtimeディレクトリーから。まず計画のみを表示する。
python3 -m singularitydog_hw.can_pipeline_probe \
  --window 4 --gap-ms 1 --cycles 5 --max-seconds 10 \
  --expected-uids /private/path/expected-uids.json \
  --output /private/path/new-pipeline-capture
```

`--execute-readonly` を付けた場合だけポートを開きます。計画表示では出力ディレクトリーやlockも作りません。UID照合ファイルはどちらの場合も読み込みます。

| 設定 | 範囲・意味 |
|---|---|
| `--window` | 1–12。未完了要求の最大数。既定1 |
| `--gap-ms` | 0–5ms。前回write完了から次回writeまでの最小間隔。既定0 |
| `--request-order` | `interleaved`（既定）：各IDの位置→速度。`by-parameter`：全IDの位置→全IDの速度。初回の個体照合はどちらも逐次 |
| `--cycles` | 1–20巡。1巡は12 ID × position/velocityの24要求。既定1 |
| `--max-seconds` | 1–30秒。接続開始前からの処理予算。既定10秒 |
| 個別要求 | 最大250ms、または全体予算までの短い方。再送なし |

最初に12個体をType0で**逐次照合**します。その後、各巡の位置・速度をType17で読み、受信可能なデータを処理してから空いたwindowへ次の要求を送ります。返信はIDとparameter indexで対応付けるため、返答順序が変わっても集計できます。人工的な待ち時間中も受信を継続します。

通信は `/dev/robstride-usb2can`、UART 921600bps、8N1、DTR/RTS無効、serial exclusiveです。`manual-calibration.lock` と `can-readonly.lock` を取得します。協調lockを使わない別プログラムの排除を保証するものではありません。依存は既存 `can_readonly.py`、`can_timing_probe.py`、pyserialです。

欠落によるtimeout、不正フレーム、不正値、未要求の返信、同巡で応答済みキーの重複、未完了キーの二重要求、巡の境界に残るbyteがあれば終了します。実際のwrite直前にも許可した標準読取byte列との一致を検査します。失敗後の再試行・別profileへの自動切替はありません。SIGINT/SIGTERMでは処理を打ち切り、ポートとlockを閉じます。

read/writeのtimeoutは残り全体予算と最も早い未完了要求の期限以下に制限します。ただしOSの停止、ポートopen、ファイル書込み、ドライバーの異常停止まで含めた厳密な実時間保証はありません。

保存先はGit/runtime外の新規privateフォルダーです。`events.jsonl` は実write前・後、read直後の時刻と生フレームを記録し、`summary.json` は要求ごとのRTT、巡の所要時間・最古最新の取得幅、要求／応答数、未完了要求、途中終了理由を保存します。途中までの巡は完了巡の統計に混ぜません。UIDと生フレームは標準出力へ出しません。

r2では各要求に`write_expected_bytes`、`write_returned_bytes`、`write_call_entered`を追加。17byte未満のwriteは失敗とし、例外やwrite前の拒否を区別します。17byte返却はシリアルへの受付結果で、CANバスへの送達やモーター受理の証明ではありません。

**照合の限界：** このプロトコルにはtransaction sequence番号がありません。各巡の完了・受信残留なしを確認しても、境界確認後に届く前巡の遅延返信が、次巡で新たに送った同じID/parameterに一致する場合は完全には識別できません。この検査の成功を実機制御の鮮度保証や50Hz制御の成立とは扱いません。`full_controller_50Hz_verified` は常に `false` です。

基準比較は [逐次読み取り計測](can-readonly-timing-probe.md)、CAN/UARTの違いは [1Mbps調査](jetson-usb2can-1mbps-research-20260922.md) を参照してください。

## 2026-09-22の実測

初回r1ではwindow1/gap0が中央値82.156ms、window4/gap1msが37.398ms、いずれも20巡・492要求／492返信だった。window4/gap0は最初の巡で2返信が未取得となり、250ms期限で終了した。

r2の追加診断では、gap1msの要求順変更に大きな差はなく（交互37.568ms、parameter順37.260ms）、parameter順・gap0.5msは中央値26.639ms、最大27.333ms、20巡・492要求／492返信だった。gap0.25msでは3返信が欠落し、最初の巡で終了。その後の短縮試験は行っていない。全条件のwrite返却は17byteだった。0.5msも短い読み取り試験の成功であり、運用条件の保証ではない。全体制御の20ms周期は未達。[測定条件・監査・限界](hardware-readonly-20260922.md)

1巡24要求で毎回1msを空ける方式では、巡内の23区間だけで23ms以上となり、writeや応答処理を加える前から20msを超える。要求順の比較は欠落原因の切り分けであり、それだけで20ms達成とはならない。次の時間短縮は、送信間隔・返信欠落・取得値の古さを一組で測って判断する。欠落した値を前回値で埋めたり、timeoutを長くして見かけの成功へ変えたりしない。
