# STOP代理の送信前遅延とGCの有限比較

R13の500周期では、推論終了から両バスのC++送信関数開始までが通常約0.2msのところ、第124・256・388周期だけ約2.9〜3.2msだった。132周期間隔はGCを疑う材料だが、R13には当該区間のGC時刻がなく、原因は未確定である。[生記録に結び付いた分類](hardware-native-cycle-20260928.md)。

`native_pipeline_benchmark.py` の `--output-dispatch-trace` は、推論終了、主スレッドの起動確認、前後タスク投入、各ワーカー入口と起動確認、C++開始、最初のwriteを時刻別に記録する。主スレッドCPU時間とGC開始・終了の時刻、世代、スレッドIDも有界な配列に保持し、周期終了後にJSON化する。通常の判定や送信条件は変更しない。

追加の `--defer-gc-during-cycles` は、**明示選択した無駆動STOP代理診断だけ**で自動GCを測定周期中に一時停止する比較条件である。最大500周期、`--record-storage trace`、`--output-dispatch-trace`、方策推論を伴う `--mode stop-proxy` を要求する。ワーカー準備後にGCの有効状態と閾値を保存し、測定前に一時停止する。正常終了・例外・中断のいずれも `finally` で元の閾値と有効状態を再設定・読戻し、復元を確認できなければ診断を `ABORTED` にする。明示的な `gc.collect()` まで禁止する機能ではなく、GCイベントと溢れ件数は引き続き記録する。

このオプションは現行ソースの次版候補であり、凍結済みR15キットには含まれない。Jetsonへの配置や実機計測は行っていない。

以下は**計画表示だけ**で、機器や出力先を開かない。

```sh
PYTHONPATH=runtime python3 -B -m singularitydog_hw.native_pipeline_benchmark \
  --mode stop-proxy --cycles 500 --record-storage trace \
  --output-dispatch-trace --defer-gc-during-cycles
```

比較時はGC条件以外のキット、起動、モデル、ポート、CPU設定、送信間隔、記録方式を揃え、両条件とも返信欠落・安全拒否・最終返信と周期完了まで検査する。GCイベントが遅延区間に重なったか、遅延が主スレッドのチェック前・チェック中・タスク投入・ワーカー待機のどこに現れたかを先に判定する。GCが見えないだけでは高速化を証明しない。

過去の別条件では500周期のGC延期で20ms超過が8/500から20/500へ増えた。今回の選択肢は原因を切り分けるための候補であり、Jetsonでの新しい性能値、20ms達成、実出力の承認はまだない。[過去の比較](hardware-native-cycle-20260928.md#r8r12準備処理と500周期への延長)。
