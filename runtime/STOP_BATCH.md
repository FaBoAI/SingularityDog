# 停止命令のまとめ書込みを比較する

**2026-09-25実機結果：** 前側で1／2フレーム条件は完走、3フレーム条件はID3返信欠落。その後の通信停止はUSB抜き差し・40V Off／On後に復旧しました。6フレーム／前後同時条件は未実施です。[結果と制限](../docs/hardware-stop-batch-20260925.md)。2フレームを含め、実制御へ採用済みではありません。

`singularitydog_hw.stop_batch_experiment`は、前6台／後6台ごとに、1台・2台・3台・6台分の全ゼロSTOPフレームを1回のUART書込みで送る有限試験です。各対象は1回だけで、自動再送や連続制御はありません。励磁・駆動・報告設定・周期設定の命令は生成しません。STOPを使うので読取り専用ではありません。

```sh
# 既定は計画表示のみ。ポートを開かない。
PYTHONPATH=runtime python3 -m singularitydog_hw.stop_batch_experiment \
  --stage front --group-size 2
PYTHONPATH=runtime python3 -m unittest discover -s runtime/tests -p 'test_stop_batch*.py' -v
```

実行には`--execute-no-motion --known-reporting-off`に加え、`--front-port`、`--rear-port`、`--expected-uids`、`--expected-boot-id`、Git外の新規`--output`が必要です。前後の安定したby-path、現在起動のID、全12個体の識別情報を使います。報告設定OFF・モーター脱力・支持台上の試験に限ります。

1. 各バスの所有者がポートを開き、100ms静穏、全対象のUID・周期生値・電圧35〜45V・mode0／fault0のSTOP返信、再度100ms静穏を確認します。設定変更はしません。
2. 全対象バスの事前確認が揃ってから、グループごとの正規STOPフレームをまとめて書込みます。
3. グループ内の全IDのType2返信を確認してから次へ進みます。型・ID・mode／fault・期限・重複・部分フレームを検査します。
4. 最後に100ms静穏とフレーム境界を確認し、ポートを閉じてから排他を解放します。生ログ整形と保存は終了後です。

途中の部分書込み・送信例外は送信先の境界が不明になるため、その実行の追加送信を禁止します。closeを確認できない場合はロックを保持します。失敗した実行を成功した再試行で上書きしません。

まとめ書込みの開始・終了は**グループ全体のホスト時刻**です。個々のフレームのCAN送出時刻には割り振りません。6フレーム102バイトをUARTへ渡せても、USB基板・CAN・QDD・返信の全工程が完了した証明にはならないため、最後の返信までを別に集計します。

本試験にはIMU・推論・学習済み目標の送信を含みません。STOPでの性能を、そのままMIT制御や連続50Hzの達成とみなしません。
