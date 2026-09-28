# V3 STOP代理のI/OワーカーCPU分離診断

CPU4に主スレッドだけを固定した従来の無駆動診断では、前後CANとIMUを扱う3本のI/OワーカーはCPU4を含む元のCPU集合を維持する。そのため同一コアでの競合は可能だが、競合が約2msの推論伸長の原因という実測はまだない。

`native_pipeline_benchmark.py`の`--exclude-policy-cpu-from-workers`は、`--mode stop-proxy --v3-voltage-proxy --cycles 1..500 --main-thread-cpu 4`を伴う場合だけ有効な比較条件である。全3ワーカーのnative TIDと元のaffinityを確認し、CPU4を除いた共通集合へ固定する。CPU4以外の使用可能CPUが3個以上なければ中断する。ワーカーの役割はexecutor内で固定されないため、3本とも同じ集合を使う。ウォームアップ・ワーカー設定・読戻しは第1周期の時刻基準より前に終える。終了・例外・途中での設定失敗では各ワーカーの元の集合を復元して読戻し、確認できなければ`ABORTED`とする。主スレッドの従来の復元も継続する。timer slackのワーカー初期化と併用できる。

既存の承認済みJetsonコマンドの他の条件を揃えたうえで、このフラグの有無を別試行で比較する。引数の形だけを確認する場合は、`--execute`を付けずに次を実行する（実機のポートは開かない）。

```sh
PYTHONPATH=runtime python3 -m singularitydog_hw.native_pipeline_benchmark \
  --mode stop-proxy --v3-voltage-proxy --cycles 500 \
  --main-thread-cpu 4 --exclude-policy-cpu-from-workers
```

実行結果の`report.json`では`plan.exclude_policy_cpu_from_workers`と`worker_affinity`の`workers_before`、`target_mask`、`workers_during`、`workers_after`、`restored`を照合する。各行の`native_tid`を一致させ、周期数、推論の尾部、全工程、開始間隔を比較する。PyTorch補助スレッド、他プロセス、割込みのCPU4使用はこの設定では制御しない。またCPU4とCPU5はcpufreqポリシーを共有するため、CPU5のワーカーと周波数・熱の影響を切り離したことにはならない。[別プロセスの周波数・温度記録](cpu4-thermal-sidecar-20260928.md)と時刻を合わせても、標本間の短い変動は分からない。実機の新しい速度値や20ms達成、実出力の有効性は未測定である。

ファイルだけの回帰テストは`PYTHONPATH=runtime:runtime/tests python3 -m unittest test_native_pipeline_benchmark test_native_benchmark_cli`で行う。この選択肢は学習済み目標を送る実出力ランタイムには追加していない。
