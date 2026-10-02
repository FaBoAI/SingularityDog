# Jetson の C7・EMC 比較用スコープ

`jetson_latency_power_scope.py` は、1つの前景コマンドの間だけ、全オンライン CPU の `name=c7` かつ `latency>=1000µs` の idle state を無効にし、`/sys/class/devfreq/bwmgr/min_freq` を開始時の `max_freq` まで上げます。WFI とその他の idle state、EMC 最大値、電源モードは変更しません。

無出力の比較では `--diagnostic` を指定します。支持付き実出力では、子の既存 profile loader が承認した試験に限り `--supported-characterization` を指定します。両指定は排他的で、どちらかが必須です。このツールは性能設定だけを変更し、出力承認・トルク設定・通信監視条件を作成または変更しません。子コマンドの CAN 内容を検査する仕組みではありません。

```sh
sudo -n nice -n -10 python3 tools/jetson_latency_power_scope.py \
  --diagnostic --with-cpu-performance -- \
  env PYTHONPATH="$KIT/runtime" "$VENV/bin/python" -B \
  -m singularitydog_hw.native_pipeline_benchmark [同じ無出力診断引数]
```

`--with-cpu-performance` は既存 `jetson_cpu_performance_scope.CpuPerformanceScope` を同じプロセスで使用します。既存 CPU wrapper を子コマンドとして重ねると別の process session ができるため、重ねずこの flag を使ってください。子は元の sudo 実行ユーザーへ戻して実行され、性能設定を復元する wrapper だけが root 権限を保ちます。直接 root で実行し sudo のユーザー情報がない場合は、子も root になります。

比較時は、校正・モデル・診断引数・送信間隔・CPU 最低周波数を揃えてください。変更は子開始前に readback で確認します。標準出力に C7/EMC の元値・適用値、変更しない idle state、オンライン CPU、CPU policy（flag 使用時）、開始・復元時刻を JSON で出します。baseline は従来の CPU scope、比較は本ツールの `--with-cpu-performance` を使い、同じ記録範囲で比較します。C7/EMC 設定の変更だけで 20ms 達成を保証するものではありません。

終了・子の失敗・Ctrl+C・SIGTERM・SIGHUP・例外時には、子の process group の終了と cleanup を先に待ち、CPU policy → C7/EMC の逆順に元値へ戻します。子 cleanup の猶予は既定3秒で、必要なら `--term-grace-seconds`（最大30秒）を指定します。猶予後も子が終わらなければ SIGKILL を使用します。復元値は再書込みと全設定の quiet readback で確認し、復元失敗を exit 125 と `restored=false` または CPU 復元失敗として明示します。WFI・EMC 最大値・オンライン CPU が外部で変わった場合も成功としません。

対応する C7 または WFI がない CPU、bwmgr がない環境、設定を読み書きできない環境は子開始前に失敗します。SIGKILL、電源喪失、子が自ら別 session に逃げた場合の復元・停止は保証できません。外側の timeout は wrapper に SIGTERM を送り、wrapper の子 cleanup と最大4秒の各スコープ復元が終わる猶予を残してください。

Mac 上の検証は偽 sysfs を使い、実機の sysfs には触れません。

```sh
PYTHONPATH=.:runtime:runtime/tests python3 -m unittest \
  test_jetson_latency_power_scope tools.test_jetson_cpu_performance_scope -q
```
