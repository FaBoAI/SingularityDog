# Type1のPython側遅延を分離する診断

この独立実験は、元の `four_bus_type1.type1_transport.Type1Transport.hold_then_voltage` と認証済みC++ライブラリを、匿名AF_UNIXソケットと別プロセスの疑似返信側で動かす。物理CAN、serial、モーター有効化、正のゲイン、学習済みモデル、実機資格付与は扱わない。R3/R4のソース・原票・制御契約は変更しない。

既定は `PLAN`。指定された元ライブラリ、ビルド記録、3つのC++ソース、Python transportのSHAを照合するだけで、CDLL、ソケット、スレッドを作らない。`--execute-cpu` を明示したときだけ、CPUと匿名ソケットの実験を開始する。`--library` は元 `libdog_four_bus_type1_transport.so`、`--output` は未使用の絶対パスを指定する。

```text
python -B -m experiments.type1_latency_cause_probe.probe
  --execute-cpu --library /absolute/path/libdog_four_bus_type1_transport.so
  --library-sha256 VERIFIED_SHA
  --ordinary-source-sha256 VERIFIED_SHA
  --subset-stop-source-sha256 VERIFIED_SHA
  --extension-source-sha256 VERIFIED_SHA
  --build-record-sha256 VERIFIED_SHA
  --transport-source-sha256 VERIFIED_SHA
  --ports 4 --cycles 100 --reply-delay-us 2000
  --counter-detail minimal --start-delay-s 1
  --output /absolute/fresh/output
```

行は説明のため分けてある。実際の呼出しでは一つの引数配列として渡す。`--ready-fd` は親が用意した匿名pipe/FIFOの書込FDだけを受理する。所有者スレッドの初期化後、pid・各native TID・疑似peer pidをJSON一行で通知し、その後の `--start-delay-s` の間に親が観測を準備できる。端末・serial・device FDはこの通知口にも使えない。

LinuxではmainをCPU4、各ownerをCPU0〜3（1portはCPU0）、peerをCPU0〜3へ一時配置する。timer slackは1000ns、Python switch intervalは100µs。原CPUマスク、slack、switch intervalを各所有スレッドで復元し、読戻しを原票へ残す。開始前にGCを行い、cycle loopだけGCを延期し、元の有効・無効状態へ戻す。Darwinでのテストは機能検証として明示し、Jetsonと同条件の測定とは扱わない。

疑似peerはCPU配置と入出力準備を終えた時点で、PID・ポート数・物理能力なしのREADYを親へ送る。親は最大5秒のstartup待ちでこれを検証し、通知後にだけownerと診断要求を開始する。最終peer原票は別の2通目として回収する。cold spawnの遅れを`--start-delay-s`に依存させず、診断のnative deadline 200msは維持する。

測る境界は次の通り。

| 境界 | 記録する内容 |
| --- | --- |
| main dispatch / owner entry | queue投入前後、各ownerのnative TID、投入前のowner schedstat |
| Ctypes native call | 呼出し直前、呼出しから戻った直後の**第一Python文**で押す時刻、元Stats.end、返却status |
| checked decode | 元の復号処理の前後 |
| Batch | raw bytesと最終引数の時刻を準備する前、constructor後、registry登録後 |
| voltage | hold前の電圧buffer準備、voltage側のpublication/preflight hook前後 |
| prefix Future | `set_result` 前後とmainのtakeout |
| owner Future | `set_result` 前後とmainのtakeout |
| 失敗 | native返却status、元raw journal、owner/mainのfinally、失敗cycle数 |

mainのtakeoutは実runnerが参照する元 `pipeline._take` を使う。ただしhot/current guardはCPUだけのno-opであり、IMU取得、Torch推論、実電源・姿勢監視、実runnerのThreadPoolExecutorを再現したとは主張しない。ownerは安定した専用queue threadを使う。三軸の要求間隔900µs/window3は元のtransport実装を維持する。絶対deadline 200msはこの診断の失敗を有界にするためで、20ms/30Hzの合格ではない。

有効化をせず元 `hold_then_voltage` を呼ぶため、各疑似sessionに、元の純粋な `_checked_output` が検査したKp/Kdゼロの三軸wireを「最後の出力」として明示seedする。`_enabled`/`_zero_gain` は空のまま。`enable`、`zero_gain`、`output` は呼ばない。peerはcanonicalなゼロType1と電圧Type17のみ受理し、Type3や正ゲインを拒否する。Type2のmode2は疑似生成であり、実モーター状態の証明ではない。起動UUIDも明示したvirtual fixtureである。

計測用に3つの元メソッドをASTで独立コピーし、境界markerだけを挿入する。元Ctypes関数のidentity/argtypes/ABI/sealは差し替えず、元ファイルも変更しない。scope終了時は元メソッドを復元し、元ソースSHAとライブラリsealを検算する。変更したメソッドの元・計測版SHAと計測版sourceを `report.json` のinstrumentation ledgerに保存する。

`--counter-detail full` は各境界にwall、thread CPU、thread rusage、NV/INV context switches、faults、schedstatを記録する。counterには前後のsampling windowが含まれるため、純粋な境界時点のCPUとは扱わない。schedstatsが0または未読ならrun-delayは `null` とし、生の値が0でもOS待ちなしと結論しない。`minimal` はwallとidentityだけを残し、cycle中のcounter読取りを省く。両条件を比較し、full計測自体のGIL/CPU/GC影響を切り分ける。minimalにもmarker辞書作成の影響は残る。

**Stats.endはexactなC関数return直前ではない。** `Stats.end→第一Python文` はnative tail、Ctypes処理、GIL再取得、OS待ち等を含み得る。復号後の `Batch.completed_ns` は最終引数評価のstampであり、constructor終了やFuture通知のstampではない。低CPU・低run-delayだけでGIL原因と断定しない。

隣接 `gil_reacquire.cpp` / `build.py` / `test_probe.py` はCPUだけのnative wait、return直前stamp、controlled GIL holder、same-CPU native contentionを用意する。そこで得た因果は人工条件の診断能力の検証であり、過去の実機停止原因の断定ではない。

`trace_scope.py`は既定PLANで、`--execute-cpu-trace`を明示した場合にのみ独立したtracefs instanceを作る。実測済みのARM64 Python実行ファイルのSHAと48命令byte像を照合し、`PyEval_RestoreThread`のPAC後の入口とGIL取得後のtail branchへ、通常の2つのprobeを置く。return addressを変更せず、未知の実行ファイルへoffsetを推測しない。MONOTONICの時刻とscheduler記録、PID/TID、記録欠落、設定復元を保存する。処理区間はGIL取得とOS実行待ちを含み、thread-state swapと関数復帰のtailを含まない。

`summarize.py`はprobe・trace-report・traceの元バイトをSHAで照合して、新しいJSONだけを作る。元のC++終了→最初のPython実行の窓へ同じTIDの唯一のprobe対を結び、区間別の分布と待ち状態を集計する。欠落のあるtrace、復元未確認、重複・未完の相関を合格へ補完しない。`full`のカウンター採取もGIL/OSへ影響するため、最小記録・追跡なしの比較を別に保持する。

[Jetsonでの有限比較と残件](../../../docs/type1-command-gap-investigation-20261014.md)では、4系統の最小記録1,000周期と1系統対照、詳細記録200周期を直接追跡した。実機停止原票と診断用原票、計測時の旧ソースとcold-start修正版を分けている。修正版のJetson検証は44件合格・skipゼロで、モーター通信は行っていない。

`--drop-first-reply` は疑似peerだけの負例で、deadline失敗時にもnative status、raw journal、failed-cycle countersが残ることを検証する。poisoned sessionのmotion retryは行わない。

オフライン検証:

```text
PYTHONPATH=runtime python -B -m unittest
  experiments.type1_latency_cause_probe.test_python_stages
  experiments.type1_latency_cause_probe.test_probe
```

元transportの本物C++を一時buildし、1/4portの正系、無enable、第一文stamp、BatchとFutureの別境界、失敗原票、pin拒否、PLAN無能力、CPU/switch/GC復元、minimal/fullの違いを確認する。これらの成功は実機の荷重支持や歩行の資格には加算しない。
