# CPU4周波数・温度の別プロセス記録

`tools/sample_cpu4_thermal.py`は、既存の無駆動500周期STOP代理ベンチマークと並走する**読取り専用**の診断サイドカーである。CPU4の`cpuinfo_cur_freq`を優先し、読めない環境では`scaling_cur_freq`を使う。後者はCPU4/5で共有されるcpufreqポリシーの報告値であり、実クロックの直接測定とは扱わない。起動時に各`thermal_zone*/type`を読み、温度をzone名と種類の両方に結び付ける。既定では周波数10ms、温度100ms間隔で採り、読み取り前後の`time.monotonic_ns()`を保存する。サンプルは測定中メモリに保持し、終了後だけ別のJSONLファイルへ書く。

Jetson上で、**既に確認済みの**500周期コマンドを変えずに実行する直前に、別端末または同じシェルからサイドカーを起動する。出力先の親ディレクトリは先に作成し、Git管理外の私有領域を選ぶ。既存ファイルは上書きしない。

```sh
python3 tools/sample_cpu4_thermal.py \
  --output /private/diagnostics/cpu4-thermal.jsonl --duration-s 120 &
sidecar_pid=$!

# ここで、既に確認済みの native_pipeline_benchmark 500周期コマンドを変更せず実行する。

kill -TERM "$sidecar_pid"
wait "$sidecar_pid"
```

`--duration-s`は停止し忘れた場合の上限で、SIGTERMまたはCtrl-Cでも正常に記録を確定する。終了後のJSONL末尾に`kind=end`、`reason=signal`または`duration_elapsed`、採取件数があることを確認する。周波数が読めない場合は開始前に失敗する。採取中に個別のsysfs読取りが失敗した場合は値を`null`、`error`を説明文として残し、時系列を欠落させない。`--sysfs-root`と`--boot-id-file`はオフラインの模擬ファイル試験用で、通常は省略する。

現Jetsonの`thermal_zone2/temp`は`ENODATA`を返した。温度ゾーンが少なくとも1つ読めれば診断を続け、読めないゾーンは名前を保持して個別の`error`として記録するよう修正した。Jetson上で2秒の読取り専用起動を確認済み。これは本番500周期の同時計測結果ではない。

ベンチマークの`report.json`には`boot_id`と`measurements[*].prepare_end_ns`、`infer_end_ns`がある。サイドカーの先頭行の`boot_id`と一致することを先に確認し、同じ単調時計上の推論区間`[prepare_end_ns, infer_end_ns]`へ周波数サンプルの`[read_start_ns, read_end_ns]`を重ねる。温度の単位はmillidegree Celsiusで、各zoneにも読み取り前後の時刻が付く。周波数の標本間に起きた短い変動や、モデル呼出し中の瞬間的な実クロックは分からない。サイドカー自体のスケジューリング負荷もあるため、遅延の原因を結論づける前に、同じ条件のサイドカーなしの試行と比較する。

サイドカーはCAN、IMU、モーター、ベンチマーク本体を開かず、CPUのaffinity・governor・周波数上下限・電力モードを変更しない。既存のSTOP代理コマンドはモーターへSTOPを送るので、従来の実機条件と承認範囲に従う。このツールのオフライン試験は`python3 -m unittest tools.test_sample_cpu4_thermal`で行う。
