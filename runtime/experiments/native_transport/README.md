# C++診断用送受信

新規のPOSIX C++17エンジン。前後USB2CANをそれぞれ専有し、Pythonからはバス単位の配列を渡す。C++内部では**17バイトずつ送信**し、最小間隔0.6ms・未返信最大3件を維持する。51バイト以上の結合UART書込みは行わない。

`pselect`の時刻指定、送受信、ATフレーム検査、要求との対応付け、毎回の新しいboot-ID読取りをC++で行う。`ctypes.CDLL`で呼び出す間はGILを解放するため、前後CANとI2C読取りを並列に進められる。固定長の生バイト・時刻レコードを保持し、毎返信のPython辞書作成・再帰コピー・JSON出力を避ける。ディスク保存は全worker終了後。

```sh
python3 runtime/experiments/native_transport/build.py
PYTHONPATH=runtime python3 -m unittest discover -s runtime/tests -p 'test_native*.py'
```

Python開発ヘッダーは不要。Jetson用はJetsonでビルドする。Mac成果物を流用しない。`build-record.json`のsource/binary SHAをロード時に検査する。

許可する要求はType0識別、Type17位置/速度、明示的に選択したType4ゼロSTOPだけ。Type1移動、Type3有効化、Type18設定、Type24自動報告の設定はC++境界でも拒否する。既存の駆動用`BusTrialTransport`は変更しない。

期限超過、欠測、余分な返信、Type21/故障/不正mode、非有限値、部分送信、不正フレーム、入力残留で中断する。再送・フラッシュしての続行・同じsession再利用はない。分割受信は保持し、失敗時の末尾も証拠へ残す。プロトコルに通番はないので、正常返信後の重複が次の同一要求へ極端に遅延する場合まで一意に判別できたとは扱わない。

各バスのポートロック、共通CANロック、UID照合、設定・終了時復元はPythonの診断エントリーが所有する。NativeSession単体はポートを開かず、使用中に外部からFDを閉じない契約。終了時はworkerを待ってからFDとロックを解放する。

期限はホスト上の時刻であり、QDD内部のサンプル時刻やCAN線上の送信完了ではない。MacではCPythonのmonotonicと同じuptime clockを使用し、ロード時にも時刻系一致を確認する。LinuxはCLOCK_MONOTONIC。

`sd_wait_until`／Pythonの`native_diagnostic_transport.wait_until(library, cancel_fd, deadline_ns, spin_us=200)`は、別途明示選択する診断用の絶対時刻待機である。デバイスFDを受け取らず、CAN・IMU・モーター操作をしない。未来1秒以内の単調時計期限まで、キャンセルFDを`pselect`で監視しながら待ち、最後の200または500µsだけGILを解放したC++内で回す。既に過ぎた期限は1秒以内なら待たず、実際の現在時刻を返す。前後のキャンセル確認に失敗した場合は例外になり、計画時刻への時刻の書き戻しはしない。睡眠の遅れや他スレッドとの競合が残るため、厳密な20ms間隔を保証する機能ではない。既存の交換処理・通常の待機経路はこの関数を呼ばない。

統合入口は`singularitydog_hw.native_pipeline_benchmark`。Type17比較、実推論後の12軸STOP代理送信、複合返信の照合を分けて実行する。START/STOPの高速経路を、そのまま学習モデルの実駆動に採用したことにはならない。
