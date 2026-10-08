# 7要求をまとめるC++通信の実験

6軸の返信と電圧1件を、一つのC++交換処理で取得する候補です。6軸分の完全な返信が揃うと、その時点の変更不能なコピーをPythonへ渡します。7件全体の受信・所有者の終了・公開コールバックの完了・コピーと最終返信の一致・クローズを確認してから、次の新しい指令へ進みます。

既存入口からは選択されません。通常のactive ABI 1、STOP用prefix ABI 2を維持し、保持中のType1向けには独立したLIVE ABI 1を追加しています。STOP用ライブラリをLIVE用として読み込むことはできません。

900µs間隔、window3、元の20ms期限を使います。LIVE候補は6件の保持指令、モード2・異常なしの完全返信、電圧35〜42V、有限で狭い角度範囲、Kp3・Kd0.15以下などを検証します。返信の欠落や不一致を成功へ置き換えません。保持指令のバイト列や範囲だけで、モデル・現在の電源・機体状態の照合を完了したとは扱いません。

Jetson上のsocketpair/PTY検証は、新規20件と既存STOP用32件が成功しました。これは実CANのType1出力や全工程20ms達成の証拠ではありません。別の実モデル付きSTOP代理候補では、準備とPythonへの受け渡しが増え、初回の実CAN比較が20ms期限で中止しました。ワーカー構成の修正は別の候補として比較します。

ソースとビルド成果物のSHA、対象CPUでの検証範囲は[source-evidence.json](source-evidence.json)に記録しています。通常の実機入口・承認プロフィールは変更しません。

オフライン検証（実機ポートやネットワークを開きません）:

```sh
PYTHONPATH=runtime python3 -B runtime/experiments/native_active_transport/seven_request_prefix/test_live_seven_request_candidate.py
PYTHONPATH=runtime python3 -B runtime/experiments/native_active_transport/seven_request_prefix/test_seven_request_candidate.py
```

`build.py`は明示実行時に同じ実験ディレクトリへ共有ライブラリとビルド記録を作ります。原票の時刻と全返信は保持します。prefixは全7件の完了・電圧確認・使用可能な実出力設定を意味しません。
