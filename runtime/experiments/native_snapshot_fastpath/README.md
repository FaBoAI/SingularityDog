# 固定17バイト入力スナップショットのC++比較候補

`snapshot_from_records()` の固定長 native `Record` 入力だけを対象にした、
明示選択のファイル専用実験です。`snapshot.cpp` は Type17/STOP の17バイト
フレーム、要求との照合、front/rear所属、重複、時刻因果性、全12軸の充足、
位置・速度値の換算を処理します。IMUの因果性・有限値・期限と最終辞書の組立ては
`loader.py` に残し、既存実装と同じ順序で評価します。C++ライブラリは
ファイル記録のポインタだけを受け取り、FD・CAN・IMU・モーターAPIを持ちません。
既存 bench / transport / observer は変更していません。

既存Pythonが受け取る通常の `native.Record` 配列とトレース由来のリストを
扱います。範囲外の Python 整数や非標準レコード型は基準実装へ戻して、
その例外を維持します。C++ ABIは固定サイズと little-endian IEEE float を
ビルド時に検査し、ライブラリのSHAを指定して読み込みます。生成ライブラリは
targetごとにビルドする必要があり、Gitには保存しません。

リポジトリのルートから、既存のC++20コンパイラだけでビルドします。

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m native_snapshot_fastpath.build \
  --output /private/new-snapshot-parser.so
```

保存記録だけを使う検証は次の通りです。ファイルとライブラリのSHAは呼び出し元が
独立に固定します。出力先は新規ファイルにしてください。

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m native_snapshot_fastpath.verify \
  --library /private/new-snapshot-parser.so --library-sha PINNED_LIBRARY_SHA \
  --stop-records /private/r13/records.json --stop-sha PINNED_R13_SHA \
  --type17-records /private/type17/records.json --type17-sha PINNED_TYPE17_SHA \
  --output /private/new-validation.json
```

2026-09-28 Mac検証は `validation-macos-20260928.json` を参照してください。
保存R13 STOP 500周期、保存Type17 20周期、30種類の拒否変異、両形式の最初の
TX/RX各17バイトに対する17,340種類の1バイト変異で、値・辞書・例外型・理由が
Python基準と完全一致しました。計測は保存入力から復元済みの配列を使い、
スナップショット辞書を返すまでを交互順序で500回ずつ測っています。

| Mac wall時間 | Python中央値 | C++候補中央値 | Python p99 | C++候補 p99 |
|---|---:|---:|---:|---:|
| STOP 500保存周期 | 34.2 µs | 10.3 µs | 40.4 µs | 13.7 µs |
| Type17 20保存周期を500回再生 | 71.0 µs | 10.3 µs | 94.5 µs | 14.2 µs |

**採用しません。** 現行のSTOP主体の20 ms制御周期に対する絶対短縮は中央値で
約0.024 msだけです。Jetsonの同条件測定もなく、C++ ABIと別ライブラリを
実行経路へ増やす価値を現時点では示していません。これは保存入力のCPU段階だけの
結果であり、実機周期・出力安全性・deadline改善を示すものではありません。
