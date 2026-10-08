# 関節目標の固定長演算をまとめる実験

2026-10-08のJetson上のファイルだけの比較で、選択済みの実モデル呼び出しの中央値が **1553.8515µs → 1436.9205µs** になりました。短縮は **116.931µs、約7.53%** です。入力取得・通信・電圧待ち・原票のコピーを含む全周期は測っていません。電圧待ちが残る場合、この短縮が全周期へそのまま反映されるとは限りません。

既存のFKモデルが使う関節目標演算から、12軸／4脚の小さな四則演算と条件集約をC++の固定長ループへまとめました。三角関数、FK／IK、状態更新、射影、演算順序、float32へ変換する境界は保持しています。高速経路はinference modeのCPU・contiguous float64・所定形状だけで、条件に合わない入力は元のATen処理へ戻ります。NaN、無限大、符号付きゼロ、境界値、最初のエラーの意味を変えません。

このディレクトリは明示実行する実験用です。既定設定・実行承認・学習目標の送信・モーター制御入口は変更しません。ビルドと部品検証はCANを開かず、生成した演算を既存の実モデルへ自動登録しません。実Type1送信や全周期20ms以内の証拠としては扱いません。

| 確認 | 結果 |
| --- | --- |
| 保存済み501周期の関節目標・actor出力・観測 | ビット単位で一致 |
| 各呼び出し後の全named buffers／parameters、入力の不変性 | 一致・入力変更なし |
| 不正入力26件の拒否理由と途中までの状態 | 一致 |
| 合成240周期と途中のreset | ビット単位で一致 |
| 数値部品6件（4096組の乱数、NaN、±0、境界、fallback等） | 合格 |
| CPU4・nice−10・単一数値スレッドで4ブロックのABBA比較 | 各モデル2004回、設定と復元を確認 |

| モデル呼び出しだけの時間 | 基準 | 候補 |
| --- | ---: | ---: |
| 中央値 | 1553.8515µs | 1436.9205µs |
| p99 | 1638.766µs | 1518.346µs |
| 最大 | 1670.383µs | 1552.491µs |

測定環境はLinux aarch64、Torch 2.14.0+cpuです。元の実行原票33ファイルと、その保存アーカイブのSHA、比較レポートのSHA、数値ソースのSHAを [evidence.json](evidence.json) に記録しています。公開するC++はこの比較で使ったソースと同じバイト列です。ビルド用のreceiptは個別環境の保存先を除いたものです。

リポジトリのルートから、TorchとC++コンパイラを使えるPythonで実行します。出力先は新しいディレクトリにしてください。

```sh
python3 -B runtime/experiments/native_policy_overnight/target_envelope_scalar/build_file_only.py \
  --output-dir /private/tmp/dog-target-envelope-component-build
python3 -B runtime/experiments/native_policy_overnight/target_envelope_scalar/test_components.py \
  --build-dir /private/tmp/dog-target-envelope-component-build
```

`build_file_only.py` は固定した数値ソースとreceiptを確認し、`-ffp-contract=off -fno-fast-math` で新しいライブラリを作ります。ライブラリは読み込みません。`test_components.py` が明示的に部品演算を読み込み、同じプロセス内で元のATen処理と比較します。この手順の合格は数値部品の検証であり、実モデル501周期の再検証や通信性能の測定は含みません。新たなTorch・コンパイラ・実モデルへの採用は、その環境で個別に検証します。
