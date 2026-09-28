# 推論経路の native / lean 再構築実験

現在の `policy_shadow.load_policy()` と同じ Trial28A / model_149 の actor を使い、
保存済みの C++ projection と診断テンソルを省いた controller を再構築します。
モデルの重み、入力の校正、出力の関節順序、50 Hz を前提とする内部時間積分は変更しません。
推論結果を返すだけで、CAN・I2C・SSH・モーター出力の API は含みません。
既存 runtime の既定 loader も変更しません。

モデル、ライブラリ、保存入力は明示した外部ファイルだけを使用します。
ネット接続・パッケージの自動インストール・別実装への暗黙の切替はありません。
`.pt`・`.pth`・`.so`・`.dylib` はこのディレクトリの Git 対象から除外しています。

## 再利用した実装と適用範囲

- `projection.cpp` と `torch_projection.cpp` は 2026-09-24 の
  `cpp-policy-integration-r2` 保存版をバイト単位で再利用。
- `lean_swing_core.py` は 2026-09-25 の `native-policy-lean-r1` 保存版と同一。
- `lean_swing_deployment.py` は同保存版の import をパッケージ相対 import に変更しただけ。
- 新規部分は固定 SHA の検証、target-local build、現行 eager loader との比較、
  保存/reload 比較、明示的な成果物 loader、CPU replay と拒否テスト。

由来は本プロジェクトで作成した policy / projection の保存ソースです。
`contracts.py` の `PINS`・`REUSED_PINS` が各原本を固定します。
上記ソフトウェアへ新しいライセンスを付与していません。
リポジトリの CERN-OHL-S-2.0 通知は列挙されたハードウェア設計に限られ、
このソフトウェアやモデル重みには自動適用されません。

native 化は IK/FK projection の固定 4 脚 scan/refine 部分に限ります。
`source_scope_check()` は original bundle から保存 native source を復元して SHA を確認し、
lean 版に残る全メソッド・状態更新・検査・中間 dtype 丸めの AST 一致を検査します。
未使用の診断テンソルを作らない `step_target()` を forward から呼びます。
freeze は採用していません。

## target 上でのビルド

既存の CPU PyTorch、PyTorch の C++ header / library、C++20 compiler が必要です。
Mac のライブラリを Jetson へコピーせず、使用する target 上でビルドしてください。
コマンドはリポジトリのルートから、PyTorch をまだ動かしていない新しいプロセスで実行します。

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m native_policy_overnight build \
  --bundle /path/to/pinned-bundle \
  --output /path/to/new-private-artifact-directory \
  --compiler c++
```

bundle は `model_149.pt`、`swing_core.py`、`swing_deployment.py` の 3 ファイルです。
別の重み・controller・options は拒否します。出力ディレクトリは未作成で、親は存在する必要があります。
ビルド中に experiment source が変わった場合も失敗します。

成功時の出力：

- `torch_projection_fileonly.so`：target の PyTorch ABI 用 custom operator。
- `lean_policy_fileonly.pt`：reset 可能な stateful TorchScript。private に保持。
- `validation.json`：現行 eager・native・保存reload の比較。
- `manifest.json`：source / bundle / library / model / validation の SHA と target 環境。
- `build-report.json`：compiler 引数、所要時間、結果、manifest の SHA。

loadable な manifest は 240 tick 比較と 26 異常入力の拒否比較が合格した場合だけ作成します。
参照 eager との許容差は float64 絶対誤差 1e-9、float32 絶対・相対誤差 1e-6。
parameters・非浮動小数状態は完全一致、candidate と保存reload は全て完全一致を要求します。
全 tick で target・全 buffers・全 parameters と入力非変更を確認します。
これは有限の検証であり、全ての幾何境界の同値性を証明するものではありません。

## 読み込み API

```python
import torch
from native_policy_overnight import load_verified

# プロセスの起動時に一度だけ設定する。
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
model, provenance = load_verified(
    "/path/to/private-artifact-directory/manifest.json",
    expected_manifest_sha256="build の成功時に取得した manifest の SHA-256",
    bundle="/path/to/pinned-bundle",
)
with torch.inference_mode():
    q_target = model(gyro, gravity, command, q, dq, h)
```

入力は CPU float32。gyro・gravity・command は `[1,3]`、q・dq・h は `[1,12]`。
返り値は model 座標の `[1,12]` target です。CAN 座標への変換や送信は行いません。
model は loader が一度 reset し、その後は呼び出すたびに状態が進みます。
通常の tick ごとに reset せず、独立試行開始時・warmup 終了時だけ
`model.reset(torch.tensor([0], dtype=torch.long))` を明示的に呼びます。
一つの model を複数スレッドから同時に呼ばないでください。

manifest の SHA は成功した build から取得した値を指定します。
ロード時に読み込んだ manifest 自体から都合よく期待値を作り直す運用にはしません。
source、現行 reference loader、bundle、target OS/architecture/PyTorch/ABI、成果物の SHA が変わると拒否します。
同名 custom operator が他の loader で登録済みの場合も拒否するため、新しいプロセスを使います。
この loader は thread 数・CPU governor・affinity・ハードウェア設定を変更しません。

## 保存入力による replay

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m native_policy_overnight replay \
  --bundle /path/to/pinned-bundle \
  --manifest /path/to/private-artifact-directory/manifest.json \
  --manifest-sha256 BUILD_OUTPUT_SHA256 \
  --input-report /path/to/saved-report.json \
  --output /path/to/new-replay-report.json \
  --samples 60
```

保存形式は旧 integrated report の `inference.inputs` と、現在の
dual-policy-once summary の `observation.observer_tick.inputs`、または同じ 6 vector の JSON に対応します。
過去の数値だけを取り出します。保存時刻を fresh telemetry として扱いません。
10 回 warmup 後、比較順を交互にし、各 sample の計測外で reset します。
forward の wall / thread CPU 生計時、分布、出力・全状態の差を保存します。
取得、入力組立、送信、連続制御の所要時間は含みません。

## ローカル確認

```sh
PYTHONPATH=runtime:runtime/experiments python3 -m unittest discover \
  -s runtime/experiments/native_policy_overnight -p 'test_*.py' -v
```

private fixture 不要の拒否・状態差検出テストは 15 件です。
2026-09-27 の Mac / CPU PyTorch 2.10.0 で target-local build、240 tick、26 異常入力、
保存reload、現在の保存入力を使う replay が成功しました。
数値と成果物の SHA は `validation-macos-20260927.json` を参照してください。
この結果は Jetson 上の時間、20 ms 全工程、学習済み target の実送信を検証したものではありません。
