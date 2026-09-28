# 接地・荷重移行・自立・短距離歩行の記録と判定

2026-09-28。これは**翌日の実機検証用ソフトと手順**であり、実機で立位・歩行を達成した記録ではない。オフラインのテストデータ、既存の支持台上保持、停止命令を代理送信した速度計測から、無支持の立位や学習モデルによる歩行を合格へ昇格させない。

## 4段階を同じ仕組みで実行する

| 段階 | 実施範囲 | 終了後に確認すること |
|---|---|---|
| `supported_stance` | 支持台が胴体を支えたまま、4足接地で有限時間の学習出力 | 12軸の実送信・返信、足の滑り、接触、終了時の停止 |
| `partial_load` | 独立した落下受けを確保し、合図の区間だけ部分的に荷重を移す | 荷重移行の実施と全支持への復帰を動画で確認。トルクから荷重割合を計算しない |
| `stand` | 同じ落下受けの下で短時間の自立を観察 | 支持が実際に外れている区間と姿勢維持を動画で確認。学習出力重み1の実周期が必要 |
| `walk` | 前進のみの低速・短距離試験。加速・減速後、静止区間を置いて全支持に戻す | 実際の移動距離・滑り・接地状態。指令速度×時間を実移動距離としない |

`ground_trial_plan.py` は各段階の有限な時間、数値範囲、前段階の記録、受け止め方法を照合する。`ground_trial_trajectory.py` は時間に応じた前進指令と操作の合図を作る。`ground_trial_output.py` は実機処理へ接続する別の入口で、指定がなければ **PLANのみ**。以前の無出力ツールへ駆動コマンドを追加していない。

初回以降は前の全段階の `PASS_REVIEWED_STAGE` 評価ファイルをSHA256で指定する。合格は**その記録された1試験**についての結果であり、再試行を自動開始する許可ではない。機体構成、12台のUID、基本プロファイルが変わった場合は同一証拠として扱わない。

## 支持と停止

- `supported_stance` は支持台を残す。支持台上の成功から無支持立位の成功を推定しない。
- `partial_load` 以降には、全重量を受けられる固定式の独立した受け、または胴体を受け止める人と遮断・支持操作を分担する2人が必要。
- 終了前に `resupport_window_open` が出たら全支持を戻し、見える端末で新たにEnterを入力する。以前のEnterや経過時間を全支持復帰の確認として再利用しない。
- ソフトウェアからのSTOPはUSB切断後に届く保証がない。実測済みのアクチュエータ通信監視と物理的な受け・遮断を併用する。停止応答がないときに「停止済み」と記録しない。
- 合図は「実施してほしい操作」であり、接地、荷重、立位の成立をセンサーで確認した通知ではない。

## ログと元動画を1組にする

実機ログは `singularitydog.ground-trial-report.v1`。元のネイティブ送受信記録、12軸の実測・指令、IMU、開始・終了時刻、操作の合図、実際のEnterの時刻を保存する。編集していない動画と合わせ、非公開の作業ディレクトリに置く。公開Gitへ生UID・動画・計測生ログを追加しない。

参照パスとSHA256は一括作成できる。`prepare_ground_review.py` は既存ログと元動画を読み、未使用の非公開ディレクトリへ `association.json` と未記入レビューを作る。動画同期や接地の観察結果は空欄のまま残す。

```bash
python3 tools/prepare_ground_review.py \
  --report /private/path/report.json \
  --plan /private/path/reviewed-ground-plan.json \
  --profile /private/path/reviewed-policy-profile.json \
  --video /private/path/original-video.mp4 \
  --output /private/path/new-review-folder
```

動画を見て `association.json` の同期時刻と方法を記入した後、その**確定したファイル**へ未承認レビューをひも付ける。

```bash
python3 tools/prepare_ground_review.py \
  --association /private/path/new-review-folder/association.json \
  --review-output /private/path/new-review-folder/physical-review.json
```

この操作も承認を作らない。実物の観察結果を記入した人だけが、レビュー名・時刻・`APPROVE_THIS_RECORDED_STAGE` を記録する。後から同期ファイルを変更した場合は、再び内容を確認してレビューを結び直す。

`singularitydog.ground-video-association.v1` の例：

```json
{
  "schema": "singularitydog.ground-video-association.v1",
  "references": {
    "report": {"path": "report.json", "sha256": "元ファイルのSHA256"},
    "plan": {"path": "ground-plan.json", "sha256": "元ファイルのSHA256"},
    "profile": {"path": "policy-profile.json", "sha256": "元ファイルのSHA256"},
    "video": {"path": "original-video.mp4", "sha256": "元動画のSHA256"}
  },
  "sync": {
    "trial_start_ns": 1234000000000,
    "trial_end_ns": 1240000000000,
    "video_start_s": 2.0,
    "video_end_s": 8.0,
    "uncertainty_ms": 40.0
  },
  "synchronization_method": "動画に映る端末の合図と音声をログへ対応させた方法を記す"
}
```

時刻は例示。実データで置き換える。動画の区間は最初の駆動周期から最終STOP返信までを覆い、対応する2区間の長さが同期誤差の範囲内で一致する必要がある。100msを超える同期誤差では合格にしない。ホスト時刻はセンサー内部の測定時刻ではない。

動画を見た人のレビューは `physical_review_template()` で未記入の雛形を作れる。雛形は承認されていない。最終的なassociationファイルのSHA256へひも付け、実際に確認した人・時刻を記入する。

```python
from singularitydog_hw.ground_trial_review import physical_review_template
# JSONへ保存してレビューする。Noneを自動的にTrue/Falseへ変換しない。
review = physical_review_template()
```

レビューで確認する項目は、実機を見たこと、試験全体と終了が見えること、開始前・終了後の4足接地、段階に応じた接地、滑り、沈み、過負荷を示す異常、意図しない接触、緊急の受け止め、予定した支持操作、現物の停止。立位・歩行では、観察区間で胴体の支持が実際に外れ、独立した落下受けも重量を支えていなかったことを別項目で確認する。歩行中に4足すべてが常時接地することは要求しない。`independent_body_catch_used` は受けが実際に配置されていたこと、`unplanned_catch_required` は崩れて受け止めが必要になったことを区別する。

歩行の `measured_distance` は次の形にする。距離＋測定誤差が計画の `maximum_measured_distance_m` 以下である必要がある。距離が誤差以下なら、移動を確認できたことにはしない。

```json
{
  "distance_m": 0.08,
  "uncertainty_m": 0.01,
  "method": "video_with_measured_reference",
  "reference": "元動画に映る実測10cmの基準と、胴体の同じ点を追跡"
}
```

## オフライン判定

Jetsonを使わず、Mac上のコピーでも実行できる。

```bash
PYTHONPATH=runtime python3 -m singularitydog_hw.ground_trial_review \
  --report /private/path/report.json \
  --plan /private/path/ground-plan.json \
  --profile /private/path/policy-profile.json \
  --association /private/path/association.json \
  --video /private/path/original-video.mp4 \
  --physical-review /private/path/physical-review.json \
  --output /private/path/evaluation.json
```

評価器は基本プロファイルと段階計画の参照ファイルを検査し、元動画をストリームでハッシュ化する。通信や駆動は行わない。出力ファイルは上書きせず、0600で保存する。

| 状態 | 意味 |
|---|---|
| `RECORDED_REVIEW_REQUIRED` | 技術記録は揃っているが、人による現物レビューが未完了、またはシミュレーション・再生データ |
| `FAIL` | 中断、欠けた通信証拠、不正な値、時刻・ハッシュの不一致、上限超過、現物異常など。次段階へ渡さない |
| `PASS_REVIEWED_STAGE` | 元通信記録・動画対応・明示的な現物レビューが揃った、その1段階の記録。`dependency_eligible=true` |

12軸のUIDとMode 0・通信監視の読戻し、実際のType1送信とType2返信、位置・速度・トルク・温度・電圧、最終12軸STOP、固定角度分枝、IMUの取付変換・バイアス・ノルム・傾き、入力鮮度を再計算する。電圧は全軸の初期取得＋実行中の循環取得であり、毎周期12軸を読み直したとは扱わない。

`actual_controller_20ms_pass` は、その有限試験の**実際の駆動送信を含む全周期**で、周期の処理時間と最古入力取得開始→最終ホスト送信の両方が20ms以内だったことを示す。中央値だけで判定しない。CANバス上の物理送信完了や長時間50Hz動作を証明する値ではない。`full_controller_50Hz_verified` は自動的にTrueへ変えない。

## オフラインで確認済みの範囲

`test_ground_trial_review.py` は合成したCAN・IMU・動画対応データで判定規則を検査する。動画・ログの取り違え、改変された指令、欠けたSTOP、UID不一致、非有限・未知の値、IMU時刻再利用と傾き、20ms判定の虚偽、古い全支持確認、低い学習出力重み、途中停止、指令積分を実距離とする誤りを拒否する。**これらのテスト成功は実機の校正・立位・歩行成功を意味しない。**

## 翌日の実行コマンド

配布キットには `inputs/ground-supported_stance-template.json`、`ground-partial_load-template.json`、`ground-stand-template.json`、`ground-walk-template.json` が入る。すべて未承認で、UID・校正値・前段階の実機証拠・受け止め担当を自動補完しない。ソースから同じ未承認雛形を作る場合：

```bash
PYTHONPATH=runtime python3 - <<'PY'
import json
from pathlib import Path
from singularitydog_hw.ground_trial_plan import template_ground_plan
output = Path('/private/path/ground-stand-template.json')
with output.open('x') as stream:
    json.dump(template_ground_plan('stand'), stream, ensure_ascii=False, indent=2)
PY
```

記入済みファイルの静的照合は、次のPLANコマンドで行う。通信ポートを開かず、モーターを有効化しない。`PLAN_ONLY` と `hardware_opened=false` を返す。

```bash
PYTHONPATH=runtime python3 -m singularitydog_hw.ground_trial_output \
  --ground-plan /private/path/reviewed-ground-plan.json \
  --profile /private/path/reviewed-policy-profile.json
```

下は、基本プロファイルと対象段階のレビュー、現在の支持・役割・動画準備が揃った**将来の実機実行用**。未承認雛形では実行できない。実在する前後USBポート、Jetson上でビルドして照合したライブラリ、音声・音声SHA、現在の電源エポック、未使用の出力先へ置き換える。1回の起動で1段階だけを行う。

```bash
PYTHONPATH=runtime python3 -m singularitydog_hw.ground_trial_output \
  --ground-plan /private/path/reviewed-ground-plan.json \
  --profile /private/path/reviewed-policy-profile.json \
  --execute-ground --support-in-place --cutoff-ready \
  --catch-ready --video-ready --roles-ready \
  --front-port /dev/serial/by-id/FRONT_ADAPTER \
  --rear-port /dev/serial/by-id/REAR_ADAPTER \
  --library /private/path/libdog_active_transport.so \
  --audio /private/path/test-start.wav \
  --audio-sha256 ACTUAL_AUDIO_SHA256 \
  --audio-device ACTUAL_ALSA_DEVICE \
  --power-epoch CURRENT_REVIEWED_POWER_EPOCH \
  --output /private/path/new-trial-output
```

`--support-in-place` は初期支持があることを示す。段階に従い支持を変える際にも、独立した受けと遮断担当を保つ。これらのフラグをソフトが現物確認した事実とは扱わない。

端末は操作者から見える位置で使う。`initial_hold` では全支持を保つ。`active_window_open` から対象段階の操作・観察を行い、`active_window_close` で前進指令がゼロになる。ゼロ指令だけで物理的停止を保証したとは扱わない。`resupport_window_open` が**画面に表示されてから**全支持を戻し、Enter（または `resupported`＋Enter）で確認する。早押しは有効な確認にならない。`q`＋Enter、`stop`＋Enter、Ctrl+Cは中断用。全支持への復帰が確認できないときは通常のゲイン低下へ進めず、中断する。

歩行指令は前進最大0.05m/s、横移動・旋回ゼロ。`maximum_measured_distance_m` は終了後の動画判定の範囲であり、リアルタイムのオドメトリ停止装置ではない。経路・独立した受け・遮断を先に準備する。

| 明日の作業 | 作業時間の目安 | 合格の条件 |
|---|---:|---|
| キット照合・Jetson用ビルド・PLAN | 3〜8分 | コピーとABI、参照ファイル、未解決項目を確認 |
| 基本プロファイルの実機証拠を照合 | 既存の未解決事項による | 12軸角度、IMU方向、通信監視、実際の全工程の証拠が必要 |
| 対象段階の現物準備 | 各2〜5分 | 支持・受け・足元・役割・録画・端末を準備 |
| 1段階の有限試験 | 最大10秒＋起動時検査 | 途中中断しないこと自体を現物合格としない |
| 保存・動画対応・現物レビュー | 各1〜3分程度 | 元ログと動画を結び付け、次段階へ渡せるか判定 |

不合格なら、その原因がある区間を修正・再試験する。段階を繰り返すたびに全ての校正作業をやり直す設計ではないが、校正や機体構成、電源分枝の根拠が変わった場合は対応する証拠を取り直す。これらの目安は立位・歩行の達成時間を保証しない。
