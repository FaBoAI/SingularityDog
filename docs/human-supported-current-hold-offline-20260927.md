# 全身を人が支える2秒現在位置保持のオフライン生成

**適用範囲:** この生成器は、40 V Offで支持台を外してから姿勢を読取り、人が胴体を全保持したままEnableと2秒STOPまで通す旧手順だけを扱う。現在検討中の「高い支持台上でEnableし、通電中に支持台を抜く」手順には適用できない。後者は支持台除去中の物理経路、合図と時間窓、荷重計測、受け止め条件を別に審査する必要がある。

[`build_load_transfer_2s_human_supported.py`](../tools/build_load_transfer_2s_human_supported.py)は、審査済みの supported-active r2 パッケージからモーター実行コードを**バイト単位でコピー**し、新しい同一起動の全12軸生角を開始位置の基準に結び付ける。生成器自身は通信ポートを開かず、実機へ転送せず、既存パッケージを変更しない。新パッケージはリポジトリ外の非公開ディレクトリだけに作る。

入力の読取り記録は `fixed_stance_readonly_capture.py` の Type0/17 のみ、3掃引、12 UID一致、同一起動、全軸 run_mode 0、速度絶対値0.1 rad/s以下、位置幅0.02 rad以下、電圧35〜43 V、電流絶対値0.05 A以下、`sampling_issues=[]` と `sampling_stability_heuristic_passed=true` を必須にする。速度スパイクで失敗した記録は、位置幅が小さくても使えない。再取得で合格させる。実行時の21サンプルの駆動前安定判定も別に残る。

2026-09-27の r1/r2 は、後から操作者が中央支持台に載せた状態で取得したと確認したため、この人支持パッケージの入力から明示的に除外する。r2 が数値上の安定判定を通っていても、支持台を外した姿勢の証拠にはならない。

物理レビューファイルは次の形で**非公開の場所**に作り、取得ログの実際の SHA-256 を記入する。現物を確認した結果のみ `true` にする。

```json
{
  "schema": "singularitydog.human-supported-physical-review.v1",
  "boot_id": "<capture boot_id>",
  "capture_summary_sha256": "<summary.json SHA-256>",
  "capture_events_sha256": "<events.jsonl SHA-256>",
  "capture_draft_sha256": "<capture-draft.json SHA-256>",
  "stand_removed_under_40v_off": true,
  "support_stand_absent_during_capture": true,
  "torso_fully_human_supported_during_capture": true,
  "two_operators_continuous_full_support": true,
  "four_paws_floor": true,
  "no_slip_sink_or_clamp_contact": true,
  "cutoff_operator_ready": true,
  "no_load_easing_planned": true,
  "full_support_through_stop_planned": true,
  "box_returned_40v_off_no_anomaly": true,
  "reviewed_for_two_second_human_supported_hold": true,
  "operator_note": "<現物確認の記録>"
}
```

能動保持の凍結前に、同じ姿勢・同じ起動で、出力禁止の2秒試験が25周期と全12軸STOPを完走している必要がある。生成時は `--source-package`, `--capture-summary`, `--capture-events`, `--capture-draft`, `--capture-source`, `--physical-review`, `--disabled-package`, `--disabled-manifest-sha256`, `--disabled-summary`, `--disabled-events`, `--output`, `--remote-package-dir` を指定する。`--remote-package-dir` は、別途転送する場合の Jetson 上の固定パスをハッシュ付きラッパーに埋め込むための情報で、生成器は転送しない。

生成パッケージは2秒、既存のゲインとトルク監視値、電圧・故障・新鮮なType2返信・watchdog確認、全12軸STOPを維持する。ラッパーは実行のたびに全身支持、荷重を緩めないこと、40 V Offでの支持台移動、足接地、受け止めと遮断担当、音声告知、試験許可の明示フラグを要求する。現在姿勢の生角を固定目標にする試験であり、荷重移行や自立の合格証拠にはならない。
