# 人による全身支持姿勢の無駆動2秒事前確認

現在、実機用パッケージは生成していない。2026-09-27 の r1/r2 読取り記録は、いずれも胴体を中央の段ボール箱で支えた状態で取得された。支持台を外して人が全重量を支えた姿勢の記録としては使用できず、生成器は両方の記録の SHA-256 を拒否する。

[`build_load_transfer_2s_human_disabled.py`](../tools/build_load_transfer_2s_human_disabled.py) は、将来の**別の**支持台なし・2人で全重量を支えた全12軸記録を受け取るオフライン生成器である。審査済み `load-transfer-2s-preflight-r8` の10個のモーター実行ソースをバイト単位でコピーし、`LIVE_OUTPUT_ENABLED = False` のまま保つ。変更するのは固定パス、証拠照合、実行時の人の支持条件を確認するラッパーと、記録を指すレビューだけである。生成器は UART/CAN を開かず、転送・実行しない。

入力には、同一起動・同じ12 UID の Type0/17 読取り3掃引、完全なイベント、記録元スクリプト、記録 SHA を参照する非公開の現物レビューが必要である。レビューは `support_stand_absent_during_capture` と `torso_fully_human_supported_during_capture` を含む全物理条件を真にし、観察内容を `operator_note` に記す。読取り記録の `RECORDED_REVIEW_REQUIRED` はそれだけでは駆動許可ではない。記録が安定性判定を通っていても、実行時には別途、無駆動21サンプルの全12軸安定窓を通過する必要がある。

将来、条件を満たす記録が得られた場合だけ、非公開領域の新しいディレクトリへ次の形で生成する。`<...>` はその時点の実ファイルに置き換える。

```sh
python3 SingularityDog/tools/build_load_transfer_2s_human_disabled.py \
  --source-package /private/tmp/fabo-stance-20260927-private/load-transfer-2s-preflight-r8 \
  --capture-summary '<future-capture>/summary.json' \
  --capture-events '<future-capture>/events.jsonl' \
  --capture-draft '<future-capture>/capture-draft.json' \
  --capture-source SingularityDog/runtime/singularitydog_hw/fixed_stance_readonly_capture.py \
  --physical-review '<private-physical-review>.json' \
  --output '<fresh-private-package-directory>' \
  --remote-package-dir /home/jetson/singularitydog-tests/load-transfer-2s-human-disabled-r1
```

生成物は `manifest.json`、`preflight-review.json`、コピーした記録と物理レビューを SHA-256 で固定する。将来の実行ログは新しい `summary.json` と `events.jsonl` に出る。`validate_disabled_run(package, summary, events, manifest_sha256)` はパッケージを再照合し、同一起動、Enable/非ゼロゲインなし、両バス25周期、全12軸21点安定窓、電圧と watchdog、初期・最終の全12軸 STOP 確認を要求する。合格結果も電気的・静止姿勢の事前確認に限られ、荷重保持や自立を示さない。

この生成器は**支持台を外した状態で記録し、無駆動事前確認する手順専用**である。支持台で胴体を支えたまま Enable し、その後に台を引く案には、そのまま流用しない。その案には、支持台ありの姿勢と台を引く条件を別に定義したレビュー・実行手順が必要である。
