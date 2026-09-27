# 読取り専用の立位姿勢記録を確認する

Jetsonで取得した `summary.json` と `capture-draft.json`、取得時と**バイト単位で同じ**非公開の12 UIDファイルを、Git外の私有ディレクトリーに置く。手元の `fixed_stance_readonly_capture.py` と `can_readonly.py` も取得時と同じソースであることが必要。次のコマンドはファイルだけを読み、モーターへ接続しない。

```sh
python3 -B tools/promote_fixed_stance_capture.py \
  --summary /private/path/capture/summary.json \
  --capture-draft /private/path/capture/capture-draft.json \
  --expected-uids /private/path/expected-uids.json \
  --output /private/path/stance-capture-unreviewed.json \
  --review-template /private/path/physical-pose-review-template.json
```

この段階では、全12 UID・起動ID・summaryのSHAとdraftの結び付き、3回の生角度の中央値、角度と速度の安定性、各読取りの時刻を再計算する。出力は `supported_pose_placed_by_operator=false`、`output_allowed=false` で、立位パッケージの入力としては不合格になる。`run_mode=0` や読取り専用の記録は、現時点の全12 STOPを証明しない。

四脚を**同時に一つの姿勢で保持**した現物と写真・動画を別に確認した担当者は、生成された物理レビューテンプレートを別ファイルへコピーする。立位目標として使うには、足先4点が床に直接接地し、胴体だけが支持されていることを確認する。確認できた場合に限り、`pose_class` を `four_foot_floor_supported_stance`、`foot_support_kind` を `floor`、`foot_support_height_cm` を `0` に設定する。`operator_note` に支持方法・姿勢・床接地の確認内容、`evidence_reference` に私有領域の写真・動画の参照を書き、`supported_pose_placed_by_operator` と `simultaneous_physical_stance_verified` を `true` にする。テンプレートの起動ID、12 UID、生角度、SHAは変更しない。確認できなければ分類欄は `null`、両フラグは `false` のままにする。

足先を約12 cmの台に載せたL字は校正姿勢であり、この立位分類には該当しない。床への経路や荷重支持を、L字の読取りだけから推定しない。物理レビューが通っても出力は引き続き禁止される。

```sh
python3 -B tools/promote_fixed_stance_capture.py \
  --summary /private/path/capture/summary.json \
  --capture-draft /private/path/capture/capture-draft.json \
  --expected-uids /private/path/expected-uids.json \
  --physical-review /private/path/physical-pose-review.json \
  --output /private/path/stance-capture-reviewed.json
```

レビュー済みの出力は [立位パッケージ作成ツール](../tools/build_fixed_stance_package.py) が期待する姿勢記録形式になるが、パッケージは無効化専用のまま。現起動の全12軸保持、各区間の全掃引と関節限界、全軸STOP、支持台、荷重移行、自立の証拠はそれぞれ別に必要である。どの出力も自動で駆動を許可しない。
