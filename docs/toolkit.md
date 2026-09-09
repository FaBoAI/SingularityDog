# ツールと実行範囲

すべてこのパッケージ向けの新規記述で、Python 標準ライブラリだけを使用します。第三者のソースや私的な実験フォルダから実装をコピーしていません。

| ツール | 実行すること | 実行しないこと |
|---|---|---|
| `audit_urdf.py` | 木構造、関節数、有限値、質量、慣性の正定値性・三角不等式 | メッシュ読込、全体重心、干渉、強度、GPU の実設定検査 |
| `evaluate_trace.py` | 最初の区間の速度、初回失敗、各脚の完了足上げ、材料点の滑り | 高速駆動ログの捏造、実機の合格判定、過去の測定値の再計算を装うこと |
| `artifact_manifest.py` | 明示ファイル一覧の SHA-256 固定・照合 | 出自・ライセンス・物理的正しさの保証 |
| `bounded_job.py` | 有限コマンド、共有ロック、detach、終了記録の照合 | SSH、GPU 構築、学習器実装、自動再試行 |
| `make_dashboard.py` | ループ番号、変更、結果、記録動画の比較 HTML | 動画の生成・補間・高速化、未計測結果の推定 |
| `check_publication.py` | 許可リストの形式、代表的な秘密・接続情報の検査 | 内容の出自を人の代わりに判定すること |

## 有限ジョブ

要求 JSON の schema は `singularitydog.job.v1`。`job_id`、`argv`（絶対パスの実行ファイルを先頭にした配列）、`cwd`、`timeout_s`、`artifact_root`、`manifest_file`、`manifest_sha256`、未作成の `completion_file`、`expected_completion` を指定します。実行時の絶対パスは利用者の環境で生成し、公開リポジトリへ保存しません。

```bash
python tools/bounded_job.py run --request local-request.json --jobs local-jobs --lock local-locks/gpu.lock --detach
python tools/bounded_job.py status --job-dir local-jobs/experiment-001
```

`expected_completion` はたとえば `{"status":"COMPLETED","updates":500,"final_iteration":499}`。子の exit 0 に加え、この実記録が一致して初めて成功です。実行要求に何を認めたかは利用者が確認し、ツールの起動を第三者への操作や追加の実験への包括的な許可と解釈しません。

## 公開物と私的な実験入力

公開物は `RELEASE_FILES.json` に列挙しています。公開 manifest 自身は自己参照を避けてハッシュ対象外ですが、他のソース・文書・選択した測定値は対象です。実機固有の学習コードや CAD を公開できないため、このリポジトリ単体では過去の学習をそのまま再実行できません。必要な実装を管理する手順と、共通の検証・比較ツールを提供します。
