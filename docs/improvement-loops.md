# 改善ループを動画で比較する

[比較画面](progress.html) は `evidence/improvement-loops.json` から生成しています。HTML をローカルで開くと、ループの選択、状態の絞り込み、変更と結果の左右比較ができます。GitHub のファイル画面は HTML のソース表示になるため、取得したリポジトリの `docs/progress.html` をブラウザで開いてください。

## 番号と結果

今回の記録系列では L00 が動画評価済みの基準です。L01 以降は診断を含む改善ループで、過去全体の通算回数ではありません。各ループの ID は再利用しません。比較対象、仮説、変更、開始・終了状態、実測値、動画の出所を同じレコードに結び付けます。

新しい実験では前進だけでなく前後左右を同じプロトコルで残します。各動画にはループ ID、指令、実測値、記録時間を表示します。比較画面には現在の状態を出し、未測定をゼロで埋めたり、動画のないループに基準動画を流用したりしません。

今回公開できる基準の実測は前進 10.37 cm/s。L04 は 96 更新の診断完了であって、速度改善の測定結果ではありません。L05 は準備中です。この区別が画面にも表示されます。

## 更新

1. 新しい ID と実験の比較条件を JSON に加える。開始前は `PLANNED`、実行中は `RUNNING`、実測評価後は `EVALUATED`。診断の正常終了には `DIAGNOSTIC_COMPLETE` を使う。
2. 測定していない `forward_speed_m_s` と `lift15_counts` は null。予定・実行中のレコードへ最終測定値や動画を登録しない。
3. 公開可能な測定値だけで公開 HTML を再生成する。新しい出力パスを指定し、内容を確認してから既存ページを更新する。

```bash
python tools/make_dashboard.py --index evidence/improvement-loops.json --output progress-next.html
```

ローカル専用の動画一覧は公開リポジトリの外に置きます。`video` は `{"path":"recordings/loop.mp4","sha256":"実際の64桁SHA"}` の形式で、一覧 JSON からの相対パスを使います。

```bash
python tools/make_dashboard.py --index local-loop-index.json --output local-view/index.html --include-videos
```

この場合だけ、SHA を照合した実ファイルを `media/` にコピーします。再生速度やフレームは変更しません。第三者の形状を含む動画はローカル表示に留め、公開版へ追加しません。将来、公開可能な自作モデルだけの動画を用意した場合は出自を確認して別途公開方針を決めます。
