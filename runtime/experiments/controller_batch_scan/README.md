# 30Hz controllerの投影スキャンのバッチ化 — CPU専用の等価性検証

実機の推論（中央値6.51ms）のうち、状態付きcontrollerの`step`が約4.6msを占める。その大半は、IMUの上向きへ足先を投影する候補16点を、`feasible_ik`（1回約0.2ms）で1点ずつ順に解く処理である。`feasible_ik`は先頭次元について要素ごとの計算しかしないため、16候補を同じ式で作って1回にまとめて解き、受理・拒否を順に判定するループはそのまま残す。

`equivalence.py` は既定でPLANのみを返す。`--execute-cpu` を付けると次を行う。

- 非公開バンドルの `source/swing_core.py` と `model/policy_controller74_30hz.pt` をSHA256で照合する。
- ループ部分だけを書き換えた候補をメモリ上で構築し、定数とbufferが元のcontrollerと一致することを確かめてからscriptする。
- 合成入力（7方向の指令、bootstrap、reset、二分探索に入る大きな外乱を含む）で、毎tickの観測・actor出力・`step`の全返り値・全bufferをビット比較する。例外は最終行の文言で比較する。
- `--replay-report` で指定した実機原票の入力を流し、元のcontrollerが記録された出力を再現すること、候補が元のcontrollerと一致することを確認する。

候補は配布可能なモデルではない（`deployable_release: false`）。実機に使うには、モデル配布物・SHAの固定・契約へ別途結び付け、ユーザーの判断を得る必要がある。
