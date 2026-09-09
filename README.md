# SingularityDog

DIY 四足ロボットの設計・強化学習で得た、プロジェクト独自のノウハウと検証ツールです。検討したこと、実測できたこと、未解決のことを分けて残します。

**2026-09-09 時点：前進の実測平均は約 10.37 cm/s。目標の 20 cm/s は未達です。** 後ろ脚の足上げ、滑り、横移動時の向きの維持に課題があります。関節の回転慣性を反映した 96 更新の診断学習は完了しました。次の 500 更新は準備中で、開始・完了済みとは扱いません。

## 読む順番

- [現在地と実測値](docs/current-status.md)
- [検討と判断の記録](docs/decision-log.md)
- [ツールの対応範囲](docs/toolkit.md)
- [改善ループと動画の比較画面](docs/improvement-loops.md)
- [公開する情報の範囲](docs/publication-scope.md)

## Skills

| Skill | 使う場面 |
|---|---|
| [singularitydog-design-study](skills/singularitydog-design-study/SKILL.md) | 荷重経路、重量配置、脚形状、CAD と物理モデルを見直す |
| [singularitydog-rl-iteration](skills/singularitydog-rl-iteration/SKILL.md) | 学習実験を比較し、失敗を観測して次の変更を決める |
| [singularitydog-gait-evaluation](skills/singularitydog-gait-evaluation/SKILL.md) | 速度・各脚の足上げ・滑り・動画から到達点を判定する |
| [singularitydog-experiment-ops](skills/singularitydog-experiment-ops/SKILL.md) | 遠隔 GPU 実験の継続、証跡の固定、知識の引継ぎを行う |

各フォルダを対応するエージェントの skill 検索場所へ配置できます。リポジトリ内の相対リンクと `tools/` も使用するため、まずはリポジトリ全体を取得し、対象の `SKILL.md` を指定して使う方法が確実です。

```text
skills/singularitydog-gait-evaluation/SKILL.md を使って、
今回の実験ログから各脚の足上げと目標速度の達否を確認してください。
```

## 動作確認

公開ツールは Python 3.10 以降の標準ライブラリで動きます。

```bash
python -m unittest discover -s tests -v
python tools/audit_urdf.py examples/synthetic_robot.urdf --expected-actuated 1
python tools/evaluate_trace.py examples/synthetic_trace.json
python tools/check_publication.py --root . --files RELEASE_FILES.json
python tools/artifact_manifest.py verify --root . --manifest evidence/release-manifest.json
```

`examples/` はツール検証用の合成入力です。実ロボットのモデル・歩行ログ・学習成果ではありません。

この公開パッケージは設計・学習の知識と共通検証層です。実機固有の CAD、学習済み重み、メーカーの仕様・部品データ、第三者実装、接続先や認証情報を含みません。モデルとシミュレータ環境は利用者が適切な権限で別途用意します。
