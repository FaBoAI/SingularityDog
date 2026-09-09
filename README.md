# SingularityDog

**確認できた直近の改善系列は9回です。** 初期の立位基準は回数に含めず、H01〜H04とL00〜L04を数えています。診断や失敗も含む9回であり、成功が9回という意味ではありません。全開発の通算回数は未確定です。PPOの更新数やフォルダ数とも区別しています。

DIY四足ロボットの設計・強化学習で得た、独自のノウハウと検証ツールを公開しています。2026-09-09時点で、評価済みの前進平均は**10.37cm/s**。目標20cm/s、後脚の十分な足上げ、方向保持は未達です。第10回に当たるL05は準備中です。

## 映像でたどる開発履歴

今回指定された**CAD画像1点と、実際のシミュレータ録画5本**を掲載しています。元のMP4は全フレーム・50fpsのまま保存。下のGIFは全区間を等速で表示する縮小プレビューです。段階ごとに形状・物理設定・指令・制御方式が異なる履歴で、同条件の性能比較ではありません。

<details>
<summary>CADで検討した外観を見る</summary>

<img src="docs/media/cad-type00.png" width="640" alt="CADで検討したロボット犬の外観。実機写真や歩行結果ではありません。">

[type00：元のCAD画像](docs/media/cad-type00.png)。設計イメージです。最新の学習モデル・完成した実機を示す画像ではありません。

</details>

| 段階 | 実際の録画・全区間プレビュー | 結果と残った課題 |
|---|---|---|
| 旧形状・方向応答の試行<br>保存番号2197<br>直近9回より前 | <img src="docs/media/archive-joystick2197-left.gif" width="320" alt="旧モデル・保存番号2197の左移動録画"><br>[元MP4・5.98秒](docs/media/archive-joystick2197-left.mp4) | 外装付きモデルで左移動を試行。当時の別評価では速度・姿勢条件を満たしたが、後の足別診断では15mm以上の完了足上げは全脚0回。すり足が課題。 |
| 形状・接触表現を更新<br>保存番号250<br>直近9回より前 | <img src="docs/media/archive-dev250-left.gif" width="320" alt="別モデル・保存番号250の左移動録画"><br>[元MP4・11.98秒](docs/media/archive-dev250-left.mp4) | 左へ5cm/sの指令。別の4環境評価では平均約0.16cm/s、全脚の完了足上げ0回。姿勢は保てても移動は未達。 |
| B0・初期の立位基準<br>回数外 | <img src="docs/media/baseline-left.gif" width="320" alt="初期立位基準で左移動を指示した録画"><br>[元MP4・12秒](docs/media/baseline-left.mp4) | 左移動を指示してもほぼ立位に留まる。全脚の完了足上げ0回。ここから荷重移動・参照歩容を検討。 |
| 第4回 H04<br>追加学習後の前進 | <img src="docs/media/h04-forward.gif" width="320" alt="第4回H04の前進評価録画"><br>[元MP4・12秒](docs/media/h04-forward.mp4) | 足は動くが、前進速度は前段階から退行。世界X速度平均約0.85cm/s、15mm足上げ4 / 4 / 4 / 1回。 |
| **第5回 L00**<br>速い対角歩容・4方向 | <img src="docs/media/l00-four-directions.gif" width="320" alt="第5回L00の前後左右を並べた録画"><br>[元MP4・12秒](docs/media/l00-four-directions.mp4) | **前10.37・後5.90・左4.89・右7.98cm/s**。前進は改善したが、後脚の足上げ不足・滑り・偏向が残る。 |

旧2本の数値は同じ方策の**別評価**で、この録画から直接測った値ではありません。2197・250は保存番号で、改善ループの回数ではありません。H04・L00は既存9回に含まれ、B0・旧2本・CAD画像を今回の掲載だけで追加計上しません。ファイル名の「(1)」は元ファイルと同じ内容でした。[対応関係・日時・評価条件](docs/video-history.md)も記録しています。

<details>
<summary>実測チャートで4段階の移動・足上げを比較する</summary>

## 立位から足運びへ：実測データで見る4段階

![実測の移動XY・向き・足上げを図として再描画した履歴比較。ロボットの録画ではありません。](docs/media/forward-history.gif)

[全600フレーム・50fps・12秒のMP4](docs/media/forward-history.mp4) · [静止画](docs/media/forward-history.png)

これは**実測した移動XY・向き・足上げを図として再描画した可視化**です。実機やシミュレータ画面の録画ではなく、メーカーの外観・形状を表示していません。初期立位基準→第3回H03→第4回H04→第5回L00を、共通の観測時刻0.02〜12.0秒で並べています。段階ごとに指令速度と制御方式が異なるため、同一条件の公平なA/B比較ではありません。

| 表示段階 | 観測区間の前進変位（世界X） | 前進評価で分かったこと |
|---|---:|---|
| 初期立位基準・回数外 | 0.48cm | 姿勢を維持したが、全脚の完了足上げは0回 |
| 第3回 H03 | 15.62cm | 参照運動＋残差学習で足運びが生まれ、15mm以上の足上げは各脚4回 |
| 第4回 H04 | 10.63cm | 追加学習後も足は動くが、移動速度は退行。前進の足上げは4 / 4 / 4 / 1回 |
| 第5回 L00 | 124.37cm | 元速度記録の平均で前進10.37cm/s。足上げは13 / 14 / 1 / 13回で、後左脚が課題 |

変位は最後の位置（12.0秒）から最初の観測位置（0.02秒）を引いた世界X方向の差です。足上げ回数の順は前左 / 前右 / 後左 / 後右です。10.37cm/sは元の速度記録を平均した値で、図の始点・終点の位置差を12秒で割り直した値ではありません。見た目の移動や学習の正常終了だけで歩行合格とは扱いません。

</details>

## 実行・評価を確認した9回

1回は「仮説・設定・診断→有限GPU実行→結果評価→次の判断」の一組です。同じvariantの複数条件はまとめ、起動前失敗・CPU検討・形状検査・同じ学習中のcheckpoint違いだけは加算しません。

| 回 | ID | 変更・試験 | 結果と次の判断 |
|---:|---|---|---|
| 1 | H01 | 92秒の荷重移動と4脚上げ | 31区間完了→周期化へ。手順で動かす診断で、学習歩行ではない |
| 2 | H02 | 周期化C1参照を3条件で検証 | 全脚で15mm以上を3回→残差学習へ |
| 3 | H03 | 61観測・300更新 | 部分移動。後左脚不足・右の非足接触→報酬と終了条件を見直し |
| 4 | H04 | 追加600更新 | 4方向を完走したが速度は退行→速い対角歩容へ |
| 5 | L00 | 69観測・対角歩容を250更新 | 前進10.37cm/s、後脚と方向保持が未達→20cm/s段階へ |
| 6 | L01 | 前進指令・報酬を変更 | 約60更新で速度定義域の例外→記録付き再現へ |
| 7 | L02 | 受動記録を追加した再現診断 | 同時点の停止を再現し接地前後を取得→数値設定を比較 |
| 8 | L03 | 速度ソルバ反復数を増加 | 別軌道で再び停止→反復数変更だけの案は不採用 |
| 9 | L04 | 相対回転慣性の計上を修正 | 96更新完了→500更新試験へ。速度改善は未評価 |

**次の第10回・L05は準備中です。** 新しい500更新と4方向評価が終わるまで実施済み回数へ加えず、以前の動画を新モデルの成果として使いません。L04は初期の指令段階だけを通過しており、実機同定や故障の根絶も未確認です。

## 読む順番

- [現在地と実測値](docs/current-status.md)
- [検討と判断の記録](docs/decision-log.md)
- [改善回数・日時・可視化の定義](docs/improvement-loops.md)
- [9回の機械可読な履歴](evidence/learning-history.json)
- [ツールの対応範囲](docs/toolkit.md)
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

基本の6ツールとテストは Python 3.10 以降の標準ライブラリで動きます。追加のアニメーション描画には Pillow・imageio-ffmpeg と日本語フォントを使用します（[描画手順](docs/toolkit.md#readmeのアニメーション)）。

```bash
python -m unittest discover -s tests -v
python tools/audit_urdf.py examples/synthetic_robot.urdf --expected-actuated 1
python tools/evaluate_trace.py examples/synthetic_trace.json
python tools/check_publication.py --root . --files RELEASE_FILES.json
python tools/artifact_manifest.py verify --root . --manifest evidence/release-manifest.json
```

`examples/` はツール検証用の合成入力です。実ロボットのモデル・歩行ログ・学習成果ではありません。

この公開パッケージは設計・学習の知識と共通検証層です。編集可能なCAD・モデルファイル、学習済み重み、メーカーの仕様・部品データ、第三者実装、接続先や認証情報を含みません。モデルとシミュレータ環境は利用者が適切な権限で別途用意します。
