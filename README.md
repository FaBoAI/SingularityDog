# SingularityDog

**直近の改善系列は25回です。** H01〜H04とL00〜L20の診断・失敗を含む回数です。第25回・L20の9動作評価を09/11 14:26:56 JSTに終え、登録基準は**2/9件が合格、7件が未達**でした。

本学習は**実行累計500更新・保持系列488更新分**です。R2の163＋R3歩行157＋R4の180で、R2の12更新は未保存。500更新の予算に到達です。

**9動作を3×3で比較できます。** 全体は14秒です。6方向移動と伏せを各12秒、手振りとお座りは周期途中の14秒まで表示します。終了した枠は最後のカラー画像を保持し、各プレビュー画像内の左上に英語の動作名を大きく重ねます。合否は全評価区間の結果です。

<img src="docs/media/l20-nine-motions.gif" width="720" alt="第25回・9動作の14秒比較。手振りとお座りは周期途中まで。終了した枠は最後のカラー画像を保持。各画像内の左上は大きな英語動作名、合否は全評価区間の結果です。">

[9分割MP4・14秒](docs/media/l20-nine-motions.mp4)

| 動作 | 登録基準 | 個別動画 |
|---|---|---|
| 前進 | 未達 | [MP4](docs/media/l20-forward.mp4) |
| 後進 | 合格 | [MP4](docs/media/l20-backward.mp4) |
| 左移動 | 未達 | [MP4](docs/media/l20-left.mp4) |
| 右移動 | 未達 | [MP4](docs/media/l20-right.mp4) |
| 左旋回 | 未達 | [MP4](docs/media/l20-turn-left.mp4) |
| 右旋回 | 未達 | [MP4](docs/media/l20-turn-right.mp4) |
| 伏せ・四足復帰 | 未達 | [MP4](docs/media/l20-prone.mp4) |
| 高い挨拶・四足復帰 | 未達 | [MP4](docs/media/l20-wave.mp4) |
| 浅いお座り・四足復帰 | 合格 | [MP4](docs/media/l20-sit.mp4) |

4つの独立方策を、同じD16機体で元の親＋0更新と比較した有限評価です。単一方策の連続切替や実機の検証は含みません。旧D13の歩行合格とは分けて記録します。[結果・比較条件・復旧の詳細](docs/l20-evaluation.md) · [選択値JSON](evidence/l20-evaluation.json) · [現在地](docs/current-status.md)

脚・取手・背中・上部PLAは黄色、顔・胴体の基部・カーボン・QDD・足先球は黒、IMUは赤、顔は黒地に白い目です。 [最新仕様](docs/next-loop-spec.md)。過去の結果：[第23回・高い挨拶（位置ずれ未達）](docs/l18-evaluation.md) · [第24回・浅いお座り（登録基準達成）](docs/l19-evaluation.md) · [全履歴](docs/improvement-loops.md)

## 映像でたどる開発履歴

READMEでは**最初・よちよち歩き・第5・10・15・20回の動画ダイジェスト**を掲載します。全ての録画・比較動画39本とCAD画像1点は引き続き公開し、[全動画の履歴](docs/video-history.md)と[25回の実行・評価履歴](docs/improvement-loops.md)からたどれます。改善回数25回と動画本数39本は別の集計です。

元のMP4は全フレーム・50fpsのまま保存。GIFは全区間を等速で表示する縮小プレビューです。段階ごとに形状・物理設定・指令・制御方式が異なる履歴で、同条件の性能比較ではありません。

<details>
<summary>CADで検討した外観を見る</summary>

<img src="docs/media/cad-type00.png" width="640" alt="CADで検討したロボット犬の外観。実機写真や歩行結果ではありません。">

[type00：元のCAD画像](docs/media/cad-type00.png)。設計イメージです。最新の学習モデル・完成した実機を示す画像ではありません。

</details>

| 段階 | 実際の録画・全区間プレビュー | 結果と残った課題 |
|---|---|---|
| **最初**<br>旧形状・保存番号2197<br>直近25回より前 | <img src="docs/media/archive-joystick2197-left.gif" width="480" alt="最初の掲載動画・旧形状での左移動。保存番号2197は改善回数ではありません"><br>[元MP4・5.98秒](docs/media/archive-joystick2197-left.mp4) | 外装付きの旧モデルで左移動を試行。同じ方策の別評価では速度・姿勢条件を満たしたが、後の足別診断では15mm以上の完了足上げは全脚0回。すり足が課題。 |
| **よちよち歩き**<br>第4回 H04<br>初期の前進 | <img src="docs/media/h04-forward.gif" width="480" alt="よちよち歩き・第4回H04の初期前進評価、全12秒"><br>[元MP4・12秒](docs/media/h04-forward.mp4) | 足は動くものの、移動はまだゆっくり。世界X速度平均約0.85cm/s、15mm以上の足上げは前左・前右・後左・後右の順に4 / 4 / 4 / 1回。前段階より速度は退行し、足運びと前進の両立が課題。 |
| **第5回 L00**<br>速い対角歩容・4方向 | <img src="docs/media/l00-four-directions.gif" width="480" alt="第5回L00の前後左右を並べた録画"><br>[元MP4・12秒](docs/media/l00-four-directions.mp4) | **前10.37・後5.90・左4.89・右7.98cm/s**。前進は改善したが、後脚の足上げ不足・滑り・偏向が残る。 |
| **第10回 L05**<br>修正モデル・500更新・4方向 | <img src="docs/media/l05-four-directions.gif" width="480" alt="第10回L05の前後左右を並べた12秒の実録画"><br>[元MP4・12秒](docs/media/l05-four-directions.mp4) | **前16.54・後6.23・左8.05・右6.54cm/s**（変位/12秒）。20cm/s未達。後左脚不足、後進の滑り、右の約50度の偏向が残る。 |
| **第15回 L10**<br>左右旋回と向き保持を追加、500更新 | <img src="docs/media/l10-six-directions.gif" width="480" alt="第15回L10の6方向・12秒。左移動と左右旋回の未達を含む"><br>[元MP4・12秒](docs/media/l10-six-directions.mp4)<br>[同じ初期化のL09比較4方向](docs/media/l10-matched-l09-four-directions.mp4) | **前32.16・後10.67・左8.36・右11.65cm/s**。3/6ケースが開発基準達成。旋回は左+106.78°・右−148.29°、平均0.155/0.216rad/s。最大位置ずれ28.3/19.2cmで、その場旋回は未達。 |
| **第20回 L15**<br>その場での伏せ・起立、500更新・最終499 | <img src="docs/media/l15-model499-stationary-prone.gif" width="480" alt="第20回L15の伏せと起立・全24秒。終端の自動リセットを含む"><br>[元MP4・24秒](docs/media/l15-model499-stationary-prone.mp4)<br>[測定値と未達](docs/l15-evaluation.md) | **伏せ2秒を2回達成、全基準は未達**。第2起立の高さ条件1.9375/2秒、最大方位ずれ21.46°・位置ずれ6.93cm・起立傾き7.92°。72秒評価はスキップ。 |

「最初」は以前の一覧の先頭にあった旧形状の動画です。保存番号2197は改善回数ではなく、直近25回へ加算しません。「よちよち歩き」は既存の第4回H04で、新しい実験回数ではありません。第20回のプレビューは24秒全区間で、終端の自動リセットも含みます。間の回や初期の試行、同条件の比較動画は[全動画の履歴](docs/video-history.md)に残しています。

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

## 実行・評価の全履歴

直近25回は、H01〜H04とL00〜L20の有限な試験を数えたものです。同じ学習のcheckpoint違い、追加動画、次回準備は加算しません。L07の15条件も1回の再設計検証にまとめています。

各回の変更・日時・測定値・次の判断は[25回の実行・評価履歴](docs/improvement-loops.md)、録画と比較条件は[全動画の履歴](docs/video-history.md)、集計用データは[履歴JSON](evidence/learning-history.json)を参照してください。

## 読む順番

- [現在地と実測値](docs/current-status.md)
- [検討と判断の記録](docs/decision-log.md)
- [改善回数・日時・可視化の定義](docs/improvement-loops.md)
- [25回の機械可読な履歴](evidence/learning-history.json)
- [次ループの配色・素材・500更新方針](docs/next-loop-spec.md)
- [PLA部品別のCAD換算重量](docs/pla-parts-mass.md)
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
