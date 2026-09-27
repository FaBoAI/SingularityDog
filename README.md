# SingularityDog

**GPT-6 Astraが、3D CADの設計・強化学習・評価・改善を進めるロボット犬プロジェクト。**

<a href="docs/media/cad-yellow-assembly-20260914.png"><img src="docs/media/cad-yellow-assembly-20260914.png" width="100%" alt="SingularityDogの黄色・黒のCAD全体像。黄色の脚・取手、黒いBodyと顔を備えた2026年9月14日の全体CAD更新版"></a>

*黄色の脚・取手と黒いBody・顔を反映したCAD全体像（2026-09-14の全体更新版）。その後の共通底板・顔固定・左右連結部・足先の改訂は、[最新STL部材一覧](docs/stl-parts.md)で確認できます。*

**[BOM・部品表](docs/bom.md)** · **[色別STL・ダウンロード](docs/stl-parts.md)** · [動画で見る進捗](#動画で見る進捗) · [実機Deploy](#実機deployの状況) · [現在の課題](#現在の課題) · [ハードウェアライセンス](#ハードウェアライセンス)

## GPT-6 Astraと、実物を作りながら開発する

SingularityDogは、**GPT-6 Astraを開発エージェントに据え、人と一緒にロボット犬を作る**FaBoのプロジェクトです。GPT-6 Astraが機構・固定具の設計変更、STLの準備、制御・学習コード、Isaac Labでの実験と結果整理を進めます。人は目標を決め、部品の採寸・調達・印刷・組立を行い、割れや嵌合などの現物情報を返します。

**設計 → シミュレーション・学習 → 動画と数値で評価 → 印刷・現物合わせ → 実機計測 → 改善**を繰り返します。うまくいかなかった試行も記録し、次の設計と学習に反映する開発記録です。

2026-09-26時点。**第37回まで評価済み。実機はRS05全12軸を公式FW 0.5.0.13へ更新し、前後2系統のCANで12軸の現在位置保持と全軸停止を確認しました。** I2Sスピーカーの試験開始音声も確認済みです。実入力・推論・STOP代理送信の単発計測は中央値20.063msまで短縮しましたが、連続20ms制御の達成を示すものではありません。学習済みモデルによる実機起立・歩行は未実施です。前進の最終目標は40cm/sです。

**[立位前のまとめテストと所要時間](docs/prestand-test-checklist-20260927.md)** · [9月25〜26日の実機結果・残件](docs/2026-09-25-26-hardware-status.md) · [支持付き立位までの短縮手順](docs/fast-track-standing-20260927.md) · [9月24日までの成果・課題・次の手順](docs/hardware-summary-20260924.md) · [20ms化の実測と改善方針](docs/control-20ms-strategy-20260924.md)

**[Deployまでの全体計画（9月22日）：課題・事前準備・実機試験の順序と完了条件](docs/deployment-roadmap-20260922.md)**

## 機体とBOM

| 構成 | 現在の設計 |
|---|---|
| 駆動 | 4脚×3軸、RobStride RS05×12。実機も全個体RS05と操作者が確認 |
| フレーム・外装 | カーボンポールとPLA印刷部品。脚・取手・上部は黄色、Body・顔枠・足先は黒 |
| 計算・姿勢計測 | Jetson Orin Nano、RobStride USB2CAN（CH340）×2（前脚／後脚）、GY-ICM20948V2（I2C） |
| 電源 | Makita 40V 2.5Ahとモバイルバッテリー。MakitaとJetsonは共通底板へ固定 |
| 顔 | φ96mmディスプレイ＋ESP32-S3。前面リングを4本のネジで固定 |

**[BOM全件：数量・寸法・重量・未確定項目](docs/bom.md)**<br>
**[STL一覧：黄色44個・黒45個の候補構成](docs/stl-parts.md)**

STLは個別に取得できます。89個は1台の候補構成数で、これから印刷する残数ではありません。現在の選定は、PLA補強、左右連結部の補強、K1 Max用の共通底板、顔の前面固定、外側でナットをセットできる足先r3を含みます。実際に装着した版と、荷重・嵌合の確認結果はこれから記録します。

## 動画で見る進捗

**第1・2・3・10・20・30回**をたどります。プレビューをクリックすると動画ファイルを開けます。第1・2回は公開録画がなく、第3回は計測チャート、それ以降の掲載映像はシミュレーション録画です。

| 第1回 — まず1本ずつ足を上げる | 第2回 — 周期運動へ進む |
|---|---|
| 92秒・31区間の荷重移動を実行し、4脚それぞれ約20mmの足上げを保持。<br>**公開動画なし** · [記録](evidence/learning-history.json) | 周期参照を3条件で比較。全脚が15mm以上の足上げを各3回。<br>**公開動画なし** · [記録](evidence/learning-history.json) |

| 第3回 — 参照運動に学習を加える | 第10回 — 慣性を修正して再学習 |
|---|---|
| <a href="docs/media/forward-history.mp4"><img src="docs/media/forward-history.gif" width="420" alt="第3回を含む初期の前進計測比較チャート。実録画ではありません"></a><br>300更新で部分的な移動を確認。左後脚の足上げと右移動の接触が課題。<br>[第3回を含む計測チャート動画](docs/media/forward-history.mp4) · [記録](evidence/learning-history.json)<br>※実録画ではなく、初期の複数条件の測定値を再描画。 | <a href="docs/media/l05-four-directions.mp4"><img src="docs/media/l05-four-directions.gif" width="420" alt="第10回の4方向歩行シミュレーション"></a><br>前進16.54cm/s。当時の20cm/s目標と足上げの基準は未達。<br>[MP4を見る](docs/media/l05-four-directions.mp4) · [評価](docs/video-history.md#l05第10回の結果動画) |

| 第20回 — 伏せと起立に挑戦 | 第30回 — 9動作を比較 |
|---|---|
| <a href="docs/media/l15-model499-stationary-prone.mp4"><img src="docs/media/l15-model499-stationary-prone.gif" width="420" alt="第20回の伏せと起立シミュレーション"></a><br>伏せを2秒ずつ2回保持。起立保持・位置・方位に課題。<br>[MP4を見る](docs/media/l15-model499-stationary-prone.mp4) · [評価](docs/l15-evaluation.md) | <a href="docs/media/l25-a-nine-motions.mp4"><img src="docs/media/l25-a-nine-motions.gif" width="420" alt="第30回条件Aの9動作を3×3で比較したシミュレーション録画"></a><br>A/B/C各7/9合格。移動6方向とお座りを達成し、伏せ・挨拶は未達。<br>MP4：[A](docs/media/l25-a-nine-motions.mp4) / [B](docs/media/l25-b-nine-motions.mp4) / [C](docs/media/l25-c-nine-motions.mp4) · [評価](docs/l25-evaluation.md) |

第30回は条件Aの14秒プレビューです。動作名は各画面の左上、先に終了した枠は最後のカラー画像を保持します。合否は全評価区間で判定し、静止表示の時間は成功時間に数えません。9動作は専門方策ごとの個別評価です。

第31〜37回は、伏せ・挨拶の位置や方位の保持を中心に改善を試しました。最新の第37回も位置ずれ50mm以下の基準には届かず、新しい合格モデルはありません。[第37回の結果](docs/l32-evaluation.md)

[全47本の録画・比較動画](docs/video-history.md) · [全1〜37回の履歴](docs/improvement-loops.md) · [節目ごとの結果一覧](docs/milestone-history.md)

## 実機Deployの状況

**現在地：12軸の現在位置保持と停止を確認。10°移動は最初の周期で中断。** 学習済みモデルによる実機起立・歩行はまだ実施していません。

| 段階 | 状況 | 次に確認すること |
|---|---|---|
| 印刷 | 完了の報告あり | 装着した部品の版、足先の嵌合、PLA補強部の強度、組立後の重量 |
| 組立・配線 | RS05×12とI2C IMUを確認。USB2CANを前脚／後脚に分割。12軸をFW 0.5.0.13へ更新 | 自動報告を6軸同時に安定して開始できるか再検証 |
| 校正・停止 | 4脚の手動校正・L字参照候補を保存。ID11に続きID6・9・10を交換・照合 | 原点・正方向・可動域・再起動後の生角度の継続性。物理的な通信断停止時間 |
| 駆動・保持 | 4脚の脚別動作と、前脚上脚の顔側約10°・前脚付け根約5°の限定動作を確認。12軸の現在位置保持を前後各100周期完走し、全12軸STOPを確認 | 前脚付け根の10°軌道を再検証し、目標追従と接触の有無を確認 |
| Jetson上の推論 | 実CAN・IMU入力から単一モデルを計算し、12台へのSTOP代理送信まで単発計測 | 全周期20ms以内の連続制御と学習済み目標角の送信を検証 |
| 立位・歩行 | 未検証 | 支持補助付き立位、荷重移行、低速前進を段階的に検証 |

9月25日の単発3回の実測は、最古入力の取得開始から最終ホスト送信まで**中央値20.063ms・最大20.136ms**、最後の返信まで中央値22.791msでした。送信はSTOP命令による代理測定で、推論した目標角は記録のみです。9月26日の前脚付け根ID3・6の10°試験は、最初の周期で返信の鮮度判定により中断し、完了周期は0でした。全12軸STOPを確認し、操作者に動き・異常・接触は見えませんでした。最初の周期の判定を修正してローカル試験は通しましたが、**実機再試験はまだです**。[9月25〜26日の記録](docs/2026-09-25-26-hardware-status.md) · [現起動の証跡](docs/stance-current-verification-1743-20260926.md)

以後は **実機計測 → 原因の切り分け → 制御・構造の修正 → 必要なGPU再学習 → 実機再確認** を優先します。[実機移行計画](docs/real-world-deployment.md)

## 現在の課題

| 優先 | 課題 | 次の取り組み |
|---|---|---|
| 1 | 前脚付け根の10°試験が最初の周期で返信鮮度判定により中断 | 最初の周期のみ125ms、通常周期100msの修正版を同一起動の証跡とともに実機再検証。停止確認は維持 |
| 2 | 単発のSTOP代理送信でも中央値20.063msで、連続20ms制御に未達 | 取得・推論・送信を連続して測り、最大値と欠測を確認。学習済み目標角は未送信 |
| 3 | 校正・目標追従・停止後の姿勢変化が未確定 | 原点・正方向・再起動後の生角度を照合し、支持台上で有限軌道を完走させる |
| 4 | 前脚の一部軌道で黄色い部材がカーボンポール固定具に接触 | 付け根を水平側へ寄せる経路を実物とCADで確認し、接触する軌道を避ける |
| 5 | 最新の印刷部品と学習時モデルに差がある | 補強部品・電池配置・質量を現物と照合し、シミュレーションへ反映 |

PLAの割れやナットの入れにくさは設計へ反映済みですが、修正版の印刷・装着・荷重試験の結果をもって解決を判断します。[現在地の詳細](docs/current-status.md)

## ハードウェアライセンス

**本プロジェクト独自の現行機構部品と設計資料を、[CERN-OHL-S-2.0](LICENSE-HARDWARE.md)で公開します。** 製作・改変・販売を認め、改良した設計や製品を配布する際も設計を共有する方式です。改良が公開設計へ還元されることを重視して、Strongly Reciprocal版を選びました。[CERN公式](https://cern-ohl.web.cern.ch/)

対象は[適用ファイル一覧](evidence/hardware-license-scope.json)に明記します。市販モーター・Jetson・電池・ディスプレイの内部設計や商標、第三者のCAD、ソフトウェア、学習済み重み、写真・録画へ一括適用するものではありません。

**現在公開しているのはSTL・BOM・選択した設計資料です。編集用CAD／生成ソースと組立・配線資料の公開は未完了です。** [OSHWAの定義](https://oshwa.org/definition/)に沿った、改変可能な完全ソースの公開を今後の課題として明示します。

<details>
<summary>過去の進捗・資料</summary>

- [節目1・2・5・10・15・20・25・30・35・40の一覧](docs/milestone-history.md)（第40回は未実施）
- [最初に掲載した旧保存番号2197の動画](docs/media/archive-joystick2197-left.mp4)
- [よちよち歩き・第4回](docs/media/h04-forward.mp4)
- [第35回](docs/l30-evaluation.md) · [第36回](docs/l31-evaluation.md) · [第37回](docs/l32-evaluation.md)

保存番号2197は第1回を意味しません。実験回数、学習更新数、動画本数、成功数は別に記録します。

</details>

<details>
<summary>検証ツール・Skills・開発者向け情報</summary>

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


</details>

[検討と判断の記録](docs/decision-log.md) · [公開範囲](docs/publication-scope.md) · [PLA換算重量の過去資料](docs/pla-parts-mass.md)

私物写真・認証情報・接続先は公開していません。
