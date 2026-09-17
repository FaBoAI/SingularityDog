# SingularityDog

**3Dプリントした四足ロボットを、Isaac Labで学習し、実機へつなぐ開発記録。**

2026-09-17更新。**第37回まで評価済み・実機の印刷完了。次は実機Deployを優先します。** 組立・配線・関節校正と実機歩行は確認中です。前進の最終目標は40cm/sです。

[現在地](docs/current-status.md) · [進捗一覧](#過去の進捗) · [動画で見る進捗](#動画で見る進捗) · [BOM](docs/bom.md) · [STL部材一覧](docs/stl-parts.md) · [実機Deploy計画](docs/real-world-deployment.md) · [全動画](docs/video-history.md)

## 現在できていること

| 項目 | 状況 |
|---|---|
| シミュレーション | 過去の第30〜34回で、移動6方向＋お座りの7/9動作が各回の基準に合格。実機での成功とは別です |
| 最新の学習 | 第36回の最終r8で挨拶500更新、第37回A/B各500更新と有限診断を完了。位置ずれ50mm以下は未達、新たな合格モデルはなし |
| 機体・印刷 | 印刷完了の報告あり。PLA補強、共通底板、顔の前面固定、ナットを外でセットする足先ホルダーを整備 |
| 次の優先事項 | Jetson／CAN／IMUの接続、停止処理、12関節の校正、支持補助付き立位と低速歩行 |

[第35回](docs/l30-evaluation.md) · [第36回](docs/l31-evaluation.md) · [第37回](docs/l32-evaluation.md)

## 過去の進捗

指定した節目 **1・2・5・10・15・20・25・30・35・40** を一覧にしました。実験回数と、学習更新数・保存番号・成功回数は別です。条件が異なるため、速度をそのまま改善率として比較しません。

| 回 | 取り組みと成果 | 残った課題 | 記録・動画 |
|---:|---|---|---|
| **1** | 荷重移動を92秒・31区間で診断。4脚それぞれ約20mmの足上げを保持 | 固定手順の確認段階 | [H01記録](docs/milestone-history.md)。公開動画なし |
| **2** | 周期参照3条件で、全脚が15mm以上の足上げを各3回 | 固定手順の診断から歩行へ | [H02記録](docs/milestone-history.md)。公開動画なし |
| **5** | 250更新。4方向12秒完走、前進**10.37cm/s** | 後左脚の足上げ・横移動の偏向 | [動画](docs/media/l00-four-directions.mp4) · [記録](docs/milestone-history.md) |
| **10** | 500更新。前進**16.54cm/s**（変位÷12秒） | 当時の20cm/s目標と足上げが未達 | [動画](docs/media/l05-four-directions.mp4) · [記録](docs/milestone-history.md) |
| **15** | 左右旋回を追加。前進**32.16cm/s**、3/6方向合格 | 左移動の速度・旋回時の位置保持 | [動画](docs/media/l10-six-directions.mp4) · [結果](docs/l10-evaluation.md) |
| **20** | 伏せを2秒ずつ2回保持 | 2回目の起立保持1.9375秒、位置・方位が未達 | [動画](docs/media/l15-model499-stationary-prone.mp4) · [結果](docs/l15-evaluation.md) |
| **25** | 9動作を3×3で比較。実行500更新、合格2/9 | 伏せ・挨拶と移動の退行 | [3×3動画](docs/media/l20-nine-motions.mp4) · [結果](docs/l20-evaluation.md) |
| **30** | A/B/C各500更新、各**7/9合格** | 伏せ・挨拶の位置や方位の保持 | [A](docs/media/l25-a-nine-motions.mp4) / [B](docs/media/l25-b-nine-motions.mp4) / [C](docs/media/l25-c-nine-motions.mp4) · [結果](docs/l25-evaluation.md) |
| **35** | 9試行・277主更新。支持動作と位置ずれを診断 | 新規合格0。最新伏せXY100.320mm・挨拶61.583mm | [結果・比較図](docs/l30-evaluation.md)。新動画なし |
| **40** | **未実施** | 確定結果は第37回まで。実施後に追記 | [実機移行の方針](docs/real-world-deployment.md) |

[節目ごとの詳しい進捗](docs/milestone-history.md) · [第1〜37回の履歴データ](evidence/learning-history.json) · [全改善履歴](docs/improvement-loops.md)

初期映像も保持しています：[最初に掲載した動画](docs/media/archive-joystick2197-left.mp4) · [よちよち歩き・第4回](docs/media/h04-forward.mp4)。前者の2197は保存番号で、第1回を意味しません。

## 動画で見る進捗

第**1・2・5・10・20・30回**を順にたどります。プレビューをクリックするとMP4を開けます。掲載映像はシミュレーション録画です。

| 第1回 — 荷重移動 | 第2回 — 周期参照 |
|---|---|
| 4脚それぞれ約20mmの足上げを保持。<br>**公開動画なし** · [記録を見る](docs/milestone-history.md) | 全脚が15mm以上の足上げを各3回。<br>**公開動画なし** · [記録を見る](docs/milestone-history.md) |

| 第5回 — 4方向への歩行 | 第10回 — 慣性を修正して再学習 |
|---|---|
| <a href="docs/media/l00-four-directions.mp4"><img src="docs/media/l00-four-directions.gif" width="420" alt="第5回の4方向歩行シミュレーション"></a><br>前進10.37cm/s。後左脚の足上げなどが課題。<br>[MP4を見る](docs/media/l00-four-directions.mp4) · [記録](docs/milestone-history.md) | <a href="docs/media/l05-four-directions.mp4"><img src="docs/media/l05-four-directions.gif" width="420" alt="第10回の4方向歩行シミュレーション"></a><br>前進16.54cm/s。当時の20cm/s目標は未達。<br>[MP4を見る](docs/media/l05-four-directions.mp4) · [記録](docs/milestone-history.md) |

| 第20回 — 伏せと起立 | 第30回 — 9動作を3×3で比較 |
|---|---|
| <a href="docs/media/l15-model499-stationary-prone.mp4"><img src="docs/media/l15-model499-stationary-prone.gif" width="420" alt="第20回の伏せと起立シミュレーション"></a><br>伏せを2秒ずつ2回保持。起立保持・位置・方位に課題。<br>[MP4を見る](docs/media/l15-model499-stationary-prone.mp4) · [評価](docs/l15-evaluation.md) | <a href="docs/media/l25-a-nine-motions.mp4"><img src="docs/media/l25-a-nine-motions.gif" width="420" alt="第30回条件Aの9動作を3×3で比較した14秒のシミュレーション録画"></a><br>A/B/C各7/9合格。伏せ・挨拶の位置や方位に課題。<br>MP4：[A](docs/media/l25-a-nine-motions.mp4) / [B](docs/media/l25-b-nine-motions.mp4) / [C](docs/media/l25-c-nine-motions.mp4) · [評価](docs/l25-evaluation.md) |

第1・2回は記録のみです。第30回のプレビューは条件Aの14秒抜粋で、動作名を各画面の左上に表示し、先に終了した枠は最後のカラー画像を保持します。合否は全評価区間に基づき、静止表示の時間を動作成功へ数えません。

[全47本の公開録画・比較MP4](docs/video-history.md) · [最新の数値報告：第37回](docs/l32-evaluation.md)

## BOM — 機体を構成する部品

| 区分 | 主な構成 | 詳細 |
|---|---|---|
| 駆動 | QDD 12軸。シミュレーションはRobStride RS05、実装型番・IDは照合待ち | [駆動・フレーム](docs/bom.md) |
| 計算・通信 | Jetson Orin Nano、USB-CAN、IMU | [型番・未確定項目](docs/bom.md) |
| 電源 | Makita 40V 2.5Ah、モバイルバッテリー | [実測寸法・重量](docs/bom.md) |
| 顔 | φ96mm DisplayとESP32-S3、前面リングを4本のネジで固定 | [表示・配線](docs/bom.md) |
| 構造・固定 | カーボンポール、PLA部品、ベルト、ネジ・ナット | [数量と締結内訳](docs/bom.md) |

**[BOM全件を見る](docs/bom.md)** · [機械可読データ](evidence/bom.json)

購入品、印刷品、締結部品を分けています。未確定の型番・本数・重量を購入確定値として扱わず、ネジ総数も確認できた内訳だけを記載します。

## STL — 印刷する部材

**[色別・部品別のSTL一覧と個別ダウンロード](docs/stl-parts.md)** — 候補構成は黄色44個・黒45個、合計89個。

| 色 | 部材 | 改訂内容 |
|---|---|---|
| 黄色 | 脚の可動部・カーボン固定・背中・取手 | PLA補強版を優先し、旧版と重複させない |
| 黒 | Body側固定・B02/B04/B06/B08 | 割れた左右連結部の補強版を収録 |
| 黒 | Jetson＋Makita共通底板・下段ポール固定 | K1 Max用の一枚底板と着脱クランプ |
| 黒 | 顔マウント | 前面リング4ネジ固定、背面基板とフレキの空間を確保 |
| 黒 | 足先本体＋ナットホルダーr3 | ナット2個を外で装填し、奥当てで位置合わせ。小型試験セットあり |

一覧の個数は**1台の候補構成**で、これから印刷する残数ではありません。実際に装着した版は確認中です。形状検査と、印刷後の嵌合・荷重強度の確認を分けて記録しています。STLはmm、100%で読み込みます。

旧配布パックは[一覧内の履歴](docs/stl-parts.md)から参照できます。PLA重量はソリッド体積換算と実スライス量・実測重量を区別します。

## ここからの実機Deploy

1. 構成と配線を記録し、Jetson／CAN／IMUの受信を確認する。
2. 停止処理と12関節の原点・回転方向を確認する。
3. モーター出力なしの推論照合から、支持付き立位・低速歩行へ進む。
4. 実測ログをシミュレーションへ戻し、必要な修正・再学習を行う。

[実機移行計画](docs/real-world-deployment.md) · [今後の仕様](docs/next-loop-spec.md)

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

私物写真・認証情報・接続先・編集可能なCAD・学習済み重みは含めていません。
