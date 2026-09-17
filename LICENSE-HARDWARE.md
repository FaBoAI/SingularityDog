# Hardware licence — CERN-OHL-S-2.0

SPDX-FileCopyrightText: 2026 SingularityDog contributors

SPDX-License-Identifier: CERN-OHL-S-2.0

2026-09-18から、[適用ファイル一覧](evidence/hardware-license-scope.json)に列挙した、本プロジェクト独自の現行機構部品のSTLと設計資料を **CERN Open Hardware Licence Version 2 — Strongly Reciprocal** で公開します。対象ファイルを明示するこの文書を、各ファイルのライセンス通知とします。

ライセンス原文は [LICENSES/CERN-OHL-S-2.0.txt](LICENSES/CERN-OHL-S-2.0.txt) です。CERN公式サイトから案内された原文を変更せず収録しています。原文自体の著作権者はCERNです。以下の説明は原文を置き換えたり、追加条件を課したりするものではありません。

## 対象

- 現行の[STL選定manifest](docs/printing/current/manifest.json)にある90個別STL（装着候補86種類、公差代替2種類、試験片2種類）。
- [BOM](docs/bom.md)・[BOMデータ](evidence/bom.json)、[STL部材一覧](docs/stl-parts.md)、上記の選定manifest。
- 適用範囲はファイル名とSHA-256で固定しています。旧配布パック全体や、今後追加するファイルへの自動的な一括指定ではありません。

市販モーター、Jetson、バッテリー、ディスプレイ等の製品内部設計、メーカーのCAD・仕様資料、商標・ロゴ、リポジトリ内のソフトウェア・学習済み重み・写真・画像・動画はこの通知の対象外です。第三者の権利はその権利者に帰属します。適用対象外のファイルに新しい利用許諾を与えるものではありません。

## 製作・改変・配布

ライセンスは、対象設計を利用・研究・改変し、製品を製作・販売することを認めます。対象設計の改変版を配布する際は、既存の通知を保持し、変更日・変更内容を記録して同じライセンス条件を適用します。製品の配布時などに必要となるComplete Sourceの提供・提供場所の通知については、原文の第3・4節を確認してください。

Source Location: <https://github.com/FaBoAI/SingularityDog>

対象設計と製品は無保証です。保証の否認と責任の制限は、ライセンス原文の第6節によります。CERNや部品メーカーによる推奨・保証・開発参加を示すものではありません。

## 完全ソースの公開状況

現時点の公開物はSTL、BOM、選択した設計資料です。編集用CAD／生成ソース、十分な組立・配線資料は未公開であり、これらを含む **Complete Sourceの公開が完了したとは表明していません**。ライセンスを付けることと、改変に適したソースを揃えることを分けて管理します。OSHWの完全なソース公開に向けて、公開可能な編集データと製作資料の整理を進めます。

初回適用では、STLの形状・バイト列を変更していません。ハードウェアの変更履歴は既存の選定manifestと[設計・評価の記録](docs/decision-log.md)を参照してください。

## このライセンスを選ぶ理由

CERN-OHL-S-2.0は、配布される改良版も共有していく方針に合うため選びました。CERN-OHL-W-2.0（弱い相互性）やCERN-OHL-P-2.0（許諾的）もありますが、このプロジェクトでは設計改善の共有を優先します。非商用限定や改変禁止の条件は加えません。

- [CERN Open Hardware Licence 公式](https://cern-ohl.web.cern.ch/)
- [公式ライセンス一覧とガイド](https://ohwr.org/licences/)
- [OSHWA：Open Source Hardwareの定義](https://oshwa.org/definition/)
