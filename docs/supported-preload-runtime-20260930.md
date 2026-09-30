# 9月30日：箱を残す5秒の伸展・復帰処理

4足の接触を確認した後、脚へ荷重がかかる方向の小さな指令を送り、追従と復帰を調べるための実行処理を追加した。**今回完了したのはソフトウェア実装とMac上の擬似通信試験。Jetsonへの配布・駆動・荷重支持の確認は未実施。** 学習モデルの1%出力を確認したr64とは別の試験である。

## 修正内容

| 問題 | 対策 |
|---|---|
| 0.25mm相当の往復経路を作れても、実行処理へ未接続 | `supported-geometric-preload-5s-v1` と専用CLIを追加。251点の経路を20msの絶対時刻に対応させる |
| 停止・電源再投入・手動調整で始点が変わる | 有効化前とゼロゲイン初期化後にモデル角・生角を再照合。0.05°を超える差、符号不一致、±360°の枝の取り違えを拒否 |
| 目標を元に戻しただけで復帰完了にする | 4秒以降の指令と実測を確認。指令は2量子化単位以内、実測は0.15°以内かつ各軸の追従上限以内であることを確認してからゲインを下げる |
| 異常時に残りの復帰軌道を送ってしまう | 中止・USB故障・入力の古さ・トルク異常・周期飛ばしは、復帰を強制せず既存の両CANのSTOP処理へ移る。STOP未確認なら成功にしない |
| 幾何経路を学習モデルの実出力実績へ混ぜる | `output_kind=geometric_preload`、`preload_targets_sent`、復帰結果を別記録。学習推論を周期中に呼ばず、IMUとモデル入力の妥当性検査は続ける |
| 古い成功ログを変更後のコードへ流用する | 現ソース・経路・起動・電源世代・無出力診断・指令断保護・ソフト試験を結合。診断の開始と終了でソースSHA-256を照合 |
| ソフトの合格を手作業で記入する | 専用テストを実行して合否・テスト名・出力ログ・ソースSHA-256を保存する検証ツールを追加 |

動作は1秒の始点保持、1.25秒の伸展、0.5秒の延長保持、1.25秒の復帰、残りの始点保持とゲイン下降で構成する。上限は0.25mmの幾何目標、関節差1°、名目目標の1°/s・5°/s²、Kp6／Kd0.15。量子化された送信参照値の段差は別記録とし、実測速度・加速度と混同しない。静止PD推定0.2Nmは指令計算上の上限であり、モーターの物理的なトルク上限ではない。

接触とリンクの変形があるため、幾何目標0.25mmがそのまま胴体の実上昇になるとは限らない。旧r65の4足平面残差や方向未照合を別の指標で合格に置き換えず、候補ファイルの未解決事項を保存したまま個別に確認する。

## 実行前に使うソフト検証

リポジトリー直下で次を実行する。すべてファイルと擬似通信の処理で、実機ポートを開かない。出力先は新しい名前を使う。

```sh
python3 tools/validate_supported_preload_software.py \
  --output /tmp/supported-preload-software-validation.json

PYTHONPATH=runtime python3 -m singularitydog_hw.policy_live_profile \
  --write-preload-template /tmp/supported-preload-plan.json

PYTHONPATH=runtime python3 -m singularitydog_hw.policy_output \
  --profile /tmp/supported-preload-plan.json

python3 tools/verify_tested_jetson_sources.py
```

テンプレートは `PLAN_ONLY`／`output_allowed=false`。送信間隔の初期案は成功実績に合わせた0.9ms・同時未返信数3とし、当日の診断で再確認する。空欄へ古い証拠をコピーして実行するものではない。ソフト検証レポートも `approved_for_runtime=false` を維持する。

実行モードは `--execute-supported-preload` と `--absolute-epoch-cadence` を明示して選択する。通常の `--execute-supported`、箱を抜く `--execute-fixed-catch`、地上試験の実行経路との取り違えを拒否する。実機用の完全なコマンドは、当日のキット・電源世代・承認済み設定・音声・ポートが揃ってから生成する。

## 当日の証拠を揃える順序

1. 現在のコードと一致する、幾何経路以外の支持付き設定を用意する。原点・符号・IMUの既存記録は再利用し、現在の起動・電源・姿勢・通信断保護・無出力診断を結合する。この設定を実行すること自体は、経路生成の前提ではない。
2. 同一起動の12軸の静止角とUIDを記録する。この支持付き設定を `preload_source_profile` として固定し、既存の `plan_supported_preload.py` と `build_supported_preload_path.py` で方向と往復経路を計算する。新しいpreload設定自身を入力にすると自己参照になるので使わない。r64設定を現在のloaderへ渡して出るソース不一致は正常な拒否であり、ハッシュ検査を外さない。
3. 無出力診断で `--provenance-mode supported-geometric-preload-5s-v1 --power-epoch CURRENT_EPOCH` を追加し、現在のソースと電源世代を記録する。`CURRENT_EPOCH` は実際の記録に合わせる識別子で、電源状態の自動検出ではない。通常の診断の引数・有限周期数・入力校正も必要。
4. ソフト検証のPASS、同一ソースでの現電源診断、経路SHA、始点、方向、接触、箱と受けを残す条件を別のレビュー記録へ結び付ける。未承認の元経路を書き換えて承認済みにしない。
5. 箱と低い受けを残す5秒枠を1回だけ実施し、横動画と指令／実測／停止のログを照合する。新しい電源、姿勢、経路へ変わった場合は該当部分を取り直す。

## 関連コード

- [実行ランナー](../runtime/singularitydog_hw/policy_output_runtime.py)・[CLI](../runtime/singularitydog_hw/policy_output.py)
- [設定と証拠の検査](../runtime/singularitydog_hw/policy_live_profile.py)・[経路検査](../runtime/singularitydog_hw/supported_preload_path.py)
- [擬似通信・異常停止テスト](../runtime/tests/test_supported_preload_runtime.py)・[CLIテスト](../runtime/tests/test_supported_preload_cli.py)・[設定テスト](../runtime/tests/test_supported_preload_profile.py)
- [検証レポート生成](../tools/validate_supported_preload_software.py)・[診断のソース照合](../runtime/tests/test_native_pipeline_provenance.py)
- [歩行までのToDoとスケジュール](walking-plan-20260930.md)

## 初回実装（コミット2acbd34）の検証結果

- 実行・設定・CLI・経路の専用47件がすべて合格。[当時の実行ログ付きソフト検証レポート](../evidence/supported-preload-software-20260930.json)に当時のソース・テスト・依存ファイルのSHA-256を保存した。
- 関連するランタイム40モジュールの699件中696件が合格、保存モデルのCPU比較用データが未指定の3件はskip。失敗0件。skipした3件をモデル同等性の新しい実証に数えない。
- 経路生成・履歴ソース照合・検証レポート生成のツール34件が合格。
- r64の試験済みJetsonソース178件を再照合。現在の変更で差が出た2ファイルを追加保存し、歴史ソース4件を現行版と分離した。

この検証でハードウェアは開いていない。擬似バスは理想的な角度追従を仮定しているため、実機の20ms周期、足先接触、荷重、実トルクの再検証は当日のH1〜H3で行う。

## 追加修正後の検証結果

[追加監査の修正](additional-fixes-20260930.md)後は、専用54件が合格。[新しいソース付き実行結果](../evidence/supported-preload-software-20260930-r2.json)と[そのログ](../evidence/supported-preload-software-20260930-r2.json.log)を別保存した。Python／C++位置コードの照合と、整数変換後の範囲・変位・静止PD上限も検証した。詳細監査は有効化前の準備で行い、周期内には追加していない。

関連ランタイム43モジュールは789件中786件合格、保存モデル比較データ未指定の3件はskip。関連ツール78件が合格。履歴ソース178件も一致し、旧評価器の追加保存により現行版と異なる履歴コピーは5件となった。実機の合格項目は増やしていない。
