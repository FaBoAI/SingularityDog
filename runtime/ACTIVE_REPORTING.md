# RS05 Active Reportingの有限検証

`singularitydog_hw.active_report_probe` は自動報告を1台・前6台・後6台・12台で確かめるCAN専用ツールです。モーターの励磁・駆動指令は生成しませんが、報告設定・周期設定・停止命令を送るため、読取り専用ではありません。

## 事前検証

リポジトリーのルートで実行します。既定動作は計画の表示だけで、serialポートを開きません。

```sh
PYTHONPATH=runtime python3 -m unittest discover -s runtime/tests -p 'test_active_report*.py' -v
PYTHONPATH=runtime python3 -m singularitydog_hw.active_report_probe --stage one --motor-id 1 --seconds 1
```

実機では、前後の明示的な`/dev/serial/by-path`、全12個体の非公開UIDファイル、現在のboot ID、Git外の新しい保存先が必要です。`--known-reporting-off`は開始時の報告OFFが既知である場合だけ指定します。100msの無受信だけから、設定値を読めたとはみなしません。

`--execute-no-motion`を明示した場合のみ実行します。操作者の支持・脱力・即時電源Offの条件を確認し、最初は`--stage one --seconds 1`、成立後に10秒と60秒へ進みます。対象拡張は別実行で`--stage front`、`rear`、`both`です。前の試験が自動で次の段階を開始することはありません。

## 検証範囲

- 公式サンプルのType24 payload `01 02 03 04 05 06 F_CMD 08`を使用。受信型は初回にIDごとに2または24を発見し、途中の型変更を拒否します。確認後は`--report-kind 2`または`24`へ固定できます。
- 実機で確認した開始時の列を再検証する場合は、`--report-kind 24 --activation-type2-prefix`を明示します。ON送信後250ms以内の最初のType2を各IDにつき1件だけ別記録し、その後はType24のみを周期報告として受け付けます。OFFにも各ID1件のType2を別区間で記録し、残留Type24を排出します。これらのType2をSTOP応答や設定完了の証明にはしません。順序・型・mode・faultが違えば中断します。
- UID照合、電圧35〜45V（この試験の範囲）、mode0／fault0を確認。`0x7026`が1なら周期は書き換えません。変更する場合は全対象の設定と読戻しを完了してから報告を開始し、終了時に元の周期へ戻します。
- `--period-policy observe-current`では周期を一切書き換えず、Type18を終了処理でも禁止します。`0x7026`の生値0も未解明の値として保存し、時間へ換算しません。実際の受信間隔を測るためのモードで、既定の`set-10ms`は引き続き生値0を拒否します。
- 前後ポートは独立したI/O担当が所有。報告開始後に同期barrierで受信を止めません。詳細ログ整形とファイル保存は全担当の終了後です。
- 各IDの更新間隔、先頭／末尾の空白、最大20ms超の更新空白、解析失敗、部分フレームを記録します。ホスト時刻を保持し、同じ値の連続報告を重複除去しません。
- 自動報告中のType2をSTOPへのACKとは扱いません。報告解除→残留排出・100ms静穏→STOP返信の確認を別区間にします。旧FWの停止返信だけで物理停止時刻が確定したとは扱いません。
- 受信エラー・通常の中断では8秒の別枠で解除・復元を試みます。**UART書込みの一部失敗／例外ではフレーム境界が不明になるため、追加送信を禁止**し、復元不明として終了します。未確認の復旧コマンドを自動送信しません。
- `COMPLETE`はCAN自動報告のホスト観測試験と後処理の完走です。IMU・推論・指令送信の全工程20ms、速度換算、校正、実機歩行の合格ではありません。

結果は`summary.json`、各バスの`*-raw.jsonl`、`*-tx.json`、`*-samples.jsonl`へ保存します。個体情報と接続情報を含むため、生ログはGit外で管理します。

## 個体・版数だけの確認と台数比較

`--preflight-only --read-versions`では、UID・周期生値・電圧・停止応答・公式Type4の版数応答を記録し、静穏を確認して終了します。報告ON/OFFとType18の周期書込みは送信直前にも禁止します。成功表示は`PREFLIGHT_COMPLETE`で、自動報告試験の成功とは区別します。生値0は未解明として保存し、設定には使用しません。

```sh
# 以下は計画表示だけ。実機用の明示引数は上記と共通。
PYTHONPATH=runtime python3 -m singularitydog_hw.active_report_probe \
  --stage both --preflight-only --read-versions --period-policy observe-current

# 報告するのはID1・2・3。最初のOFF対象をID3に固定する比較条件。
PYTHONPATH=runtime python3 -m singularitydog_hw.active_report_probe \
  --stage front --motor-ids 1 2 3 --off-first-id 3 --seconds 1 \
  --report-kind 24 --activation-type2-prefix --period-policy observe-current
```

`--motor-ids`は選択バス内の昇順・重複なしの部分集合に限定し、`--stage one`との併用は拒否します。`--off-first-id`は選択済みのIDだけを受け付け、実際の解除順を記録します。残りの対象は引き続き20ms以上の受信区間を挟んで解除します。実際の受信列に部分フレームが残る場合、静穏だけで正常とせず、次の問い合わせ・STOPを禁止します。

測定失敗で解析が停止した場合は、全OFFの完全書込みと100ms静穏の後、最初のON直前から保存した受信列を独立したパーサーで一度だけ再解析します。保存バイト数・時刻・型・対象ID・mode0/fault0・末尾残留ゼロまで検証できた場合だけ、`cleanup_boundary_verified`を記録し、新しいSTOP照会を許可します。元のパーサー、失敗内容、`INCOMPLETE`判定は変更しません。受信欠落・時刻不明・不正フレーム・未完了OFFでは照会を禁止し、照会自体が失敗した場合も確認を失効させます。

## 保存済み入力による組立て比較

`fast_policy_inputs.prepare_cycle`は生フレーム・時刻・ID等を検査して変更不可の入力を作ります。毎回の組立てで古さ・順序・範囲を検査し、既存の推論側の校正・目標範囲検査も維持します。Type2の保存済み比較と有限の実入力・停止代理送信比較で検証しています。新しい入力の準備費用も毎回の計測へ含め、Type24受信や学習済み指令の駆動へは接続していません。

`python3 -m singularitydog_hw.fast_policy_replay --help`で再生用引数を確認できます。旧実装・保存した結果と完全一致することを検査し、「準備」「準備後の組立て」「準備＋組立て」を別々に計測します。準備済みの同じ入力を繰り返す時間を、新しい入力を取得する処理時間や全工程20msの達成とはみなしません。

2026-09-25のID1初回検査ではUID照合に成功した一方、`0x7026`の読取り値が0でした。最初の実行は設定変更・停止命令の送信前に中断しています。0の意味を推測せず、周期を変えない限定観察で実際の応答を調べます。

周期を変更しない2回目では、ON送信後にType2、続いてType24を受信しました。受信型の変化として中断し、OFF→100ms静穏→STOP返信→閉鎖を確認しています。当該実行では1秒の周期測定を完走していません。

FW 0.5.0.9でのr3では、ID1は1秒・10秒・60秒の受信と解除を完走しました。一方、前6台では受信間隔は20ms以内でも解除しきれないIDが残りました。r4はOFF間隔を20ms以上にし、受信を継続しましたが、問題は解消しませんでした。[旧FWの実機結果](../docs/hardware-active-reporting-20260925.md)に履歴を保存しています。

全12台を0.5.0.13へ更新した後、単台・3台の1秒受信と解除は成功しましたが、前6台の開始欠測は残っています。修正版では6台の開始失敗を保持したまま、独立した受信列検査・OFF・STOP確認まで完了しました。**多台数の通常制御への採用と全工程20msは未達成です。** [更新後の結果・残課題](../docs/hardware-post-firmware-remediation-20260925.md)を参照してください。

`singularitydog_hw.active_report_recovery`は失敗した実行の証跡、同一起動、既知の個体、現在のポートを限定して報告OFFを行う復旧専用ツールです。ONや周期設定は送信できず、100ms静穏後だけUIDの再照合とSTOPを許可します。自動再試行はありません。通常試験の成功扱いに置き換えるためのツールではありません。

[今日の検証計画](../docs/hardware-test-plan-20260925.md)
