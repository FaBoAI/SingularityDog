# 前脚付け根10°試験の連続準備と記録

`tools/front_hip_test_session.py`は、**同一起動の証拠を一つのセッションに集める非駆動CLI**。保持→無効化preflight→現物の経路確認→有効化試験の順序を固定する。パッケージの作成とログの取込みを一括化し、転送・モーター駆動・40 V操作・物理確認の承認は行わない。試験を自動で再実行する機能ではない。

同じ入力でコマンドを再実行すると、保存済み段階を検証してその結果を返す。別のhold、起動ID、ログ、レビューを同じセッションへ混ぜた場合は止まる。失敗時は新しい起動や姿勢に切り替えない限り、失敗した段階から再開できる。完成した各段階の`*-receipt.json`と`session.json`にはSHA-256が残る。未完成のパッケージは自動削除せず、原因を調べてから新しいセッションを使う。

`SESSION`はMac上の新しいディレクトリ、`SOURCE_DISABLED`は現起動で凍結した無効化ソース、`HOLD`は同一起動で全12軸の5秒保持とSTOPを終えた`summary.json`を指す。以下はパスを各自の実ファイルに置き換える。

```sh
python3 -B tools/front_hip_test_session.py create \
  --session "$SESSION" --source-disabled "$SOURCE_DISABLED" \
  --hold-summary "$HOLD"
python3 -B tools/front_hip_test_session.py status --session "$SESSION"
```

上の`create`は従来の単発10°。今回の**最初の1秒保持→5°へ移動・保持→10°へ移動・保持を同一Enableで行い最後にSTOP**する試験には、新しいセッションを作り、`create`に`--continuous-5-10`を明示する。単発と連続のパッケージ・レビュー・ログは同じセッションに混在できない。

```sh
python3 -B tools/front_hip_test_session.py create \
  --session "$CONTINUOUS_SESSION" --source-disabled "$SOURCE_DISABLED" \
  --hold-summary "$HOLD" --continuous-5-10
```

`$SESSION/front-hip-step10-disabled`をJetsonへ別途転送する。実行する場合は現起動のJetson上で、支持台・脚/配線の離隔・即時40 V Offを確認し、既存の凍結launcherを**1回だけ**起動する。`LOG`は`/home/jetson/singularitydog-logs`の直下に作る新規ディレクトリ名にする。

```sh
python3 -B /home/jetson/PACKAGE/front-hip-step10-disabled/prepared_fullbody.py \
  --preflight-only --supported \
  --output /home/jetson/singularitydog-logs/NEW-PREFLIGHT-LOG
```

Jetsonから`summary.json`と`events.jsonl`を同じMacのログディレクトリへ回収して、次を実行する。preflightの完走・全12軸STOP・起動ID・wrapper SHA・イベントSHAを既存builderで検証し、実測12軸角を**未承認ドラフト**へ転記する。

```sh
python3 -B tools/front_hip_test_session.py ingest-preflight \
  --session "$SESSION" --log-dir "$PREFLIGHT_LOG"
python3 -B tools/front_hip_test_session.py status --session "$SESSION"
```

操作者がドラフトを別ファイルへコピーし、現在の12軸姿勢を基準として、±3°の始点変動と右前ID3＋10°／左前ID6−10°の全経路に黄色部材・カーボン固定具・配線の干渉がないことを現物で確認する。確認内容と必要な既存フラグを記録した**別の**`physical-review.json`を使う。連続profileのドラフトには`continuous_profile`、`continuous_waypoints_deg: [5.0, 10.0]`、未承認の`continuous_19s_reviewed: false`も入る。同一Enableで約19秒間この経路を保持・移動でき、最後のSTOPで脱力して戻る範囲も接触しないことを別途確認してから、最後の欄を`true`にする。`build-active`はこの三項目が正確にそろわない限り止まる。ドラフト自体は変更しない。隙間を確認できない場合や姿勢が大きく変わった場合は、有効化せず新しいpreflightとレビューを取得する。

```sh
python3 -B tools/front_hip_test_session.py build-active \
  --session "$SESSION" --source-active "$SOURCE_ACTIVE" \
  --physical-review "$PHYSICAL_REVIEW"
python3 -B tools/front_hip_test_session.py status --session "$SESSION"
```

`$SESSION/front-hip-step10-active`は**試験用パッケージ**であり、ビルド直後には実機へ送らない。転送後、実行直前に40 V On、支持台、手離し、4脚・配線の離隔、現位置からの全経路、即時40 V Offを再確認する。試験launcherの明示フラグは `--execute-role-group-step1 --clearance-confirmed --cutoff-ready --supported --hands-off --output /home/jetson/singularitydog-logs/NEW-ACTIVE-LOG`。launcher自身が起動ID、ピン留めしたコード、実測始点と物理確認済み範囲を検証し、終了時に全12軸STOPする。CLIがこれらのフラグを自動付与して実行することはない。

試験後、Jetsonから`summary.json`と`events.jsonl`を回収する。成功・途中停止のどちらも取り込み、試験番号ごとの独立した証拠として残す。例えば`attempt-1`で失敗しても、その名前やファイルを上書きせず`attempt-2`へ進む。**自動再試験はしない**。STOPを確認できなかった場合、CLIは次の動作を勧めず現物の点検を表示する。

```sh
python3 -B tools/front_hip_test_session.py ingest-result \
  --session "$SESSION" --log-dir "$ACTIVE_LOG" --attempt attempt-1
python3 -B tools/front_hip_test_session.py status --session "$SESSION"
```

この流れは前脚付け根2軸の診断用で、全12軸の立位移行や学習済み方策の歩行試験を許可しない。連続profileも同じ機体条件と全12軸STOPを要する。自動再試験はなく、試験後の実測角とSTOPを確認してから次の試験に進む。
