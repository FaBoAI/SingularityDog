# 全12軸診断パッケージの現起動再凍結

2026-09-26時点のJetson起動ID候補は `ac5c430a-15f2-486c-97f5-1e301a10a960`。読み取り専用スナップショットでは12台のUID一致と最低電圧37.23Vが報告された。ただし、前回の4aaf起動用パッケージは現起動で使えず、現在JetsonとのSSH到達性が失われているため、**r14の7ファイル検証キャプチャは未取得、無駆動プリフライトと5秒保持パッケージは未凍結・未転送・未実行**である。

前回の5秒保持はID4の誤差が約1.67°となって約4秒で停止した。停止返信は全12台で確認済み。現起動では、ID4の固定・遊び・配線を脱力状態で点検し、異常なしとの操作者確認を得た。まず、従来と同じID4を含む全軸1°の末尾保持判定、ID10だけKp4・他11軸Kp3を維持して再測定する。ID4の条件を変更する場合は別の診断版とし、標準保持完了とは区別する。

再開手順は次のとおり。各工程は**同じ起動ID**で実施し、途中でJetsonが再起動したら最初からやり直す。

1. JetsonとSSHの到達を回復し、現在の`boot_id`、前後CANポート対応、40V電圧、12台のUIDとSTOP状態を再確認する。
2. 凍結済み`postfw-receiver-r1`を、`--stage both --seconds 1 --preflight-only --read-versions --period-policy observe-current --known-reporting-off --execute-no-motion`で実行する。実行には前後ポート、期待UID、現起動ID、未作成のログ出力先を明示する。Type4 STOP/読取りは使うが、Enable・正ゲイン・設定書込は許可しない。`summary.json`と前後各`-tx.json`・`-raw.jsonl`・`-samples.jsonl`の計7ファイルを回収し、`PREFLIGHT_COMPLETE`、起動ID、ポート閉鎖、ロック解放を確認する。
3. 同一起動の7ファイルを使い、既存のファイル専用ビルダーで`disabled`版を凍結する。旧4aafキャプチャを現起動IDで渡すとビルダーは`Capture is incomplete or from a different boot`で拒否することをローカル検証済み。
4. 凍結した`disabled`版をJetsonへ転送し、20周期の無駆動プリフライトを実行する。Enable・正ゲイン0件、全12台の停止返信、前後ポート閉鎖・ロック解放、結果`PREFLIGHT_PASSED_RESET_CONFIRMED`を確認する。
5. その**現起動のプリフライト実測`summary.json`と`events.jsonl`**を凍結した`active`版へ組み込む。旧起動の成功記録を流用しない。active版は5秒・100周期の現在位置保持だけを許可し、任意の10°軌道や学習済みモデル目標を許可しない。実機での保持試験は支持台・脚の離隔・手離し・即時40V Offの再確認後に行う。

既存r13の元ソースでは、disabledラッパー13件、active＋disabledラッパー14件、双方のmanifest整合性をローカル検証済み。現起動版の実測結果はまだない。
