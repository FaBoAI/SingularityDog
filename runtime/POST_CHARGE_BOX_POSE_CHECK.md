# 充電後の箱上姿勢・読取り照合

`post_charge_box_pose_check` は、40 V 電源の充電後に箱上姿勢の生角度枝を確認するための単回診断です。CAN へ送るのは Type0 の識別と Type17 の `run_mode`、`current`、`voltage`、`position` 読取りだけです。全12軸の UID を同一起動の基準記録と照合してから、各軸の位置を3回読みます。基準との差は Type17 出力軸角度を直接引き算し、360°の折り返し補正をしません。

Jetson上で、前後の `/dev/serial/by-path/` 名を実機の接続に合わせて指定します。`--execute-readonly` を省くと計画表示だけで CAN を開きません。

```sh
PYTHONPATH=/home/jetson/singularitydog-tests/current/runtime python3 -m singularitydog_hw.post_charge_box_pose_check \
  --baseline /home/jetson/singularitydog-logs/RO-box-pose-after-all-camera-L-20260927.json \
  --front-port /dev/serial/by-path/FRONT_BY_PATH \
  --rear-port /dev/serial/by-path/REAR_BY_PATH \
  --output /home/jetson/singularitydog-logs/RO-post-charge-box-pose-check-20260927.json \
  --execute-readonly
```

基準と Jetson の起動 ID が違う場合、CAN 接続前に終了します。UID 不一致、返信欠落、非正規の送信、別バスからの返信でも中断し、自動再試行はしません。既存の二つの共通所有ロックと前後各ポートのロックを保持し、ポート実体を再確認します。結果は Git 外の新規ファイルへ権限 `0600` で保存します。`RECORDED_REVIEW_REQUIRED` は読取り完了を示すだけで、STOP 状態、角度校正、荷重移行、持ち上げ動作の許可ではありません。
