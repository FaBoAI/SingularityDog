# Type17とType2の12軸一括比較

`native_feedback_compare.py`は、位置・速度を一緒に返すType2の換算候補を、既存のType17位置・速度と照合する。固定範囲はRS05候補の位置±12.57rad、速度±50rad/sで、計測値から都合のよいscaleを求めたり、設定を書き換えたりしない。

QDDを有効化せず、**胴体を独立して支持し、既に脱力した機体**で実施する。Type2の返信を得るSTOPは状態を変えるため、純粋な読取りとは呼ばない。支持を外した自立中の機体では使わない。

各比較は次の順序で、前6軸・後6軸を並列処理する。

| 段階 | 各バスへの送信 | 目的 |
|---|---|---|
| 初回のみ | Type0×6 | 全12軸の新しいUID返信を期待値と照合。違えばSTOP前に終了 |
| 前側の記録 | Type17位置×6、速度×6 | Type2より前の基準値 |
| Type2の取得 | STOP×6 | 位置・速度・トルク・温度を一返信で取得 |
| 後側の記録 | Type17位置×6、速度×6 | Type2より後の基準値 |

既定3回、最大5回。1軸につき1回5要求なので、これは20ms周期そのものを測る試験ではない。取得間隔・3段階全体の時間は別途出力する。異常・欠落・重複・状態不一致・期限超過では再送せず終了し、両バスの成功分と失敗バイトを保存する。

## 出力の読み方

- Type17の前後値から、Type2を受けた時点の比較値を線形補間する。使うのは各要求開始〜返信読取りの**ホスト時刻の中点**であり、センサー内部の標本時刻ではない。
- ホスト時刻区間に由来する線形モデル上の補間幅を残す。未知の加速度・内部バッファ遅延まで保証する誤差上限ではない。
- 位置差は生の差と2πを法とする差を両方残す。たとえば生差360°・周期差0°は`STATIC_MODULO_ONLY_BRANCH_UNRESOLVED`とし、通常の一致へ昇格させない。校正や生角を変更しない。
- 前後の位置差0.5°以内、Type17速度端点0.15rad/s以内、速度端点差0.1rad/s以内、比較区間100ms以内を静止に近い記録の目安にする。外れる場合は`INCONCLUSIVE_MOTION_OR_TIMING`。両端が同じでも途中で動かなかった証明にはならない。
- 静止に近い記録について、位置差0.2°・速度差0.2rad/sを比較の目安として表示する。安全限界ではない。各軸の実際の最大誤差も併記する。
- 静止で速度値が一致しても**速度scaleの動的検証は未完了**。`dynamic_scale_validated`と`approved_for_runtime`は常にfalse。

## 一括試験ツールから呼ぶAPI

シリアルポート、共通ロック、バス別ロック、キャンセルFD、起動ID監視は呼出し元が所有する。このモジュール単独ではデバイスを開かない。

```python
report, evidence = collect_feedback_comparison(
    sessions, expected_uids,
    boot_id=guard.boot_id,
    supported_disabled=True,
    cycles=3,
    check=guard.check,
)
```

`sessions`は`front`と`rear`の`NativeSession`で、`stop_proxy=True`、有効なboot FDと一致する起動IDを要求する。`report["status"] == "ABORTED"`なら、その接続で後続試験へ進めない。生の`evidence`と`report`はGit外の私有ログへ保存する。

保存後の再解析は`analyze_feedback_comparison(evidence, expected_uids)`で同じ表を得られる。UID・フレーム・軸・時刻順・前後の対応を再検査し、機器との通信は一切しない。
