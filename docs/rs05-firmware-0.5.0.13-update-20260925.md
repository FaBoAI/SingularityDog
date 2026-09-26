# RS05 0.5.0.13 更新準備とr2初回試行の記録 — 2026-09-25

**最新状態：全12台を1台ずつ0.5.0.13へ更新し、更新後の版数・UID一致・STOP返信mode0／fault0を確認しました。** 最終結果は[更新成功記録](rs05-firmware-update-success-20260925.md)と[結果JSON](../evidence/rs05-firmware-update-success-20260925.json)にまとめています。Active Reportingの改善、20ms制御、全NVMの保持は未検証です。

以下は更新準備からr2初回試行までの記録です。r2ではID1へSTARTを1回送信し、2秒以内に応答がなく中断しました。BIN本体の転送は0bytesで、40V再投入後に全12台の元の版数0.5.0.9・識別一致・停止を確認しました。[初回試行・復旧の記録](rs05-firmware-update-attempt-20260925.md)

準備段階ではbootloader移行を行わず、r2のSTART送信後に実際に移行したかも確認できませんでした。後続のr4で期限後のSTART返信を記録し、r5でSTART応答期限を3秒へ限定変更した経緯は成功記録に分けています。0.5.0.13には自動報告と返信の偶発競合の改善が記載されていますが、今回のType24解除失敗への効果は未検証です。[公式更新履歴](https://github.com/RobStride/Product_Information/releases/download/V26.04.07/update.pdf)

## 取得物と根拠

| 項目 | 確認値 |
|---|---|
| 対象 | RS05 / 0.5.0.13（指定版。0.5.0.14へ変更しない） |
| リリース | [V26.04.07](https://github.com/RobStride/Product_Information/releases/tag/V26.04.07)、公開 2026-04-09 |
| 正規BIN | [rs05_0.5.0.13.bin](https://github.com/RobStride/Product_Information/releases/download/V26.04.07/rs05_0.5.0.13.bin) |
| 容量 | 93,584 bytes |
| SHA-256 | `3a4dfc1e9ad0ff2116b7b876c1fced3b8d23cf705e5d6366e5a286bac3989047` |
| 検証 | 保存BINを再計算し、保存manifest・GitHub release asset digestと一致 |

BINと取得記録はローカル保管。公開用には[準備状況JSON](../evidence/rs05-firmware-0.5.0.13-preparation-20260925.json)を保存し、個体UID・生ログ・private IPは含めません。

参照ソースを固定します。

- [公式OTA.py](https://github.com/RobStride/Product_Information/blob/6ad12f50006273b7ea4eea88980f927d97c22f0d/OTA/OTA.py) — commit `6ad12f50006273b7ea4eea88980f927d97c22f0d`。
- [公式OTA仕様書・Qt参考コード掲載PDF](https://github.com/RobStride/Product_Information/blob/6ad12f50006273b7ea4eea88980f927d97c22f0d/OTA/OTA%20Agreement%20Description%20-%20EN_20251114102200.pdf) — 同commit。
- [RS05公式マニュアル260713](https://github.com/RobStride/Product_Information/blob/0f4ad74fdb67023e75bbcbeebecd6f1a003ce000/Product%20Literature/RS05/RS05User%20Manual260713.pdf) — §3.3.3更新、§3.3.4パラメータ保存・管理。

## 仕様差と、初回実装で許可した経路

| 項目 | 公式PDF／掲載Qtコード | 公式Python | 扱い |
|---|---|---|---|
| 失敗状態値 | `0x0F`／`0x0F00`との比較 | `0xF0` | 非ゼロをすべて失敗として停止。失敗値の推測や復旧分岐を使わない |
| Type13データへの返信 | 表ではType11 | 送信と同じType13を要求 | PythonのType13だけを受理し、それ以外は中断。r2初回試行ではSTART後へ進まなかった |
| パケット番号 | 表ではbit15..8、Qtは16bit値を使用 | bit23..8の16bit値 | QtとPythonが一致する16bit幅を採用。全11,698パケットを模擬検証 |
| 更新対象の照合 | STARTに対象CAN IDと64bit UID | UID取得時は返信の対象ID・既知UIDを照合しない | 更新前バックアップ・対象バス全6台・既知UIDを照合 |

公式PythonにはSocketCAN `can0`と別機種のBINパス・対象IDが固定されています。原文をそのまま実行せず、公式の成功時フレーム列に限定したCH340用実装を作成しました。r2は再送・途中再開を実装せず、異常時は転送を止めてポートを閉じる構成でした。後続版のSTART限定再試行と期限変更は成功記録を参照してください。DATAの再送・途中再開は使用していません。**Type11は更新モードへの移行命令であり、読み取り専用試験ではありません。**

## 準備時に比較したWindowsを使わない候補経路

| 候補 | 準備・r2初回試行時点の評価 |
|---|---|
| Jetson Linux＋SocketCAN | 対応するCANインターフェースとドライバー、上記差分の解決、対象・再送・復旧の検査が必要。公式Pythonは参照実装として利用可能だが、現在のCH340接続はSocketCANではない |
| x86-64 Linux＋公式MotorStudio | [Linux AppImage配布](https://github.com/RobStride/MotorStudio/releases/tag/v1%2C0%2C1)あり。ELFヘッダはx86-64（`e_machine=62`）でJetson ARM64にはネイティブ実行不可。互換Linux機、使用アダプターとRS05更新への対応確認が必要 |
| Mac／Jetson＋CH340 ATシリアル用実装 | 限定した成功経路を実装・模擬検証し、JetsonでID1を試行。r2はSTART無応答により未成立。後続のr5ではJetson＋CH340で全12台の更新が成立。Mac上の実機転送は未実施 |

r2初回試行時点では現構成で成功した経路はなく、公式GUIのエミュレーター実行やSTARTの自動再送も行っていませんでした。全12台の更新に用いた後続の限定経路は[成功記録](rs05-firmware-update-success-20260925.md)に記載しています。

## 準備時に定めた更新前後の照合方針

1. **更新前記録。** 対象IDとUIDを非公開記録で一致確認。FW、CAN ID／ホストID／速度、動作モード、機械ゼロ・オフセット・角度表現、通信タイムアウト、トルク等の制限、自動報告と周期値を保存する。未対応・未読取の項目は未確認のまま記録する。
2. **最初の1台。** 更新経路確定後、可能なら対象1台を通信バス上で分離して評価する。STARTはID＋UIDで対象を指定する設計だが、同一バスの他個体への影響を本構成で検証していない。全12台への一括更新は初回の評価対象にしない。
3. **更新後読取り。** 0.5.0.13、同じ個体・ID、STOPのmode0／fault0、設定値と角度の連続性を照合する。再起動要否・タイミングは確定した更新手順に従い、その後も照合する。古いパラメータ表の一括書戻しやFactory Restoreは行わない。
4. **回帰試験。** 単独の自動報告ON／OFF→停止・静穏→3台→6台の順で、更新前と同じ条件を比較する。後6台・全12台は各段階成立後。FW更新だけで20ms制御達成や実機駆動可能とは判定しない。

外部の関節校正データは履歴として保持し、更新後のゼロ・回転方向・角度を再確認します。マニュアルにはRS05の本更新だけを理由に磁気校正が必須との記載は確認できません。磁気校正・コギング校正・励磁を更新準備に含めません。

## メーカーへの質問案（未送信）

1. RS05 0.5.0.9から0.5.0.13への更新に使える、CH340版RobStride USB2CAN対応のLinux ARM64またはmacOS公式手段はありますか？ AT送信間隔、キュー制限、返信待ち条件も確認したいです。
2. 上表の失敗値`0x0F`／`0xF0`、Type13返信のType11／13、パケット番号の幅について、対象bootloaderに対する正しい仕様・修正版はどれですか？
3. 同一バス上で1台へID＋UIDを指定して更新した場合、他個体への影響はありませんか？ 切断・電源断時の復旧、再送の可否、更新後の電源再投入手順を教えてください。
4. CAN ID、ホストID、機械ゼロ・オフセット、磁気校正、制限値、通信タイムアウト、自動報告設定は保持されますか？ 版間で変わるパラメータを教えてください。
5. 0.5.0.13の自動報告競合改善は、他個体が報告中のType24 OFF不成立にも対応しますか？ `EPScan_time`の読取り値0の意味も確認したいです。

メーカーへの質問は未送信です。上記は準備時の質問案として保持します。r2の[初回試行結果](rs05-firmware-update-attempt-20260925.md)と、後続の[全12台更新成功記録](rs05-firmware-update-success-20260925.md)を区別してください。今回の成功は、未読取り設定の保持やあらゆる異常時の復旧を保証するものではありません。
