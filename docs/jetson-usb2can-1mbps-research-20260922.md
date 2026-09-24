# Jetson・RobStride USB2CANの1Mbps運用調査

調査日：2026-09-22。メーカー・販売元・Linux・pySerialの一次資料と保存記録を照合した。後半にRobStride公式GitHubの固定commitでの追加照合を記載。資料調査と同日に実施したJetsonの有限読出し試験は[実測報告](hardware-readonly-20260922.md)へ分け、CAN設定変更・モーター駆動は行っていない。

**採用方針は、CAN側1MbpsとUSBシリアル側921600bpsを組み合わせること。現基板で12軸50Hzを満たせるかは、通信方式を改善したうえで実測する。** 現在のCAN設定値はまだ確認できておらず、すでに1Mbpsの可能性もある。

## 1. 使用中の基板と通信経路

ユーザー指定品はSeeed SKU 100012096、RobStride USB-to-CAN Adapter Module。販売元の仕様はGD32 MCU＋CH340、シリアル側の上限は921600bps。さらに、公式ホストソフト向けでシリアル通信仕様書は提供されず、独自開発には標準CANアダプタを使う案内がある。製品詳細はページ内の動的表示用データにも掲載されている。[Seeed製品仕様](https://www.seeedstudio.com/Robostride-CAN-USB-Driver-Board-p-6708.html)

```text
Jetsonのアプリ／pySerial
  ↓ USB → ch341ドライバー → ttyUSB系ポート
CH340  ⇄  GD32 MCU：UART 921600bps・8N1（現コードの設定）
  ↓ GD32側のCAN制御・トランシーバー
CANバス：1,000,000bit/sを目標 → RS05 × 12
```

上図は公表部品構成と現行通信実装から整理した経路であり、基板回路図を確認した結果ではない。USB自体の転送速度、UART速度、CAN速度は別の値である。

| 確認対象 | 分かっていること | 未確認事項 |
|---|---|---|
| USB認識 | 実機記録はVID:PID `1a86:7523`、CH340 | 再接続時のポート・ドライバー・他プロセスの使用 |
| UART | 現コード921600bps・8N1で全12台の識別・読出しに成功 | 基板FWの版、バッファ容量、送受信の遅延 |
| CAN | RS05公式仕様・公式SDKの既定値は1Mbps | 今回の基板と12台の現在設定を裏付ける記録 |
| 周期 | 保存済み逐次方式は中央値81.204ms。同日の並行読出し最短成功条件は26.639ms | 連続運用、および全出力を含む50Hz動作 |

モーターの既定値の根拠は[RobStride公式SDK](https://github.com/RobStride/Python_Sample#hardware-setup)、実測は[CAN計測記録](can-readonly-timing-probe.md)。全台から返信が来た事実だけでビットレートを特定したとは扱わない。

## 2. 設定を混同しない

| 接続方式 | ホスト側の扱い | 今回への適用 |
|---|---|---|
| 現CH340基板 | USBシリアルへ独自のATバイナリフレーム | 現行の実機接続 |
| SocketCAN対応アダプタ | LinuxのCANネットワークインターフェース | 機種・FW・Linuxドライバーを確認して別途採用する方式 |
| Jetson内蔵TTCAN | SoCのCANピン・トランシーバー・mttcan | 今回のUSB基板とは別経路 |

`ip link ... bitrate 1000000`は実在するSocketCANインターフェース用で、`ttyUSB0`の設定方法ではない。`slcand`も別のシリアルプロトコルを前提とするため、ATバイナリ形式の現基板へそのまま使わない。[Linux SocketCAN](https://docs.kernel.org/networking/can.html)、[Linux SLCAN実装](https://github.com/torvalds/linux/blob/v6.8/drivers/net/can/slcan/slcan-core.c)

NVIDIAのpinmux・mttcan設定は内蔵CAN向け。CH340 USB接続のためにJetson-IOやCANピン設定を変更する手順にはしない。この資料の対象リリースと実機OSの一致も別途確認が必要である。[NVIDIA内蔵CAN資料](https://docs.nvidia.com/jetson/archives/r38.2/DeveloperGuide/HR/ControllerAreaNetworkCan.html)

現コードの`baudrate=921600`は維持する。これを1000000へ変えてもCAN側は1Mbpsにならず、販売元が示すUART上限を越える。

**現CH340基板のCANビットレート照会・変更命令は、今回調べた一次資料では確定できなかった。** 別基板のAT命令、推測した文字列、未確認のファームウェア書込みを試す手順は採用しない。

一方、旧MotorStudio説明書にはCH340のATモードとフレーム例が公開されている。「すべて非公開」ではなく、**フレーム構造の情報は一部あり、設定・FIFO・エラー通知等の完全仕様は未確認**という状態である。公式変換コードの`(CAN_ID << 3) | 0x04`もIDをシリアル用に包む処理で、速度設定ではない。[MotorStudio説明書241122・PDF1/7/8頁](https://github.com/RobStride/MotorStudio/blob/main/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6-Instructions/Instructions%20for%20using%20the%20Studio_241122.pdf)、[公式ID変換コード](https://github.com/RobStride/CAN-USB-data-conversion/blob/main/switch/mainwindow.cpp)

RS05にはモーター側のType23ビットレート変更があり、再通電で反映される。これはUSB2CAN基板の設定ではない。全台が応答している状態で先にモーターだけ変更すると通信を失う可能性があるため、アダプタを含む切替・復帰方法が確定してから扱う。[RS05公式マニュアル260713 §4.1.11](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf)

CAN 1Mbpsの根拠は、同一基板・FWのメーカー回答、仕様に対応した照会結果、または既知1Mbpsに設定した計測器による受動観測で取得する。計測器の追加は、終端・配線・listen-only対応を確認して別の無駆動試験として行う。

## 3. Jetson側で確認する情報

Linux v6.8の標準`ch341`ソースには`1a86:7523`が登録され、921600bpsを含むUART速度設定の実装がある。チップ／ドライバーの対応範囲と、この製品のFWが受け付ける速度は別である。Jetsonのベンダーカーネルで実際に結び付いたドライバーを確認する。[Linux v6.8 ch341.c](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/ch341.c)

次回の無駆動診断で使う、シリアルポートを開かない確認コマンド。`ttyUSB0`は列挙された実ポートに置き換える。必要なツールがない場合はその項目を未取得として記録する。

```sh
uname -r
cat /etc/os-release
cat /etc/nv_tegra_release
lsusb -d 1a86:7523
lsusb -t
modinfo ch341
readlink -f /sys/class/tty/ttyUSB0/device/driver
stat -c '%A %U %G %n' /dev/ttyUSB0
id
fuser -v /dev/ttyUSB0
ip -details link show type can
```

CANインターフェースが表示されなくても、CH340接続の不具合とは限らない。`modinfo`の有無だけで結論を出さず、sysfsとUSBの接続先を併せて確認する。`fuser`は権限によって他ユーザーのプロセスを見落とすため、出力なしだけでは未使用の証明にならない。権限・所有者・使用プロセスはローカル診断に残し、その出力全体を公開リポジトリへ追加しない。

pySerialが導入済みなら、USBの位置とポートの対応だけを取得できる。シリアル番号は表示しない。[pySerialの列挙機能](https://pyserial.readthedocs.io/en/latest/tools.html)

```sh
python3 - <<'PY'
from serial.tools.list_ports import comports
for p in comports():
    if (p.vid, p.pid) == (0x1a86, 0x7523):
        print(p.device, p.description, p.location)
PY
```

運用ポートは既存の`/dev/robstride-usb2can`が目的のデバイスへ解決されるか確認する。同じVID/PIDのCH340機器が増えた場合は、その値だけで自動選択しない。

JetPackの呼称だけで動作条件を推定せず、L4T・kernel・Python・pySerialの実版を記録する。Windows用CH341ドライバーをLinuxへ導入する手順や、認識済みドライバーの入替えを初手にはしない。

## 4. USBシリアル処理で改善できること

- Serialの`timeout=0.003`設定での`read()`は、必ず3ms待つ指定ではない。要求バイト数が揃えば戻る。フレームより大きい固定長を毎回要求してtimeoutまで待つ設計を避け、分割・結合受信を継続パーサーで処理する。
- 現方式のDTR/RTS無効化、フロー制御なし、8N1を維持する。ポートopen時の制御線変化はOS／ドライバー依存で、完全に防げる保証はない。
- `exclusive=True`に加え、すべての診断・校正・駆動ツールを同じlock規約に揃える。pySerialのPOSIX排他は協調的な`flock`で、無関係なプログラムまで強制排除するものではない。

以上の根拠：[pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)、[pySerial v3.5 POSIX実装](https://github.com/pyserial/pyserial/blob/v3.5/serial/serialposix.py)。現コードは[ReadOnlyCAN](../runtime/singularitydog_hw/can_readonly.py)。

FTDIで使われる`latency_timer=1`や`setserial low_latency`を、そのままCH340の高速化策とはしない。調べたLinux v6.8のCH341実装にはFTDI型の属性・専用設定処理がなく、USB-serial共通の設定もこれを代替しない。[CH341実装](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/ch341.c)、[USB-serial共通実装](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/usb-serial.c)

まずアプリの逐次往復・送信間隔・USBからの到着時刻を分けて測る。必要な場合だけ、無駆動の別計測でUSBトレースやホスト負荷を調べる。ログ取得自体が遅延へ影響するため、通常計測と区別する。

## 5. 20ms周期に対する伝送量の計算

以下は**線上の転送量の概算**で、実測応答時間や全制御周期の保証ではない。

- 現AT形式は8byteのCANデータに対し17byteのシリアルフレーム。8N1では170bit、921600bpsで約0.1845ms／フレームとなる。[現実装](../runtime/singularitydog_hw/can_readonly.py)
- Classic CANの29bit拡張ID・8byteデータは、3bitのフレーム間隔を含め、stuffなし131bit。stuffを保守的に見積もると約160bit／フレームとなる。下表はこの131〜160bitを用いた自前計算で、再送・エラー・追加のバス待ちは含まない。フレーム構造の参照：[TI SLLA270](https://www.ti.com/lit/an/slla270/slla270.pdf)

| 1周期の通信案 | CANフレーム総数 | CAN 1Mbpsの線上時間 | UARTの各方向のデータ | UARTの各方向の線上時間 |
|---|---:|---:|---:|---:|
| 12軸の指令＋各軸1返信 | 24 | 3.14〜3.84ms | 204byte | 2.21ms |
| 12軸の位置・速度を別要求＋各返信 | 48 | 6.29〜7.68ms | 408byte | 4.43ms |

前者を50Hzで行う場合、CANは概算最大19.2%、UARTは送信・受信それぞれ約11.1%の線上占有率。後者はCAN約38.4%、UART各約22.1%。自動報告・診断・停止等の追加通信は別予算となる。
USB処理、基板FWの処理・集約待ち、モーター応答、OSスケジューリングはこの表に含まない。UART送受信とCAN処理の重なり方もあるため、各列を単純合計して周期を決めない。

**帯域計算では20msに検討余地があるが、逐次方式の約81msをCAN 1Mbps化だけで解消できるとは言えない。** 保存計測の24往復×中央値3.164msは約75.94msで、逐次応答待ちが大きいと推測できる。個々の3msの内訳をUSBだけの責任とは断定しない。同日の有限試験ではUART設定を維持したまま並行化・間隔比較で26.639msまで短縮し、0.25ms間隔では返信欠落が生じた。[実測値と制約](hardware-readonly-20260922.md)

改善順序は次のとおり。

1. UART921600を維持し、CAN1Mbpsの根拠と無駆動基準値を取得する。
2. CANを1処理が所有し、連続受信・時刻管理・ID／種別／パラメーター照合を実装する。
3. 無駆動のType0/17で送信間隔を段階比較する。未完了の同じ要求を重ねず、欠落・古い返信・順序逆転・残留を検出する。
4. 駆動時のType2返信で位置・速度等をまとめて取得する案を検証する。Type17との単位・折返し・精度の一致を確認し、無駆動で得られると仮定しない。[RS05公式通信仕様](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf)
5. IMU・推論・出力検査を接続し、無出力60秒→10分、その後に前提を満たした限定出力で実送受信負荷を測る。

現在の駆動処理の5ms送信間隔は12軸で間隔だけでも55ms。過去の返信欠落対策なので、受信構造を直して欠落を測らず削除しない。返信後の4ms静穏待ちも同じように測定対象にする。
推論単体の保存実測は中央値7.13ms・最大9.33ms。通信だけ20msに収めても全処理は収まらない。周期p50/p95/p99/最大に加え、全12軸の取得幅、観測年齢、指令送信幅、欠落、最終有効指令から停止までを記録する。
送信側は期限切れの指令を破棄し、推論停止後に古い指令を再送してモーターwatchdogを更新し続けない。詳細な合格条件は[Deploy計画の通信・停止設計](deployment-roadmap-20260922.md#5-通信と20ms周期の事前検証)に揃える。

## 6. 配線側で確認すること

1Mbpsの高速CANは、幹線の両端に各120Ωの終端を置く構成が基本。12台それぞれへ120Ωを追加する方式にはしない。幹線・短い分岐・CAN_H/CAN_Lのツイストを確認する。[TIの物理層資料 §3・§4](https://www.ti.com/lit/an/slla270/slla270.pdf)

実機は分配基板を使っているため、幹線と分岐の実長、内蔵終端の有無・位置を図にする。各モーター・USB2CAN・分配基板に何Ωが入っているかは未確認。
電源をすべて外し、USB等からの給電もない状態で、CAN_H–CAN_L間の抵抗を確認する案を準備する。120Ωが2本なら合成は約60Ωだが、他回路の影響や接続状態があるため、この値だけで通信品質を合格にしない。
GND端子の接続は基板のピン表記に合わせ、USBグラウンドとモーター側の関係も確認する。通信のGND端子を40V電源端子と混同しない。

## 7. 不明点を解消するための収集項目

| 次に取得する情報 | 使い道 |
|---|---|
| 基板型番・基板版・FW版、CAN既定値と照会方法 | 「実機CANは1Mbps」の根拠を残す |
| MCUの送受信キュー容量、溢れ通知、CANエラー／bus-off通知 | 短い送信間隔で失われる箇所を切り分ける |
| CAN送信完了とUART受付の区別、返信タイムスタンプの有無 | ホストwrite成功をCAN到達と誤認しない |
| 終端抵抗・絶縁の有無、電源投入時の動作 | 実配線と復帰手順を固める |
| JetsonのUSB接続経路、実ドライバー、ポート競合 | ホスト側の不安定要因を特定する |

メーカーへの問い合わせ案（未送信）：使用するGD32＋CH340型基板について、CAN1Mbpsの既定／固定値、FW識別方法、CAN設定の照会・変更・復帰方法、921600bpsのフレーム形式、連続送受信の上限とエラー通知を確認する。未公開なら、Linuxで12台を50Hz制御する用途に対応する公開仕様の接続方式を確認する。

現基板で周期・欠落・停止条件が成立すれば継続利用する。CAN設定やエラー状態が確認できない、または改善後も周期を満たせない場合は、SocketCAN対応USB-CANを比較候補にする。販売元も標準アダプタを案内しているが、製品名だけで採用せず、**1Mbps・29bit拡張ID・Linuxドライバー・エラー通知・FW方式**を確認する。購入・交換は今回実施していない。

## 8. RobStride公式GitHubとの追加照合

ユーザー指定の[RobStride公式アカウント](https://github.com/RobStride)を起点に、以下の版へ固定して比較した。サンプルの実行・インストールは行っていない。

| リポジトリ | 確認したcommit | 今回の用途 |
|---|---|---|
| Python_Sample | `cbf977e56c842d57a65f3f17c1b1ecaef002c424` | パラメーター、係数、受信処理 |
| SampleProgram | `5f598686b05fcc527ee0fc0ea954f0afd652b234` | STM32でのCAN命令例 |
| CAN-USB-data-conversion | `72524311fbf4980d74ad53387c075cc50d8dd417` | シリアル用フレームへの変換 |
| Product_Information | `0f4ad74fdb67023e75bbcbeebecd6f1a003ce000` | RS05専用マニュアル260713 |

**採用できる根拠：** Type17の位置`0x7019`・速度`0x701B`はfloat32で、現在の個別読出しと整合する。Type2の整数値換算とは別経路なので、下記係数差は今回のType17測定値へは適用されない。[protocol.py](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/protocol.py)

**未解決の差：** RS05マニュアル本文44–45頁のType2速度範囲は±50rad/s、トルクは±5.5Nm。一方、SDKのRS05定数は33rad/s・17Nm。どのFWがどちらに対応するかは確認できていない。現行プロジェクトはマニュアル側を根拠としており、今回SDK側へ変更していない。Type1指令・Type2返信の換算を今後採用する際は、FWの根拠と独立した読出しとの整合確認が必要。[SDK table.py](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/table.py)、[RS05マニュアル](https://github.com/RobStride/Product_Information/blob/0f4ad74fdb67023e75bbcbeebecd6f1a003ce000/Product%20Literature/RS05/RS05User%20Manual260713.pdf)

また、SDK READMEはType24による自動報告をType2として説明するが、上記RS05マニュアル本文49–50頁の返信種別は`0x18`。この記述差だけから現物の返信を決めず、実FWの確認対象にする。自動報告の設定変更は未実施。[SDK README](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/README.md)

SDKはSocketCANを開く実装で、現在のCH340＋ATバイナリ変換器にそのまま接続するものではない。`receive_read_frame()`は種別17を検査する一方、要求ID・parameter indexとの一致をそこで照合せず、`receive()`の既定timeoutはNone。現在の複数台診断で使う有限期限・ID/index対応付け・不正応答で終了する処理を維持する。[bus.py](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/bus.py)

変換repoはQt上でCAN IDとAT形式を変換するツール。`AT`、`(CAN_ID<<3)|4`、DLC、データ、CRLFの構成は確認できるが、シリアル実送信・基板FIFO容量・最短送信間隔・short write処理の保証は確認できなかった。STM32サンプルのType23もモーターCAN速度の変更例であり、CH340のUARTや変換基板の速度設定命令ではない。[Qt変換コード](https://github.com/RobStride/CAN-USB-data-conversion/blob/72524311fbf4980d74ad53387c075cc50d8dd417/switch/mainwindow.cpp)、[STM32例](https://github.com/RobStride/SampleProgram/blob/5f598686b05fcc527ee0fc0ea954f0afd652b234/RS/Robstride01.cpp)

次の効率化候補は、1返信に複数の状態値を持つType2等の活用。ただし先に係数・返信種別・取得時刻・停止経路を照合する。Type17で間隔を削るだけの方法は今回0.25msで欠落し、同じ短縮を運用へ持ち込まない。

関連：[実機Deployの全体計画](deployment-roadmap-20260922.md) · [通信の実測と既存ツール](can-readonly-timing-probe.md) · [実機集計](hardware-results-20260921.md)

## 9. USB2CANを2台に分ける導入案

2026-09-23追記。以下は導入前の計画。その後、前脚ID1〜6／後脚ID7〜12の独立した2系統で全12個体を確認し、同時読取りと左前・右後の短時間駆動・停止まで完了した。改善版の全12軸取得幅は中央値29.744ms・最大36.801msで、連続20msは未達。CAN側の実ビットレートも未確認。[2系統の実測結果](dual-can-verification-20260923.md)

導入案ではCAN_H/CAN_Lを独立した6軸ずつの2本へ分離し、同じCANバスへ変換器2枚を接続する構成にはしない。所属IDとポートを固定して照合する。

1. 配線変更はモーター40VをOffにし、USB等からの給電も外して行う。既存のCAN_H/CAN_L共通接続を分離し、各バスの幹線・分岐・内蔵終端を確認する。終端と信号の共通基準・絶縁・電源GNDの関係は各機器のメーカー仕様へ合わせ、未確認の端子同士を結ばない。[TIの物理層資料](https://www.ti.com/lit/an/slla270/slla270.pdf)
2. 現在動作する設定を記録し、根拠なしにCANを1Mbpsへ変更しない。UART921600bpsを維持し、新旧アダプタと各6台のCAN設定が対応することを先に確認する。Type23をアダプタ設定の代用にはしない。
3. 同型CH340から一意のシリアル番号が得られない場合を想定する。USB接続位置と`by-path`を運用ポートへ対応付け、毎回、各バスの全6個体を事前固定した識別情報と照合する。VID/PIDや`ttyUSB`番号だけで選ばず、ケーブルの差替えでも取り違えを検出する。[pySerialの列挙機能](https://pyserial.readthedocs.io/en/latest/tools.html)
4. 各系統の有限なType0/17確認→2系統同時取得→IMU・無出力推論→前提を満たす有限出力と全12軸停止、の順に検証する。20msごとの位置・速度24項目更新、取得幅・観測年齢、推論、実送受信、通常／異常時停止を段階ごとに測る。片側のfault・欠落・期限超過で両側へ停止を要求し、全12軸の停止応答を確認する。片側継続・自動再試行は行わず、停止未確認を明示して電源遮断手順へつなぐ。

必要なruntime変更は次の範囲になる。いずれも、この追記による実装・駆動承認ではない。

- ポート・所属ID・期待個体をバス単位の設定へ分け、各バスを単一の送受信処理が所有する。バス別lockとserial排他を設け、既存の診断・校正・駆動ツールとの共通排他も維持する。
- 2系統を同時収集し、共通の単調時計で元の要求／受信完了とキュー到着を記録する。全24項目を因果的に統合し、遅着値を過去tickへ補完せず、片側の古い値だけで処理を続けない。
- 期限付きの全12軸指令と停止を統括する処理を追加する。watchdog・全旧値の読出し・読戻し、両系統の異常伝搬、停止確認、途中失敗時の保存を試験し、既存3軸runnerの対象数だけを増やさない。
- 両系統・IMU・推論・出力を含む周期分布、最悪値、欠落、指令送信幅、停止までの時間を記録する。USB経路やホスト処理の競合も測り、既存の5ms送信間隔を測定なしに削除しない。

**約13.3msは、既存の26.639msを均等に半分へ分けた未実測の概算にすぎない。** 2台化でその時間になる保証はなく、推論・出力・停止処理も含まない。20msの全処理成立、立位・歩行の準備完了とは扱わない。
