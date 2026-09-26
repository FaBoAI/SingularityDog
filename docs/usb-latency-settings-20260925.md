# txqueuelenとLatency Timerの実機確認 — 2026-09-25

**現在のRobStride USB2CAN 2台はCH340のシリアル通信経路であり、`can0 txqueuelen`とFTDIの`latency_timer`変更は適用対象ではなかった。設定は変更していない。**

[実機設定の記録](../evidence/usb-latency-settings-20260925.json) · [全工程の比較結果](hardware-performance-refinement-20260925.md)

| 確認項目 | 実機の結果 | 判断 |
|---|---|---|
| USB2CANの通信先 | `ttyUSB0`／`ttyUSB1`、両方`ch341-uart` | AT形式の個別17バイトをシリアル送信している |
| `can0` | Jetson内蔵`mttcan`、`DOWN`、tx queue length 10 | 現在のUSB2CAN経路とは別。1000への変更を実施しても現行経路の改善試験にはならない |
| `latency_timer` | 両方のtty／usb-serial sysfs経路に属性なし | FTDI用の1ms設定を流用できない。現在値が16msであるという意味でもない |
| USB省電力 | 両方`power/control=on`、`runtime_status=active` | 自動サスペンドはすでに無効。`autosuspend_delay_ms=2000`は現状の転送待ち時間ではない |
| USB転送 | 両方12Mbps、送受信はBulk、最大パケット32バイト | CANのビットレートやUARTのボーレートとは別の値 |

SocketCANはLinuxのネットワークインターフェースを使うが、現在のプログラムはUSBシリアルの文字デバイスを開いている。[Linux公式SocketCAN資料](https://docs.kernel.org/networking/can.html)

Linux 6.8のFTDIドライバーには`latency_timer`の読書き属性がある。一方、同版のCH341ドライバーには同じ属性がなく、CH341Aの小パケットを溜めないための処理はbaud設定の内部に実装されている。後者はチップ版数にも依存するため、このソース確認を実機レジスター値の検証とは扱わない。[FTDI実装](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/ftdi_sio.c)、[CH341実装](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/ch341.c)

今回の確認はsysfs・ネットワーク状態・モジュールとビルド記録の読取り、および公開ソースの照合であり、モーター指令・ドライバー交換・USB制御レジスター書込みは行っていない。現行経路では送信間隔、同時未返信件数、基板内での転送と応答欠落を切り分ける。設定名が似ている別デバイスを変更して改善扱いにはしない。

## `setserial low_latency`の実装確認

**実機で読み込まれたCH341モジュールのビルド元を照合した結果、`setserial /dev/ttyUSBx low_latency`による待ち時間短縮はドライバーに実装されていなかった。** NVIDIAのインストール済みヘッダーに対して別途ビルドされたモジュールだが、そのCH341ソースは公式Linux stable v6.8.12とSHA-256が一致した。インストール済みモジュールと保存されたビルド成果物のSHA-256も一致し、ロード済みモジュールのBuild IDとも一致した。

| 照合対象 | 結果 |
|---|---|
| 公式v6.8.12／ビルド元`ch341.c`のSHA-256 | `b286d49dd1f2a3a8f96188888b4df7869f65c4d9bead51d5291d50150b235951` |
| インストール済みモジュール／ビルド成果物のSHA-256 | `27c8f86b7fe0316bd56bb6161ac8e10dd3e0ae65b135b60bf71960bc345e23dc` |
| モジュール／ロード済みBuild ID | `4c4cea54eb05c74cbe1ef4ebb96b2c66c55b7d63` |
| ドライバー固有の`get_serial`・`set_serial`・`ioctl` | いずれも未登録 |

以下の共通TTY／USBシリアル処理は公式v6.8.12のソース確認であり、実機カーネル全体とのバイナリー同一性を示すものではない。`setserial`や設定ioctlは実行しておらず、実際のioctl返却値や応答時間の改善を測定した結果でもない。[公式CH341ソース](https://github.com/gregkh/linux/blob/v6.8.12/drivers/usb/serial/ch341.c)

| 上流の処理 | ドライバー固有の`get_serial`／`set_serial`がない場合 |
|---|---|
| `TIOCGSERIAL` | TTY層が構造体をゼロ初期化し、USBシリアル共通処理が`line`・`close_delay`・`closing_wait`を返す。固有処理がなければ`flags`は0のまま成功する |
| `TIOCSSERIAL` | USBシリアル共通処理は終了時の待ち時間2項目を扱う。固有処理がなければ`flags`を使わず、待ち時間を変更しない要求は通常0を返す。成功だけでは`low_latency`有効化の証拠にならない |
| CH341登録内容 | `get_serial`／`set_serial`の両方が未登録。共通処理もこの2つを汎用コールバックで補完しない |

根拠：[TTY ioctl処理](https://github.com/gregkh/linux/blob/v6.8.12/drivers/tty/tty_io.c#L2623)、[USBシリアル共通処理](https://github.com/gregkh/linux/blob/v6.8.12/drivers/usb/serial/usb-serial.c#L439)、[CH341登録](https://github.com/gregkh/linux/blob/v6.8.12/drivers/usb/serial/ch341.c#L838)。終了時の待ち時間を変更する場合には別途権限チェックがある。

TTY受信の`tty_flip_buffer_push()`は、v6.8.12では常にworkqueueへ処理を登録する。`low_latency`を指定すると一般のTTY受信が即時実行に切り替わる、という説明はこの版には当てはまらない。FTDIは独自にフラグを受け取り、機器のタイマーを1msに設定する実装を持つ。[TTY受信処理](https://github.com/gregkh/linux/blob/v6.8.12/drivers/tty/tty_buffer.c#L528)、[FTDI固有処理（v6.8）](https://github.com/torvalds/linux/blob/v6.8/drivers/usb/serial/ftdi_sio.c#L1377)

CH341のボーレート設定関数`ch341_set_baudrate_lcr()`内には、小パケット対策も含まれている。CH341Aが32バイトまでデータを溜める動作への対策として、チップ版数が`0x27`より大きい場合に**PRESCALERレジスターのbit 7**を設定する。LCRのbit 7ではない。`0x27`の一部機器では挙動が反転するとの注記もある。今回確認したのはこの処理を含むソースとの一致までで、実機のチップ版数やPRESCALER現在値は読んでいない。この処理がすでに動作していれば追加の調整項目にはならず、ビットの強制設定や効果量の断定はできない。[ボーレート／小パケット処理](https://github.com/gregkh/linux/blob/v6.8.12/drivers/usb/serial/ch341.c#L239)

残る確認候補は、USB受信完了・TTY受信・アプリ受信の時刻を分けた計測である。ボーレートを上げる案はUSB2CAN基板側との対応確認が必要で、USBのパケット待ちやCAN側処理を必ず短縮するものではない。本確認では設定変更も改善量の測定も行っていない。
