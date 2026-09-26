# 実機試験前の I2S 音声案内

QDD を動かす試験の前に、Jetson から「テスト開始します」を再生します。音声再生に失敗した場合、音声付き起動コマンドは試験プログラムを開始しません。音声確認だけでは QDD に CAN 指令を送りません。

実装基板は MAX98357A の I2S アンプとスピーカーを使います。ピンヘッダーを基板の**裏面**へ取り付けているため、回路図のヘッダーを表面から見た番号と、Jetson へ挿した実際の物理ピン番号は各列で入れ替わります。実機では Jetson 40 ピンの I2S2 を使い、BCLK=12、LRCLK=35、DIN=38、DOUT=40 です。スピーカーへの出力に使うのは BCLK、LRCLK、DOUT で、DIN はマイク側です。I2C の IMU 用設定も残したまま I2S2 のピン設定を有効化し、Jetson を再起動してから再生します。

音声は [48 kHz・ステレオ・16-bit PCM WAV](assets/test-start-ja.wav) に固定しました。MAX98357A は 22.05 kHz の LRCLK に対応していないため、Mac の日本語音声で作った WAV を 48 kHz に変換し、左右へ同じ信号を入れています。2026-09-26 に実機のスピーカーから「テスト開始します」と**はっきり聞き取れた**ことを確認しました。初期の eSpeak-ng 音声は日本語として聞き取りにくかったため、確認済みの音声へ差し替えています。`aplay` の終了コードだけではスピーカーから音が出た証明にはなりません。再生経路は APE の ADMAIF1 → I2S2、デバイスは `plughw:CARD=APE,DEV=0` です。再生直前に I2S2 を I2S 形式・Jetson マスター／アンプ側スレーブへ設定します。

Jetson-IO が最初に生成した DTBO は機能名を `i2s2` に切り替えましたが、BCLK・LRCLK・DOUT が `tristate=1`、`gpio-mode=0` のままでした。実機では無音だったため、元の DTBO と `/boot/extlinux/extlinux.conf` を保存した上で、別名の [jetson-io-i2s2-output-r1.dtbo](jetson/jetson-io-i2s2-output-r1.dtbo) に BCLK=pin12、LRCLK=pin35、DOUT=pin40 の出力許可、DIN=pin38 の入力設定を記録しました。これは実機の **Jetson Orin Nano・L4T 39.2.1** で生成したオーバーレイです。再起動後の live pinmux で出力許可を確認し、4 秒の確認音と日本語音声の聞き取りに成功しました。この調整は QDD の CAN 制御設定には触れません。

Jetson で音声だけを試す場合:

```sh
PYTHONPATH=/home/jetson/SingularityDog/runtime \
  python3 -m singularitydog_hw.i2s_announcement \
  --wav /home/jetson/singularitydog-tests/audio/test-start-ja.wav --play-only
```

既存の監査済み試験を音声付きで起動する場合は、同じコマンドの `--play-only` を外し、`--` の後ろにその試験コマンドを置きます。試験の支持台・脚の離隔・即時 40V Off・同一起動のレビューは従来どおり必要です。新しく `tools/build_role_group_step.py active` で作るパッケージでは、`--announcement-wav` で検証済み WAV を指定すると、WAV と再生コードがハッシュ固定され、能動試験の呼び出し直前に自動再生されます。過去に作成済みの凍結パッケージは変更しません。

音声が聞こえない場合は QDD の試験へ進まず、I2S2 ピン設定、`I2S2 Mux`、ヘッダーの表裏とスピーカー接続を確認します。

2026-09-26 の音声付き全12軸・5秒保持では、操作者が案内を聞き取り、前後各100周期と全軸STOPを確認しました。続く[上脚4軸の鏡像方向試験](../docs/role-thigh-mirrored-20260926.md)は、後脚がほぼ10°動く一方、前脚がカーボンポールに接触して約3.5°で止まりました。音声経路の成功と、脚の掃引経路の安全性は別々に判定します。この後ろ向き経路への再試行やトルク増加は、干渉を解消するまで行いません。

その後、[前脚だけを顔側へ動かす試験](../docs/front-thigh-faceward-20260926.md)では、音声案内の後に右前−10.07°・左前+9.39°を達成し、全12軸STOPまで完走しました。操作者も前脚2本の動きと接触なしを確認しました。
