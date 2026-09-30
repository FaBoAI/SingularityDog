# 学習モデルの規模とESP32への移植

現在のランナーが照合している `model_149.pt` のactorは、**74→128→128→64→12、隠れ層ELUの全結合ネットワーク**。形状とfloat32型は[モデルローダー](../runtime/singularitydog_hw/policy_shadow.py)で固定・検査する。推論では決定論的な平均出力を使う。

| 項目 | 規模 |
|---|---:|
| 推論に使う重み・bias | 35,148パラメーター |
| 学習時の標準偏差12個を含むactor state | 35,160パラメーター |
| 推論重み・biasのFP32サイズ | 140,592 bytes（約137.3 KiB） |
| 全パラメーターを1 byteとした理論値 | 約34.3 KiB。INT8実装のbias・scale等は別途必要 |
| 全結合層の積和 | 34,816 MAC／推論、50Hzで約174万MAC／秒 |

内訳は `(74×128+128)+(128×128+128)+(128×64+64)+(64×12+12)`。これはactorのみで、ELU、観測の組立、状態更新、位相・IK等の制御処理、通信、作業メモリー、ライブラリ、保存ファイル形式の付加情報を含まない。チェックポイント全体の容量や学習に使った全ネットワークの総数ではない。

## ESP32-S3で動かせるか

actorの規模は小さく、ESP32-S3への移植は検討できる。ESP32-S3は最大240MHzのデュアルコア、単精度FPU、512KB SRAMを持ち、PSRAM付きモジュールも選べる。ただし、メモリーとMAC数だけでは推論時間や全制御20msを確定できない。[Espressif公式データシート](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.html)

現在の[C++高速経路](../runtime/experiments/native_policy_overnight/model_call_fastpath/step_scalar.cpp)にはATen Tensorとdoubleによる状態計算が残るため、そのままESP-IDFへビルドできる構成ではない。最初はFP32のactor、同じ観測順・正規化・状態初期化、同等の前後処理を移植し、保存入力列でJetsonと比較する。INT8化はその後、出力誤差と閉ループでの影響を別に確認する。

またESP32-S3の内蔵TWAI（CAN）コントローラーは1系統で、外部トランシーバーが必要。現機体は前後2系統のCANなので、第二のCANコントローラー、別MCU、または配線・通信設計の変更が必要になる。[Espressif公式TWAI仕様](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html)

比較試験は、actor単体 → 状態を含む制御の保存入力再生 → 2系統CAN／IMUを含む連続20ms計測 → 支持付き実出力の順。ここではESP32向け実装や実測はしていないため、動作可能性の評価であり、50Hz制御の達成報告ではない。
