# PLA部品のCAD体積と中実換算質量

**対象は79個、CAD中実換算の合計は2,757.258g（約2.757kg）です。** L13本体CADの73個が2,621.995g、L14用の追加スキッド6個が135.264gです。ロボット全体の質量ではありません。

質量はCADの実体積に仮定密度1,240kg/m³（1.24g/cm³）を掛けた値です。穴や切欠きを除いたCADの材料体積をすべてPLAで満たす中実換算で、スライサーのインフィル・壁厚・サポート・接着補助形状は反映していません。実際の印刷使用量や完成重量としては扱いません。

**全79個のスライサー見積と実測重量は未取得です。** [機械可読の選択値](../evidence/pla-parts-mass.json)では`null`とし、0gや中実換算値で代用していません。

L14用スキッドは別途作成した追加試作部品です。この表は、その6個を元のFusion組立へ挿入・保存済みとする記録ではありません。脚先パッド、提案中のTPU95A脚先、カーボン材、金属、モーター、電子部品はPLA表から除外しています。

## カテゴリ別集計

| カテゴリ | 個数 | CAD体積 cm³ | 中実換算 g |
| --- | ---: | ---: | ---: |
| 床板・棚・支柱・床板ブリッジ | 9 | 155.697 | 193.064 |
| 関節部・カラー・フレーム取付部 | 52 | 1220.288 | 1513.157 |
| 回収用取手 | 1 | 177.669 | 220.310 |
| 取手用クロスビーム | 2 | 325.331 | 403.411 |
| 取手用レールクランプ | 8 | 233.170 | 289.130 |
| 下板下のIMU支持部 | 1 | 2.357 | 2.922 |
| 伏せ用スキッド | 2 | 91.680 | 113.684 |
| 伏せ用スキッド固定キャップ | 4 | 17.403 | 21.580 |
| **合計** | **79** | **2223.595** | **2757.258** |

## 各部品

名称と所属リンクは、左右・前後の同形状部品も区別するための自作部品IDです。FL=前左、FR=前右、RL=後左、RR=後右。表示は小数点以下3桁に丸め、JSONは元の選択値を保持しています。合計は丸め前から計算しているため、表示行の単純和と最終桁が異なる場合があります。

### L13本体CADのPLA部品（73個）

| 部品名 | 所属リンク | カテゴリ | CAD体積 cm³ | 中実換算 g |
| --- | --- | --- | ---: | ---: |
| d20_floor_plate | base_link | 床板・棚・支柱・床板ブリッジ | 46.532 | 57.699 |
| d20_floor_plate_front | base_link | 床板・棚・支柱・床板ブリッジ | 61.942 | 76.808 |
| d20_jetson_shelf | base_link | 床板・棚・支柱・床板ブリッジ | 33.431 | 41.455 |
| shelf_pillar_0 | base_link | 床板・棚・支柱・床板ブリッジ | 1.153 | 1.430 |
| shelf_pillar_1 | base_link | 床板・棚・支柱・床板ブリッジ | 1.153 | 1.430 |
| shelf_pillar_2 | base_link | 床板・棚・支柱・床板ブリッジ | 1.153 | 1.430 |
| shelf_pillar_3 | base_link | 床板・棚・支柱・床板ブリッジ | 1.153 | 1.430 |
| FL_rear_base_mount | base_link | 関節部・カラー・フレーム取付部 | 70.259 | 87.122 |
| FL_hip_front_collar | base_link | 関節部・カラー・フレーム取付部 | 9.132 | 11.323 |
| FR_rear_base_mount | base_link | 関節部・カラー・フレーム取付部 | 70.067 | 86.883 |
| FR_hip_front_collar | base_link | 関節部・カラー・フレーム取付部 | 9.132 | 11.323 |
| RL_rear_base_mount | base_link | 関節部・カラー・フレーム取付部 | 70.067 | 86.883 |
| RL_hip_front_collar | base_link | 関節部・カラー・フレーム取付部 | 9.132 | 11.323 |
| RR_rear_base_mount | base_link | 関節部・カラー・フレーム取付部 | 70.259 | 87.121 |
| RR_hip_front_collar | base_link | 関節部・カラー・フレーム取付部 | 9.132 | 11.323 |
| d20_long_-1_-1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_-1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_-1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_-1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_-1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_-1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_-1_1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_-1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_-1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_-1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_-1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_-1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_-1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_1_-1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_1_-1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_long_1_1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_1_1_lower | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| d20_cross_1_1_upper | base_link | 関節部・カラー・フレーム取付部 | 7.113 | 8.821 |
| l13_floor_bridge_right | base_link | 床板・棚・支柱・床板ブリッジ | 4.590 | 5.691 |
| l13_floor_bridge_left | base_link | 床板・棚・支柱・床板ブリッジ | 4.590 | 5.691 |
| pla_U_recovery_handle | base_link | 回収用取手 | 177.669 | 220.310 |
| pla_fork_crossbeam_rear | base_link | 取手用クロスビーム | 162.666 | 201.705 |
| pla_rail_clamp_rear_right_upper | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_rear_right_lower | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_rear_left_upper | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_rear_left_lower | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_fork_crossbeam_front | base_link | 取手用クロスビーム | 162.666 | 201.705 |
| pla_rail_clamp_front_right_upper | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_front_right_lower | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_front_left_upper | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_rail_clamp_front_left_lower | base_link | 取手用レールクランプ | 29.146 | 36.141 |
| pla_underfloor_IMU_support | base_link | 下板下のIMU支持部 | 2.357 | 2.922 |
| FL_hip_joint_part | FL_hip_link | 関節部・カラー・フレーム取付部 | 43.626 | 54.097 |
| FL_thigh_front_collar | FL_hip_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| FL_thigh_joint_part | FL_thigh_link | 関節部・カラー・フレーム取付部 | 70.240 | 87.098 |
| FL_calf_front_collar | FL_thigh_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| FL_calf_joint_part | FL_calf_link | 関節部・カラー・フレーム取付部 | 51.404 | 63.741 |
| FR_hip_joint_part | FR_hip_link | 関節部・カラー・フレーム取付部 | 43.626 | 54.096 |
| FR_thigh_front_collar | FR_hip_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| FR_thigh_joint_part | FR_thigh_link | 関節部・カラー・フレーム取付部 | 70.240 | 87.098 |
| FR_calf_front_collar | FR_thigh_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| FR_calf_joint_part | FR_calf_link | 関節部・カラー・フレーム取付部 | 51.404 | 63.741 |
| RL_hip_joint_part | RL_hip_link | 関節部・カラー・フレーム取付部 | 43.626 | 54.096 |
| RL_thigh_front_collar | RL_hip_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| RL_thigh_joint_part | RL_thigh_link | 関節部・カラー・フレーム取付部 | 70.240 | 87.098 |
| RL_calf_front_collar | RL_thigh_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| RL_calf_joint_part | RL_calf_link | 関節部・カラー・フレーム取付部 | 51.404 | 63.741 |
| RR_hip_joint_part | RR_hip_link | 関節部・カラー・フレーム取付部 | 43.626 | 54.097 |
| RR_thigh_front_collar | RR_hip_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| RR_thigh_joint_part | RR_thigh_link | 関節部・カラー・フレーム取付部 | 70.240 | 87.098 |
| RR_calf_front_collar | RR_thigh_link | 関節部・カラー・フレーム取付部 | 8.914 | 11.053 |
| RR_calf_joint_part | RR_calf_link | 関節部・カラー・フレーム取付部 | 51.404 | 63.741 |

### L14用に追加した伏せスキッド試作部品（6個）

| 部品名 | 所属リンク | カテゴリ | CAD体積 cm³ | 中実換算 g |
| --- | --- | --- | ---: | ---: |
| pla_prone_skid_left | prone_skid_link | 伏せ用スキッド | 45.840 | 56.842 |
| pla_prone_skid_cap_left_rear | prone_skid_link | 伏せ用スキッド固定キャップ | 4.351 | 5.395 |
| pla_prone_skid_cap_left_front | prone_skid_link | 伏せ用スキッド固定キャップ | 4.351 | 5.395 |
| pla_prone_skid_right | prone_skid_link | 伏せ用スキッド | 45.840 | 56.842 |
| pla_prone_skid_cap_right_rear | prone_skid_link | 伏せ用スキッド固定キャップ | 4.351 | 5.395 |
| pla_prone_skid_cap_right_front | prone_skid_link | 伏せ用スキッド固定キャップ | 4.351 | 5.395 |

## 確認した範囲

L13では改訂部品台帳から密度1,240kg/m³の73個を選び、所属リンクと名称の組でnative体積・質量台帳へ対応づけました。L14では同密度の6個を選択。79個すべてで体積と質量が正、部品IDが重複せず、`質量 = 体積 × 密度`が数値誤差内で一致することを確認しました。カテゴリ別と全体の合計も照合しています。

次の印刷準備で各部品のスライサー設定を確定し、フィラメント見積をこのCAD換算値とは別欄で記録します。印刷後は完成品を量り、必要に応じてシミュレーションの質量・慣性を更新します。今回の記録作成ではCAD、物性、学習条件を変更していません。

[次ループの仕様と未確定事項](next-loop-spec.md)
