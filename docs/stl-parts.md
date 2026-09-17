# STL部材一覧

2026-09-17時点の**最新設計候補**です。標準Bホルダーを選ぶ場合、1台分は **黄色44個＋黒45個＝89個**。個別STLは86種で、同一の足先ホルダーを4個使います。これは設計必要数であり、印刷済み数、追加印刷数、実機に装着済みの構成を表しません。実際に装着した版は未確認です。

[購入部品BOM](bom.md) ／ [印刷・組立時の注意](printing/current/README.md) ／ [部品別の寸法・SHA・検査](printing/current/manifest.json) ／ [選定根拠と検査範囲](../evidence/stl-catalog.json)

**単位はmm、読み込み倍率は100%。全品の現物適合・締結保持力・耐荷重は未検証です。** 閉曲面の検査合格や印刷の完了を、実機への適合や強度合格へ読み替えません。色は印刷色で、材料はPLAを想定しています。

## 版の選び方

| 対象 | この一覧の優先版 | 旧版との関係 |
|---|---|---|
| B02 / B04 / B06 / B08 | 中央連結補強 r2（9/16） | 9/14の本体補強candidate_v3を継承する交換品。両方を加算しない |
| 黄色の可動脚20個 | 可動部補強 r1（9/13） | 旧黄色28点パックの可動脚20個と交換。上段クランプ8個は継続 |
| 取手11個 | 9/14の選定形状 | 取手1＋レールクランプ8＋横梁2。黄色44個の内数 |
| 共通底板 r2 | 9/15公開済み版 | B28～B33の旧6個を1個へ統合。B17～B24、B26/B27は再使用 |
| 顔の前面クランプ | r1（9/16） | B40を交換、B42を追加。B41は従来品 |
| 足先 | カセット r3（9/17） | 本体4＋選んだホルダー4。旧r2本体とr3ホルダーは互換なし |

1台分の構成は、取手、モバイル電源台、腹部スキッド、顔、機器支持部まで含む候補です。不要な機能を外す場合の最小構成はまだ確定していません。足先ホルダーA/B/Cは択一で、試験片は1台個数に入りません。旧パックの個数や配置済みプレートを、この表に重ねて加算しないでください。

## 機能別の数

| 色 | 機能 | 1台個数 |
|---|---|---:|
| 黄色 | 可動脚（4脚×5個） | 20 |
| 黄色 | 上段ポールクランプ（4組×上下） | 8 |
| 黄色 | モバイル電源台＋上クランプ | 5 |
| 黄色 | 取手＋レールクランプ＋横梁 | 11 |
| 黒 | Body側カラー＋本体マウント | 8 |
| 黒 | 横管・下段ポールクランプ | 16 |
| 黒 | 共通底板＋Jetson押さえ＋IMU台 | 4 |
| 黒 | 腹部スキッド＋キャップ | 6 |
| 黒 | 顔の支持枠＋前面リング＋クランプ | 3 |
| 黒 | 足先本体＋標準Bホルダー | 8 |
| **合計** | **試験片・代替品を除く候補構成** | **89** |

## 個別STL

各リンクから形状を確認し、ファイル画面のダウンロード機能で保存できます。外形は配布された印刷姿勢のX×Y×Zです。左右は機体自身の前向きを基準にし、FL＝左前、FR＝右前、RL＝左後、RR＝右後です。1行は1つの装着位置を表すため、同形状でも左右・前後をまとめて数えません。

### 黄色：可動脚・補強版

股・腿・脛リンクの12個は、配布姿勢で平坦な接地面積が0です。サポート・ラフトと初層をスライサーで確認してください。カラー8個も穴・張出しの確認が必要です。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| Y25 | 左前 脛側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y25_FL_calf_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y26 | 左前 脛リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y26_FL_calf_joint_part_reinforced_mm.stl) · 122×55×32 |
| Y27 | 左前 股リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y27_FL_hip_joint_part_reinforced_mm.stl) · 80×102×55.99 |
| Y28 | 左前 腿側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y28_FL_thigh_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y29 | 左前 腿リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y29_FL_thigh_joint_part_reinforced_mm.stl) · 163.98×80×66 |
| Y30 | 右前 脛側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y30_FR_calf_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y31 | 右前 脛リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y31_FR_calf_joint_part_reinforced_mm.stl) · 122×55×32 |
| Y32 | 右前 股リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y32_FR_hip_joint_part_reinforced_mm.stl) · 80×102×55.99 |
| Y33 | 右前 腿側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y33_FR_thigh_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y34 | 右前 腿リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y34_FR_thigh_joint_part_reinforced_mm.stl) · 163.98×80×66 |
| Y35 | 左後 脛側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y35_RL_calf_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y36 | 左後 脛リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y36_RL_calf_joint_part_reinforced_mm.stl) · 122×55×32 |
| Y37 | 左後 股リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y37_RL_hip_joint_part_reinforced_mm.stl) · 80×102×55.99 |
| Y38 | 左後 腿側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y38_RL_thigh_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y39 | 左後 腿リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y39_RL_thigh_joint_part_reinforced_mm.stl) · 163.98×80×66 |
| Y40 | 右後 脛側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y40_RR_calf_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y41 | 右後 脛リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y41_RR_calf_joint_part_reinforced_mm.stl) · 122×55×32 |
| Y42 | 右後 股リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y42_RR_hip_joint_part_reinforced_mm.stl) · 80×102×55.99 |
| Y43 | 右後 腿側カラー | 1 | 補強r1 | [STL](printing/current/yellow/Y43_RR_thigh_front_collar_reinforced_mm.stl) · 56×72×6 |
| Y44 | 右後 腿リンク | 1 | 補強r1 | [STL](printing/current/yellow/Y44_RR_thigh_joint_part_reinforced_mm.stl) · 163.98×80×66 |

### 黄色：上段ポールクランプ

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| Y01 | D20縦管クランプ 位置(-1,-1)・上段 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y01_d20_long_-1_-1_1_lower_mm.stl) · 40.96×22×12.75 |
| Y02 | D20縦管クランプ 位置(-1,-1)・上段 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y02_d20_long_-1_-1_1_upper_mm.stl) · 40.96×22×12.75 |
| Y03 | D20縦管クランプ 位置(-1,1)・上段 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y03_d20_long_-1_1_1_lower_mm.stl) · 40.96×22×12.75 |
| Y04 | D20縦管クランプ 位置(-1,1)・上段 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y04_d20_long_-1_1_1_upper_mm.stl) · 40.96×22×12.75 |
| Y05 | D20縦管クランプ 位置(1,-1)・上段 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y05_d20_long_1_-1_1_lower_mm.stl) · 40.96×22×12.75 |
| Y06 | D20縦管クランプ 位置(1,-1)・上段 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y06_d20_long_1_-1_1_upper_mm.stl) · 40.96×22×12.75 |
| Y07 | D20縦管クランプ 位置(1,1)・上段 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y07_d20_long_1_1_1_lower_mm.stl) · 40.96×22×12.75 |
| Y08 | D20縦管クランプ 位置(1,1)・上段 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y08_d20_long_1_1_1_upper_mm.stl) · 40.96×22×12.75 |

### 黄色：モバイル電源台

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| Y09 | モバイル台（短辺USB・壁20mm） | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y09_pla_center_bank_tray_20mm_usb_front_crossbars_lower_clamps_mm.stl) · 110.2×184×36.3 |
| Y10 | モバイル台 上クランプ 前 左 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y10_pla_center_bank_upper_clamp_front_left_mm.stl) · 16×44×18 |
| Y11 | モバイル台 上クランプ 前 右 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y11_pla_center_bank_upper_clamp_front_right_mm.stl) · 16×44×18 |
| Y12 | モバイル台 上クランプ 後 左 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y12_pla_center_bank_upper_clamp_rear_left_mm.stl) · 16×44×18 |
| Y13 | モバイル台 上クランプ 後 右 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y13_pla_center_bank_upper_clamp_rear_right_mm.stl) · 16×44×18 |

### 黄色：取手一式

取手の印刷強度と、取手で機体を持ち上げる耐荷重は未確認です。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| Y14 | 取手 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y14_pla_U_recovery_handle_mm.stl) · 187.96×85.5×24 |
| Y15 | 取手レールクランプ 前 左 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y15_pla_rail_clamp_front_left_lower_mm.stl) · 18×56×36 |
| Y16 | 取手レールクランプ 前 左 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y16_pla_rail_clamp_front_left_upper_mm.stl) · 18×56×36 |
| Y17 | 取手レールクランプ 前 右 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y17_pla_rail_clamp_front_right_lower_mm.stl) · 18×56×36 |
| Y18 | 取手レールクランプ 前 右 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y18_pla_rail_clamp_front_right_upper_mm.stl) · 18×56×36 |
| Y19 | 取手レールクランプ 後 左 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y19_pla_rail_clamp_rear_left_lower_mm.stl) · 18×56×36 |
| Y20 | 取手レールクランプ 後 左 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y20_pla_rail_clamp_rear_left_upper_mm.stl) · 18×56×36 |
| Y21 | 取手レールクランプ 後 右 下半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y21_pla_rail_clamp_rear_right_lower_mm.stl) · 18×56×36 |
| Y22 | 取手レールクランプ 後 右 上半分 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y22_pla_rail_clamp_rear_right_upper_mm.stl) · 18×56×36 |
| Y23 | 取手横梁 前 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y23_pla_fork_crossbeam_front_mm.stl) · 52×206×40 |
| Y24 | 取手横梁 後 | 1 | 既存形状・9/14選定 | [STL](printing/current/yellow/Y24_pla_fork_crossbeam_rear_mm.stl) · 52×206×40 |

### 黒：Body側モーター固定・中央連結補強

B02/B04/B06/B08は橋の下にサポートが必要です。9/14の本体補強版に加えてもう1個取り付ける部品ではありません。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| B01 | 左前 Body側 股関節カラー | 1 | collar_reinforced_r1 | [STL](printing/current/black/B01_FL_hip_front_collar_reinforced_r1_mm.stl) · 56×72×6 |
| B02 | 左前 Body側モーターマウント・中央連結補強 | 1 | 中央連結r2 | [STL](printing/current/black/B02_FL_rear_base_mount_crosslink_r2_mm.stl) · 73×162×93 |
| B03 | 右前 Body側 股関節カラー | 1 | collar_reinforced_r1 | [STL](printing/current/black/B03_FR_hip_front_collar_reinforced_r1_mm.stl) · 56×72×6 |
| B04 | 右前 Body側モーターマウント・中央連結補強 | 1 | 中央連結r2 | [STL](printing/current/black/B04_FR_rear_base_mount_crosslink_r2_mm.stl) · 73×162×93 |
| B05 | 左後 Body側 股関節カラー | 1 | collar_reinforced_r1 | [STL](printing/current/black/B05_RL_hip_front_collar_reinforced_r1_mm.stl) · 56×72×6 |
| B06 | 左後 Body側モーターマウント・中央連結補強 | 1 | 中央連結r2 | [STL](printing/current/black/B06_RL_rear_base_mount_crosslink_r2_mm.stl) · 73×162×93 |
| B07 | 右後 Body側 股関節カラー | 1 | collar_reinforced_r1 | [STL](printing/current/black/B07_RR_hip_front_collar_reinforced_r1_mm.stl) · 56×72×6 |
| B08 | 右後 Body側モーターマウント・中央連結補強 | 1 | 中央連結r2 | [STL](printing/current/black/B08_RR_rear_base_mount_crosslink_r2_mm.stl) · 73×162×93 |

### 黒：横管・下段ポールクランプ

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| B09 | D20横管クランプ 位置(-1,-1) 下半分 | 1 | D17_retained | [STL](printing/current/black/B09_d20_cross_-1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B10 | D20横管クランプ 位置(-1,-1) 上半分 | 1 | D17_retained | [STL](printing/current/black/B10_d20_cross_-1_-1_upper_mm.stl) · 40.96×22×12.75 |
| B11 | D20横管クランプ 位置(-1,1) 下半分 | 1 | D17_retained | [STL](printing/current/black/B11_d20_cross_-1_1_lower_mm.stl) · 40.96×22×12.75 |
| B12 | D20横管クランプ 位置(-1,1) 上半分 | 1 | D17_retained | [STL](printing/current/black/B12_d20_cross_-1_1_upper_mm.stl) · 40.96×22×12.75 |
| B13 | D20横管クランプ 位置(1,-1) 下半分 | 1 | D17_retained | [STL](printing/current/black/B13_d20_cross_1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B14 | D20横管クランプ 位置(1,-1) 上半分 | 1 | D17_retained | [STL](printing/current/black/B14_d20_cross_1_-1_upper_mm.stl) · 40.96×22×12.75 |
| B15 | D20横管クランプ 位置(1,1) 下半分 | 1 | D17_retained | [STL](printing/current/black/B15_d20_cross_1_1_lower_mm.stl) · 40.96×22×12.75 |
| B16 | D20横管クランプ 位置(1,1) 上半分 | 1 | D17_retained | [STL](printing/current/black/B16_d20_cross_1_1_upper_mm.stl) · 40.96×22×12.75 |
| B17 | D20縦管クランプ 位置(-1,-1)・下段 下半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B17_d20_long_-1_-1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B18 | D20縦管クランプ 位置(-1,-1)・下段 上半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B18_d20_long_-1_-1_-1_upper_mm.stl) · 40.96×22×12.75 |
| B19 | D20縦管クランプ 位置(-1,1)・下段 下半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B19_d20_long_-1_1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B20 | D20縦管クランプ 位置(-1,1)・下段 上半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B20_d20_long_-1_1_-1_upper_mm.stl) · 40.96×22×12.75 |
| B21 | D20縦管クランプ 位置(1,-1)・下段 下半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B21_d20_long_1_-1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B22 | D20縦管クランプ 位置(1,-1)・下段 上半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B22_d20_long_1_-1_-1_upper_mm.stl) · 40.96×22×12.75 |
| B23 | D20縦管クランプ 位置(1,1)・下段 下半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B23_d20_long_1_1_-1_lower_mm.stl) · 40.96×22×12.75 |
| B24 | D20縦管クランプ 位置(1,1)・下段 上半分 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/lower_pole_clamps/B24_d20_long_1_1_-1_upper_mm.stl) · 40.96×22×12.75 |

### 黒：共通底板・機器支持

共通底板とB17～B24・B26/B27は既存公開ファイルへのリンクです。IMU台B25は仮寸法で、現物への適合は未確定です。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| B25 | IMU取付台（仮寸法） | 1 | D17_retained | [STL](printing/current/black/B25_pla_underfloor_IMU_support_mm.stl) · 30×40×2 |
| B26 | Jetson端部押さえ 前 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/B26_jetson_edge_retainer_front_mm.stl) · 11.2×78×3 |
| B27 | Jetson端部押さえ 後 | 1 | 共通底板r2同梱・既存形状 | [STL](printing/common-deck-k1max-r2/printable_mm/B27_jetson_edge_retainer_rear_mm.stl) · 11.2×78×3 |
| COMMON_PLATE_R2 | Jetson＋Makita 共通底板 | 1 | 共通底板r2 | [STL](printing/common-deck-k1max-r2/printable_mm/common_equipment_plate_K1Max_r2_mm.stl) · 280×130×26 |

### 黒：腹部スキッド

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| B34 | 腹部スキッド 固定キャップ 左 前 | 1 | D17_retained | [STL](printing/current/black/B34_pla_prone_skid_cap_left_front_mm.stl) · 18×36×11.7 |
| B35 | 腹部スキッド 固定キャップ 左 後 | 1 | D17_retained | [STL](printing/current/black/B35_pla_prone_skid_cap_left_rear_mm.stl) · 18×36×11.7 |
| B36 | 腹部スキッド 固定キャップ 右 前 | 1 | D17_retained | [STL](printing/current/black/B36_pla_prone_skid_cap_right_front_mm.stl) · 18×36×11.7 |
| B37 | 腹部スキッド 固定キャップ 右 後 | 1 | D17_retained | [STL](printing/current/black/B37_pla_prone_skid_cap_right_rear_mm.stl) · 18×36×11.7 |
| B38 | 腹部スキッド 左 | 1 | D17_retained | [STL](printing/current/black/B38_pla_prone_skid_left_mm.stl) · 172×34.7×36 |
| B39 | 腹部スキッド 右 | 1 | D17_retained | [STL](printing/current/black/B39_pla_prone_skid_right_mm.stl) · 172×34.7×36 |

### 黒：顔フロントクランプ

B40はサポートが必要です。背面の基板・USBプラグ・フレキ空間には仮寸法を含み、現物で確認するための版です。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| B40 | 顔 背面支持枠 | 1 | 前面クランプr1 | [STL](printing/current/black/B40_display_rear_carrier_r1_mm.stl) · 110×110×80.8 |
| B41 | 顔支持 上クランプ | 1 | 従来B41再利用 | [STL](printing/current/black/B41_display_crossrail_upper_clamp_mm.stl) · 48×76×18 |
| B42 | 顔 前面リング | 1 | 前面クランプr1 | [STL](printing/current/black/B42_display_front_open_ring_r1_mm.stl) · 110×110×4 |

### 黒：足先カセット r3

Bホルダーは同じSTLを4個印刷します。A/Cを選ぶときはBの4個と置換します。先に下の試験片で嵌合を確認してください。

| ID | 部材 | 1台個数 | 版 | 個別STL・外形 mm |
|---|---|---:|---|---|
| FOOT_FL | 左前 足先本体 | 1 | 足先cassette r3 | [STL](printing/current/black/FL_FOOT_R3_CASSETTE_BODY_mm.stl) · 47.96×47.97×38 |
| FOOT_FR | 右前 足先本体 | 1 | 足先cassette r3 | [STL](printing/current/black/FR_FOOT_R3_CASSETTE_BODY_mm.stl) · 47.96×47.97×38 |
| FOOT_RL | 左後 足先本体 | 1 | 足先cassette r3 | [STL](printing/current/black/RL_FOOT_R3_CASSETTE_BODY_mm.stl) · 47.96×47.97×38 |
| FOOT_RR | 右後 足先本体 | 1 | 足先cassette r3 | [STL](printing/current/black/RR_FOOT_R3_CASSETTE_BODY_mm.stl) · 47.96×47.97×38 |
| CASSETTE_B | ナットホルダー B（標準） | 4 | 足先cassette r3 | [STL](printing/current/black/NUT_CASSETTE_B_STANDARD_mm.stl) · 23×33.2×5 |

## 足先の試験片・選択肢（89個に加算しない）

受け側試験片×1、10mmスペーサー×1、A/B/Cホルダー各1を同じ材料・印刷条件で試します。標準はB。Aはきつめ、Cはゆるめの比較用です。最終的なホルダーの種類は現物試験後に決めます。

| ID | 部材 | 個数の扱い | 個別STL・外形 mm |
|---|---|---|---|
| CASSETTE_A | ナットホルダー A（公差代替） | 選択時4個。Bの4個と交換 | [STL](printing/current/black/NUT_CASSETTE_A_FIT_OPTION_mm.stl) · 23×33.2×5 |
| CASSETTE_C | ナットホルダー C（公差代替） | 選択時4個。Bの4個と交換 | [STL](printing/current/black/NUT_CASSETTE_C_FIT_OPTION_mm.stl) · 23×33.2×5 |
| FIT_RECEIVER | ガイド嵌合試験片 | 装着0個。試験時1個 | [STL](printing/current/black/FIT_RECEIVER_SAME_GUIDES_mm.stl) · 30×32.8×13 |
| FIT_SPACER | 脚厚10mm 試験スペーサー | 装着0個。試験時1個 | [STL](printing/current/black/FIT_SPACER_10mm_FOR_M3x20_mm.stl) · 30×16×10 |

Bも上の[標準ホルダーSTL](printing/current/black/NUT_CASSETTE_B_STANDARD_mm.stl)を1個試します。試験で使った部品を適合確認後に再使用するか、作り直すかは実物の状態で判断してください。

## 旧配布物・履歴

次の公開済み配布物は履歴として保持しています。複合プレートやZIPには旧形状が含まれるため、最新版を一式印刷するときは上の個別一覧を使います。

| 履歴 | 収録内容 | この一覧との重複・差分 |
|---|---|---|
| [追加10部品](printing-next-10-parts.md) | 左前脚、顔支持、取手の旧パック | 左前脚とBodyマウント、顔支持は最新版へ置換。取手は同じ機能の1個で重複加算しない |
| [黄色28部品](printing-yellow-dense.md) | 可動脚20＋上段クランプ8 | 可動脚20は補強版へ置換。旧10部品パックとも左前脚5個が重複 |
| [共通底板 r2・11部品](printing-common-deck-k1max.md) | 共通底板1＋下段クランプ8＋Jetson押さえ2 | この一覧が同じ公開STL11個を再利用。別の11個を追加しない |

B02/B04/B06/B08の9/14補強版、旧個別黒41点、足先r1/r2はローカル作業の旧版であり、最新版と別の追加部品には数えません。今回の公開にその旧ファイル群や原CADを追加していません。

## 検査と残る確認

選択した個別90ファイル（装着候補86種＋公差代替2種＋試験片2種）について、STLのバイナリ長、有限座標、mm想定の外形、Z=0、閉曲面、1連結面、整った面向き、正の体積、元選定ファイルとのSHA一致を確認しました。STL自体は単位情報を持たないため、mmは元manifestの指定と寸法照合に基づきます。既存公開11ファイルを再利用し、新規79ファイルを追加しています。

この検査は自己交差の完全検出、実際のスライス、印刷誤差、全可動域の干渉、締結・疲労・耐荷重の検証ではありません。適合試験、スライサーの層プレビュー、実機の手動可動・段階的荷重試験が残っています。試作版同士を組み合わせた実機の完成も確認していません。

原写真、写真の派生画像、編集可能なCAD、機器メーカーの原モデルは含めていません。生成済み印刷STLと、公開用に選択した説明・検査記録だけを収録しています。
