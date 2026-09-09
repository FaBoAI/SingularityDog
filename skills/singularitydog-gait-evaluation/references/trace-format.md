# 共通トレース形式 v1

`examples/synthetic_trace.json` がツール用の小さな合成例です。実測ログではありません。

トップレベルは `schema: singularitydog.trace.v1`、`sample_dt_s`、`requested_duration_s`、固定した単位ベクトル `direction_world_xy`、開始位置 `initial_position_world_m`、開始向き `initial_heading_rad`、`target_speed_m_s`、`samples` を持ちます。長さは m、時間は s、角度は rad、力は N です。

各サンプルは駆動・物理計算後、リセット前の状態です。時刻は最初が dt、以降は 2dt, 3dt…と連続します。開始前や良い区間だけを追加・選別せず、始動も含めます。

- `time_s`：開始からの時間。
- `position_world_m`、`velocity_world_m_s`：胴体の世界位置と速度、3 成分。
- `heading_rad`：胴体の yaw。ツールで連続角へ変換。
- `terminal`、`nonfoot_contact`：実際の終了と、足以外の失敗条件に相当する接触。明示的な真偽値。
- `feet`：`FL`、`FR`、`RL`、`RR` の 4 脚。それぞれ `clearance_m`、非負の `normal_force_N`、2 成分の `material_point_velocity_world_xy_m_s`。

物理刻みでのトルク・速度等の記録はこの 50 Hz 相当の形式とは別に保持します。ツールはそれを補間・生成しません。最初の失敗以降は評価せず、除外した行数を明示します。欠損や不規則な時刻はエラーにします。

この公開ツールは汎用形式に対する新規実装です。過去の非公開パイプラインのバイト単位の複製ではなく、元の生データを含まない公開済み測定値を再計算したという主張もしません。
