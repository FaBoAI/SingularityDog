# 角度校正と360°ずれの事前修正

ソフト側では、周期角の比較・電源投入時の枝選択・現在の生角枝へ戻す逆変換を実装した。**物理的な原点、符号、機械可動範囲の確認まで完了した意味ではない。** この実装とCLIはCANへ接続せず、設定を書き換えず、モーター出力を許可しない。

実装は [`angle_calibration_audit.py`](../runtime/singularitydog_hw/angle_calibration_audit.py)、保存ファイルの入口は [`audit_angle_calibration.py`](../tools/audit_angle_calibration.py)。既存の `StaticBranchComparison` を変更せず、独立した数値診断として追加した。

## `%360`を使う箇所

向きの比較には次の最短角差を使える。359°から1°は+2°、1°から359°は−2°となる。ちょうど±180°は方向が一意でないため判定保留とする。

```text
comparison_delta = remainder(raw_now - raw_reference, 2*pi)
```

ただし、この結果を直接QDDの目標角へ送らない。原点と正方向は別に校正し、数値変換は次の対応を保持する。

```text
q_model = sign * (raw - turns * 2*pi) + offset
raw_target = (q_target - offset) / sign + turns * 2*pi
```

例として現在の生角が361°、モデル角が1°の場合、モデルで+2°動かす候補は生角363°になる。3°をそのまま送る変換にはしない。符号−1も同じ式で扱う。ここでの `raw_target` は数値候補であり、CANフレームではない。RS05の通信モード・指令範囲・経路・負荷は駆動側で別に検査する。

## 電源On/Offのたびに同じ手動質問をしない方式

`EpochAngleMap` は電源投入後の12軸UIDと生角を受け取り、各軸について**確認済みの物理可動範囲に入る枝がただ1つ**であれば、その枝を固定する。機械可動範囲が360°未満で、原点・符号・誤差範囲も確認済みなら、この一意性を根拠として毎回の「一回転していませんか」を省ける。

必要な条件は以下。

- UID、組付けリビジョン、校正元のハッシュが一致する。
- 原点の誤差上限、符号、機械可動範囲を証拠ファイルと対応付けてレビュー済み。
- 新しいモーター電源epochと、そのepochで取得したUIDを明示する。Jetsonの起動IDだけでモーター電源epochを推定しない。
- 校正誤差を含めても候補全体が可動範囲内で、代替枝がない。

URDFの関節範囲だけから「実物は絶対にその範囲外へ動かない」とは判断しない。今回の保存ログではこの区別を保ち、枝の**診断候補**だけを出す。取付け変更やQDD交換時は該当軸の証拠を無効にする。他の軸の校正履歴は残す。

同じepochの連続取得中には枝を変更しない。生角の変化を `最大速度×実経過時間＋読取り雑音幅` と照合し、360°飛び・欠測・時刻逆転・UID変更を検出したら、その診断インスタンスを無効にする。読値を毎周期moduloに入れて異常を隠さない。ここでの速度・雑音・時間上限は呼出側が明示し、既存駆動監視の値を緩和しない。

## 過去ログを使った結果

9月27日の各脚カメラL字、同日起動後の12軸読取り、旧校正の方向記録をハッシュで照合した。12個体のUIDは一致した。現在の候補符号とモデル範囲を仮定した**数値上**の結果は以下。

|脚|足先側：ID / モデル角候補|上脚：ID / モデル角候補|付け根：ID / モデル角候補|
|---|---|---|---|
|右前 FR|1 / −45.116°|2 / +14.434°|3 / −2.373°|
|左前 FL|4 / −43.852°|5 / +16.053°|6 / +1.440°|
|右後 RR|7 / −38.062°|8 / +20.113°|9 / +15.854°|
|左後 RL|10 / −20.487°|11 / +8.755°|12 / +4.250°|

ID3のみ `turns=+1`、他11軸は0の一意な数値候補になった。従来の個別「ID3だけ360°減算」コードに依存せず、同じ一般式で再現した。保存済み生角は変更していない。

旧手動校正には12軸分の方向差が残っており、**11軸は現在の符号候補と一致**する。ID10は旧記録 `+1` と現在候補 `−1` が不一致で、優先して再確認する。これは既存の11軸を自動承認するものではなく、再利用すべき証拠の選別である。`needs_direction_review_ids` はレビュー未完の軸、`priority_direction_recheck_ids` は過去記録と矛盾して追加実測を優先する軸を示す。「レビュー未完」と「試験を一度もしていない」を分けた。

カメラL字は目視合わせで角度誤差の上限が記録されていない。インポートしたプロファイルの `uncertainty_rad=0` は純粋な数値比較用であり、物理精度ゼロを主張しない。`physical_uncertainty_known=false`、各reviewフラグ=falseのまま保存する。誤差不明の候補を実機へ自動昇格させない。

## 明日の実施順

|順|実施内容|まとめ方・判断|作業目安|
|---|---|---|---|
|1|12軸UIDと生角の無駆動取得|前後2バスを一度に取得。現在の個体と旧証拠を照合|1〜2分|
|2|過去方向記録のレビュー|11軸の一致記録を再利用。交換・再組付け・矛盾がある軸だけ追加|2〜3分|
|3|ID10の相対回転方向と原点誤差の確認|黒いQDD本体に対する金属出力部の回転を確認。脚全体の移動と区別。合図時間内に動かす方式でなく、A保持→Enter→B保持→Enter|3〜5分|
|4|物理可動域と原点の誤差上限を記録|既存写真・CAD・組付けと既知姿勢を照合。確認できない軸を無理に可動端へ押さない|未確認範囲による|
|5|電源再投入の枝照合|支持姿勢を保って40V Off/On前後の12軸を一括取得。UID・候補枝・同一epoch連続値を検査|2〜3分|

手順4を省略して機械可動域が既知になったことにはしない。元のL字の全脚やり直しを電源再投入のたびに要求する方式は不要。再撮影や再測定は、保存済み証拠の不足・矛盾・交換に対応する軸へ限定する。

物理角度を測れる場合は `fit_reference_observations()` へ、同一個体・同一epochのA/B（推奨はA/B/A戻し）を渡す。外部角度の変化5°以上、相対出力軸の確認、誤差上限、証拠ハッシュを明示する。±1のうち、すべての観測誤差区間を満たす符号が1つだけならoffset区間を出す。目視でも誤差を有界にした `visual_bounded` は扱えるが、精度不明や誤差0の入力は受け付けない。倍率を勝手に合わせないため、実角20°に対して読値180°などの矛盾は検出される。

## コマンド

保存済みスナップショットから再生する。`SNAPSHOT_HOME` は保存済みJetsonホーム、実行位置はリポジトリ直下。出力はGit管理外へ置く。

```bash
python3 tools/audit_angle_calibration.py \
  --history-root "$SNAPSHOT_HOME" \
  --output /tmp/angle-audit-first.json \
  --profile-output /tmp/angle-profile-first.json
```

新しい12軸の読取りには既存の `singularitydog_hw.motor_epoch_readonly_capture` を使う。このツールはType0/17だけを許可し、`--execute-readonly` がなければ計画を表示する。前後ポートと期待UIDファイルを先に確認して指定する。

```bash
PYTHONPATH=runtime python3 -m singularitydog_hw.motor_epoch_readonly_capture \
  --front-port "$FRONT_PORT" --rear-port "$REAR_PORT" \
  --expected-uids "$UID_FILE" --output /tmp/angle-current.json \
  --execute-readonly
```

取得済みJSONを再監査する。新しい読み取りごとに出力名を変える。profileのレビュー済み証拠は `evidence_files` のパスとSHA-256を照合し、書き換わっていれば中止する。

```bash
python3 tools/audit_angle_calibration.py \
  --profile /tmp/angle-profile-first.json \
  --capture /tmp/angle-current.json \
  --output /tmp/angle-audit-current.json \
  --policy-candidate-output /tmp/angle-policy-diagnostic.json
```

`--policy-candidate-output` は、全12UID一致・静止した読取り・一意な数値枝が揃った場合に、既存の `policy_shadow` と `native_pipeline_benchmark --calibration` が読める候補を作る。offsetへ `−sign×turns×2π` を含め、現在生角の枝を無出力推論のモデル入力へ対応付ける。この経路は物理レビュー未完でも**無出力診断のみ**に使える。元captureのboot・SHA-256・全生角・未検証項目を候補へ残し、`approved_for_runtime=false` のまま出力する。手編集で角度を合わせる必要はない。新しい電源epochや起動では、新しい静止読取りから候補を作り直す。駆動校正へコピーしない。

`--references references.json` を加えると、ID文字列をキーにした外部基準観測リストを追加fitする。結果は `reference_fits_by_id` に入り、失敗した軸だけ `REMEASURE_THIS_AXIS` と理由が出る。既存profileを書き換えない。各観測の必須項目は、`uid`、`boot_id`、`motor_power_epoch`、`source_sha256`、`relative_output_shaft_observed=true`、`physical_angle_method`、`raw_rad`、`model_rad`、`uncertainty_rad`。

## ソフトの確認範囲

新規テストでは359/0境界、179/−179、±180曖昧、正負符号、複数turn、可動域が広すぎる場合、誤差が境界を跨ぐ場合、電源epoch変更、同一epochの360°飛び、UID交換、欠測、時刻逆転、現在生角枝への往復変換を確認した。CLIは証拠ハッシュ改変と既存ログ上書きを拒否する。これらはオフライン検証であり、負荷を受けた駆動や立位の合格を意味しない。

```bash
PYTHONPATH=runtime python3 -m unittest discover -s runtime/tests -p test_angle_calibration_audit.py
python3 -m unittest discover -s tools -p test_audit_angle_calibration.py
```

## 9月28日：12軸をまとめて再照合

[`review_angle_epoch_series.py`](../tools/review_angle_epoch_series.py) は、同じハッシュ固定の未承認profileを使い、時系列のType0/17・無駆動12軸記録を一度に比較する。UID不一致、各時点の数値枝、最初から最後までの生角差とモデル角候補差を軸別に保存する。異なる記録を同じ**物理姿勢**とは仮定せず、40 V Off/Onをプログラムから観測したとも主張しない。結果は常に `NUMERIC_COMPARISON_ONLY`、`output_allowed=false` である。

```bash
PYTHONPATH=runtime:tools python3 -B tools/review_angle_epoch_series.py \
  --profile "$ANGLE_PROFILE" \
  --capture "$EARLIER_READONLY_CAPTURE" \
  --capture "$LATEST_READONLY_CAPTURE" \
  --output /private/tmp/angle-epoch-series-unique.json
```

Mac保存の9月28日R11とR22を比較した結果は、全12 UID一致・全軸で数値枝が一意。ID3は両記録で `turns=+1`、ID9は両記録で `turns=−1`、他10軸は0だった。R11とR22の脚姿勢は異なり、各軸のモデル角候補にも約2.9〜24.4°の差がある。従ってこの比較は姿勢再現性、実物の原点誤差、40 V Off/Onをまたぐ連続性の合格証拠ではない。

実機の最短無駆動確認は、箱で胴体を支え、QDD脱力・4足の離隔または安定接地・即時40 V Offを確認したうえで、前後USB2CANのby-pathと期待UIDを指定し、`motor_epoch_readonly_capture --execute-readonly` を1回実行する。現場のポートが以前の記録と一致しているか先に照合し、出力名を毎回変える。そのcaptureを上記のシリーズ監査と `audit_angle_calibration.py --profile ... --capture ...` に渡せば、**12軸のUID・静止読値・数値枝の可否はその場で判定できる**。原点・正方向・実可動域の承認には、外部角度の有界な誤差、相対出力軸の観察、取付けが変わっていない証拠を別途レビューする。既存のカメラL字を精度ゼロと見なしたり、URDF範囲だけを実際の機械可動域と見なしたりしない。

## 9月28日R8の一括レビュー結果

26要求・500周期の無駆動計測に添付した最新Type0/17記録を、同じ未承認プロファイルへ再照合した。12軸のUIDは一致し、数値上は全軸で枝候補が一つ。ID3は+1周、ID9は−1周、他10軸は0周である。この結果は40 V Off/On時の**物理的な**同一姿勢や電源世代を証明しない。記録の`motor_power_epoch`は`NOT_INFERRED_FROM_JETSON_BOOT`で、独立した電源世代証跡を結び付けていない。

監査出力に`batch_review_plan`を追加し、再取得の要否を項目別に分けた。旧方向記録と候補が一致する11軸はまず既存の原記録をレビューする。ID10だけ旧方向候補と矛盾するため、9月27日の相対出力部約90°下降のA/B記録と操作者の確認を優先して照合する。必要な場合のみ同軸A→B→Aを追加測定する。**12軸すべてで**原点誤差の上限と限定可動域の物理レビューはまだなく、旧カメラL字の目視値と数値上のモデル範囲だけでは承認しない。

電源枝を詰める最短経路は、機体の姿勢を保ったまま同一Jetson起動で12軸のType0/17を40 V Off/Onの前後に一括取得し、既存の`motor_power_epoch_manifest.py`で操作者の電源イベント・外部電圧証拠・同じ姿勢と出力軸が一回転していない観察を結び付けること。該当証拠がすでにあれば再取得せず、その原本を先に検査する。`audit_angle_calibration.py`単体は電源世代を認証しない。さらに原点と符号には、同一電源世代の外部角度基準を誤差上限付きで記録し、`--references`で複数姿勢の整合を確認する。上記の未承認監査は実出力の許可ではない。
