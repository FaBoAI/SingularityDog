# 10月8日の実機テスト

10月8日の入口・期間延長条件の照合結果は[本日の修正まとめ・準備表](today-validation-20261008.md)と[本日の準備証拠](../evidence/validation-preparation-20261008.json)を参照する。本日の790ファイル版はMacでPLAN・新規展開・全SHA照合済み。以下の10月7日準備時のキットSHAは履歴として保持する。

10月7日の追加高速化候補をJetsonへ配置し、実際の取得・推論・Type1送信で測る。最初の到達点は、箱を残した2秒→10秒→20秒の完走と全12軸の終了STOP。目安は接続と電源が安定している場合で90〜120分。配置・入力照合で差が出た場合は、その修正時間を別に取る。

## 10月8日午後の更新

現在の比較基準は900µs／window3／元の20ms期限。R22の標準診断では定常500周期最大19.833513msだったが、実出力と同じ結果取り出し経路のR25では4条件中、C++復号＋従来待ちの5周期だけが完走した。通知2条件の中止と、ID6の終了STOPの曖昧さも保持する。ユーザーが40V Offと箱支持維持を確認し、その後のOnで12軸を取り直した。最低34.949Vが既存35V下限を下回ったため、約30分の充電中はCAN周期試験を止め、C++推論を接続した新ソースの部品検証・設定照合・保存入力比較を進める。

次の実機計測は、新ソース自身の無出力5→501周期から取り直す。R24のモデル部品で確認した約214µsの短縮と、R25の5周期成功を、新ソースの実Type1成功へ転用しない。後述の890µs選択とnative pair延長は当初の準備記録として残し、現在の900µs・通常経路の試験とは区別する。[最新原票と残件](today-validation-20261008.md#r25実出力と同じ結果取り出し経路で4条件を比較)。

R27の820ソースはJetson上の147件と、実ARMモデルの保存501入力検証を通過した。選択したC++経路を既存の10回＋10回でウォームアップし、最初2回の12〜17ms遅延は今回のCPU記録では再発していない。通常observerの中央値は2.423→2.242ms、最大2.448ms。CPU比較と実CANの全工程は別に測る。

充電後の4回の読取りとSSH切断を保持し、再接続後の新収録は40.353〜41.756Vで元の範囲に入った。この820ソース自身の無出力5周期は完走、定常最大19.944460ms。501周期Aは295周期後に合流待ち期限で中止し、296周期目の全返信と全12軸異常時STOP・復元を確認した。予定期限の未達と処理超過を分け、worker開始遅延を調べてから次の測定へ進む。有効化・学習目標送信は未実施。外部入口の終了時照合に残る旧benchmark SHAは、1定数だけ修正し14件と独立レビューを通過、Jetson配置も完了した。

充電後は、現在の起動・40V電源区間・12軸UID・角度・35〜42Vを読み直し、この820ソースへ新しい入力を結ぶ。900µs／window3・C++復号ON・C++推論検査ON・完了通知OFF・通常observer・元20msを使用する。無出力5→501→501で原返信と復元まで確認してから、同じ設定の箱支持2→10秒へ進む。既存のゲイン・変位・鮮度・STOP監視は維持する。

## 10月7日時点の準備記録

| 対象 | 10月7日時点 | 今日すること |
|---|---|---|
| 追加高速化ソース・模擬検証 | ○ 954件成功・4件skip、保存501周期の同値性確認 | JetsonのPython／aarch64でビルドと関連検査 |
| 配置用source-onlyキット | ○ 789ファイル、10月8日の修正前まで全SHA一致。私有保存先へ複製済み | 本日の入口・20秒延長変更を含む新キットを新しい配置先へINSTALL。モデル・補正・現起動profileは別に結ぶ |
| 過去の無出力20ms実測 | ○ R20の定常500周期×2回は全処理20ms以内 | 新候補の資格には転用せず、5→501→501を取り直す |
| 新候補の無出力・実Type1 | × 未配置・未計測 | 下の順序で確認 |
| 荷重支持・自立・歩行 | × 未実施 | 小混合の完走後、荷重用設定と支持方法を別に確認 |

キットのmanifest SHAは `46532be17daf3f1e276d02ff9a811ee5fd070c6af0e5794bb79c97ce6862ba2d`、source archive SHAは `b5ca05a59fb6af7250414da6747386d471a59af6ede8f075817e3a2e285d9249`。789ファイルにはruntime/toolsを含み、モデル重み・native binary・現起動の出力承認は含めない。[変更とMac検証](additional-speed-optimization-20261007.md)・[証拠](../evidence/additional-speed-optimization-20261007.json)。

## 実施順

| 順番 | 目安 | 作業と記録 |
|---|---|---|
| 1 | 0〜25分 | 接続・配置・targetビルド。新しい私有ディレクトリへsource-onlyキットを配置し、Jetsonの実行Pythonで診断用transport・active transport・12軸encoderをビルド。source／binary SHA・ABI・clock・フレームの同値性を照合し、通知・取消・期限・Linux coordinatorの関連検査を実施 |
| 2 | 25〜45分 | 現起動・40V電源区間・12軸UID・現在角・電圧・IMU入力を読み、新profileへ結ぶ。音声案内付きゼロゲイン診断で200ms通信断保護と全12軸STOPを確認 |
| 3 | 45〜65分 | 成功時の性能設定を揃え、890µs・window3候補で無出力5→初回1＋定常500→同条件501再測定。原返信・未完了周期・設定復元まで回収 |
| 4 | 65〜95分 | 箱を残し、学習目標0.5%・最大1°・Kp3／Kd0.15で2秒→10秒→20秒。各段の原票・追従・終了STOPと現物の結果を確認し、次段の入力へ結ぶ |
| 5 | 95〜120分 | 集計と原因切り分け。全軸への送信まで、最後の返信まで、後処理までを別々に比較。実Type1の新通知が選択されたかも原票で確認 |

開始時にJetsonと40Vの充電、箱支持・4足接地・手離し・即時40V Off、現在姿勢から全12関節の約±3°の非接触経路をまとめて確認する。電源再投入・再起動・姿勢変更があれば、その区間の角度とprofileを取り直す。既存の実行条件と承認範囲で進め、同じ状態について不要な確認を繰り返さない。

## 必ず揃える設定

毎回[成功設定](jetson-best20-settings.md)と[設定・証拠JSON](../evidence/jetson-best20-settings.json)を読む。成功参照は900µs。今回はユーザー指定の890µsを比較候補として明記し、window3・period20ms・26要求を維持する。比較で900µsへ戻す場合も別原票として残す。

- CPU最低周波数・C7・EMCは成功時の一時scopeを使用し、終了時に読み戻して復元する。
- Python開始前のCPU maskは0〜4、mainはCPU4、workerはCPU0〜3。nice −10、switch interval100µs、数値演算threadは1。
- warmup前GC、warmup10回、pin後prime10回とobserver reset、周期中GC延期を選択する。
- absolute epoch、release spin500µs、timer slack1000ns、電圧の並列処理とprepared publicationを揃える。
- 周期内disk書込みを避け、trace・dispatch・CPU実行時間・全原返信・復元記録を保存する。

一般の [`dog_supported_fk_trial.py`](../tools/dog_supported_fk_trial.py) の既定は900µs／spin200µs。本日の修正では `--request-gap-us 890 --release-spin-us 500` と明示した比較理由を受け入れ、profileからnative pairを子プロセスへ渡す。新候補・現profile・ライブラリを先にPLAN照合し、実際の子プロセス引数とSHAを保存する。音声で「箱は残す」「動かす量」「試験時間」「開始の合図」を説明してから駆動する。

## 配置とC++候補の選択

source-only installerは既定PLAN。manifest／archive／installerのSHA照合後、Git外の新規ディレクトリへだけINSTALLする。過去のR20配置を保持し、今回の候補を別配置する。診断用の [`runtime/experiments/native_transport/build.py`](../runtime/experiments/native_transport/build.py) と実出力用の [`runtime/experiments/native_active_transport/build.py`](../runtime/experiments/native_active_transport/build.py) をJetsonで実行し、両libraryの隣接build recordとbinary SHAを確認する。Macのbinaryは使わない。

新しい元Future通知には `sda_future_readiness_abi` と `sda_wait_future_ready` の両方、ABI1が必要。両symbolがない旧libraryでは従来経路へ戻るので、ソースを配置しただけで通知が働いたとしない。片方だけ、またはABI不一致は拒否する。

12軸C++一括encoderは既存の明示選択候補。期間延長のloaderは先行試験のC++選択を要求するため、今日の2→10→20秒は最初からtarget encoderを選択した同じ候補で測る。Jetson上でheader／ABI／byte・拒否条件を確認し、[targetでのfile-onlyビルドと比較](additional-speed-optimization-20261007.md#jetsonでの次の計測)を使用する。Python生成の2秒比較へencoderだけ後付けして期間延長することはしない。

## 合否とレポート

無出力benchmarkは新実Type1と同じ経路ではない。現在の無出力取得待ちは従来の診断用待機を使うため、5→501→501だけで実Type1の元Future通知や中継削減の効果を確認したことにはしない。実出力JSON原票の `acquisition_future_notification.groups_created`／`wait_calls`、`voltage_future_notification.groups_created`／`wait_calls`、`input_acquisition_wait`、`voltage_join_wait` と実際の選択経路を照合する。

| 項目 | 集計方法 |
|---|---|
| 必須の制御経路 | 最古入力の取得→実推論→全12軸への最終host writeを20ms以内で確認。host write完了とCAN実到達は区別 |
| 返信・後処理 | 全軸返信までと全処理完了までを別記。既存の限定された返信後遅れ予算を使った周期は回数・量・密集度も保存 |
| 初回 | 初回1周期を別集計。全処理の初回許容を入力鮮度の免除へ読み替えない。R20の初回鮮度超過を新候補で再確認 |
| 連続性 | 開始間隔、予定期限超過、slot skip、返信欠落、未完了周期を残す。遅い周期を削除して成功率を作らない |
| 実出力 | 目標送信・実モデル呼出し・追従・正常な終了ランプ・全12軸STOP・現物の異常有無を確認 |
| 停止・再試行 | STOP不足なら40V Off。入力鮮度・返信欠落・電圧等で中止したら原因を分類し、新しい成立条件を確認して次試験へ進む |

最終レポートは「試験／完了周期／送信までの最大／返信までの最大／全処理最大／20ms超過／欠落／STOP／現物」の表にする。途中停止は停止理由と未完了周期を含め、次試験と別の行に残す。

20秒完走後、終了後501周期と期間延長を別途検討する。今回追加するnative pairの延長範囲は20秒までであり、30／60秒へは使わない。荷重支持は補助者と遮断担当の分担、または胴体直下の固定受けを準備し、荷重用保持設定で別に確認する。小混合・最大1°の成功を、そのまま立位や歩行の成功にはしない。
