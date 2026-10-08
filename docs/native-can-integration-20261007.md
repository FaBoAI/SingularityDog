# 10月7日：前後CANのC++統合と取得要求削減候補

**最新R20は890µs／window3／20msで無出力5→501→501を完了した。** 成功時の性能設定を揃えた定常500周期×2は最大19.948092／19.884256msで、処理20ms超過・返信欠落・スキップ0。初回20.141155／20.335789msと、再測定の初回入力→終了20.042661ms、次周期への予定期限持越し1件は別に残す。再測定の鮮度条件は不合格であり、新ソースの実出力開始条件を満たしたとは扱わない。[R18〜R20の詳細](report-led-optimization-20261007.md)・[匿名集計](../evidence/report-led-optimization-20261007.json)。以下のR16／R17結果は各時点の履歴である。

R16の前後native pair経路で、支持付き2秒・mix0.005・Kp3／Kd0.15・最大1°の新しい学習目標出力を実施し、`COMPLETE_SUPPORTED_OUTPUT`、エラー0、正常gain-down、全12軸の終了STOPを確認した。実95周期・モデル呼出し68回とType1原送受信を保存し、別途ローカルで検算した。実施後の物理状態の直接回答はまだ待ちで、異音・振動・滑り・姿勢維持を返信原票から推定しない。

steady94周期のwhole iterationは中央値19.4852015ms・最大19.820743ms、steady20ms超過0。startup20.274382msは既存の最初の周期の許容を1回使用した。checked sample ageは全周期20ms内だったが、strictな全周期whole20ms・周期開始間隔20ms・歩行／荷重／active controller50Hz資格は主張しない。最初のMac未完比較、R12の不完全返信、R13／R14の4件超過、R15のjoin失敗、最初のR16 501のstartup age不成立をすべて履歴として保持する。②の前周期返信再利用は保存比較と判定候補で、実行runnerには未接続。

R17のobserver複製候補は別比較として保持する。STOP代理5周期の試行は1周期完了後、2周期目のfront電圧34.948841Vが既存の35V下限を下回り中断した。未完原票を保存し、有効化・学習目標は送っていない。これは電源条件による中断で、タイミングの改善や悪化を測れた結果ではない。その後の満充電比較は上記R19／R20の記録へ分ける。

[ソースと検証集計](../evidence/native-can-integration-20261007.json)。R11のJetsonソフトウェア67件は成功した。R12のMac回帰は239件を実行してOK（Linux専用1件skip）、独立レビューの63件もOK（同1件skip）。R12のJetsonでは239件すべて成功し、32.719秒だった。Socketと模擬placementの結果を実機のタイミング証明へ置き換えない。

| 項目 | 状態 | 到達点・残件 |
|---|---|---|
| ① C++の前後CAN統合 | ○ ソース | persistent owner 2本、両bus事前検証、同じgenerationで解放、両bus終了後の原票公開を実装 |
| ① runner・設定への接続 | ○ ソース | 通常exchangeと出力phaseを接続。既存のfeedback→電圧の先行公開は従来経路を維持 |
| ① 故障時の原票・STOP | ○ 模擬 | 片bus異常の取消、元のdeadline、返信の曖昧さ、投入後例外のraw保存、join前STOP禁止を検証 |
| ① Jetson配置・ソフトウェア検証 | ○ R11 | 757ファイルを配置、aarch64 native ABI1をビルド、67件成功、local socket各100周期完了 |
| ① R12のJetson配置・ソフトウェア検証 | ○ | 759ファイルのkitを固定。両libraryビルド確認、239件成功 |
| ① 新経路の実CAN・20ms | × R12資格 | STOP代理5周期完了だが予定期限の持越し3件。元の501は最初の周期で不完全返信、完了0。短縮・資格は未証明 |
| ① R13のJetson・実CAN | △ | 247件成功、別保存のSTOP primeと5／501周期完了。501のsteady処理20ms超過4件・予定期限持越し6件で厳密資格は未達 |
| ① R14のJetson・実CAN | △ | 253件成功、STOP primeと5／501周期完了。501のsteady処理20ms超過4件・予定期限持越し5件を保持 |
| ① R15の明示880µs比較 | × 501未完 | 261件成功。STOP代理5周期完了後、501は84周期目のjoin期限で中断、83完了・84原票保存。全返信は完全、学習出力承認なし |
| ② 全12軸返信の再利用判定 | ○ ソース | 元の要求開始時刻、全軸generation、UID／boot／power／sourceを保持。予算不足は26要求へ戻す |
| ② 実行経路の14要求化 | △ 候補 | selectorと保存入力比較のみ。実行runnerには未接続。恒常14要求が成立するとは扱わない |
| ① R16のSTOP代理・保存review | ○ 限定 | 269件成功。最初の501のstartup age超過を保持し、条件不変のrepeat501完了から支持付き2秒の新graphをreview |
| R16の支持付き学習目標（履歴） | ○ 実原票、物理回答待ち | 実95周期・モデル68回、エラー0、全12軸終了STOP。startupの既存許容1回を保持 |
| ① R17のobserver複製・STOP比較 | × 電源条件で中断 | 保存501入力のcomponent比較は別記。STOP代理5は1完了・2原票保存、34.948841Vで下限停止。新しい20ms／実出力資格なし |
| 荷重移行・箱撤去・立位・歩行 | × 未許可 | 今回の支持付き小出力から昇格しない |

## ①の変更

- `runtime/experiments/native_active_transport/transport.cpp`：既存sessionを借りるpersistent 2 ownerのoptional pair API。両batchを検証してから同じ世代を解放する。既存の900µs／window3、各17byteのwrite、boot／FD／取消／Type2照合／期限を維持する。
- `runtime/singularitydog_hw/native_active_transport.py`：`ActivePhasePair`。各busの原records／statsは終了後に確定し、旧phaseの配列を上書きしない。投入が実行された後にPython例外が発生しても、書込み済み原票を回収する。
- `runtime/singularitydog_hw/policy_output_runtime.py`：`BusWorkers`から選択時だけpairを使用。Python側の結果取得を1つにまとめる。異常時は取消→native join→同sessionのSTOP。join未確認では競合するSTOPを送らず、未確認として残す。
- `runtime/singularitydog_hw/policy_live_profile.py`／`policy_output.py`：`native_phase_pair`と`--native-phase-pair`の一致、選択ソースの診断・review・変更不可bindingを検証する。既定値は無効。既存の小混合・最大1°・箱支持2／10秒の範囲に限定し、CPU4、ownerのCPU除外、timer slack 1000ns、warmup10＋prime10、absolute epochを必須にする。

C++ owner自身でCPU maskとtimer slackを設定・読戻し・復元する。R12ではpersistent Python coordinatorも開始前に準備し、その同じthreadでCPU0..3／slack1000nsの設定・読戻し・復元を記録する。join中断時は借用handle・FD・所有権を保持し、join完了や復元成功を偽って公開しない。Macの模擬placementはLinuxの実設定証明にならない。prepared feedback→電圧は従来ownerのままで、先行公開と期限の意味を変更しない。Type24自動返信、有効化の自動再試行、USBへの6frame一括burstは選択していない。

R12の診断入口は既定PLANを維持する。明示したpair選択と固定FK profileの一致、外部SHA固定のlibraryとsource inventory、支持付きdisabled、900µs／window3、CPU4とownerのCPU除外、slack1000ns、release spin500µs、warmup10＋prime10、開始前GCと周期中GC延期、元の20ms期限を必須にし、1 startupを含む5／501周期に限定する。pairへ渡すのは出力の全zero STOPだけで、feedback→電圧は同じsessionを従来Python ownerが扱う。

## ②で分かった制約

50Hzの前周期出力返信を毎回そのまま入力へ使うと、最も古い要求から次の最終送信まで、20ms周期にbus内の送信幅が加わる。900µs間隔で6軸を送る幅は最低約4.5msある。

周期nの最初の要求オフセットをa_n、最終writeまでの幅をdとすると、元要求から20ms以内にするには `a_n + d <= a_(n-1)` が必要になる。毎回少なくとも約4.5ms前倒しする必要があるため、固定周期内で恒常的に繰り返せない。

保存原票57遷移の単純再利用は57／57で超過し、24.157421〜24.939742msだった。残作業8.5msを仮定した投影は個別候補になり得るが、短縮した実行経路の実測でも、連続14要求の証明でもない。

`feedback_reuse_cadence.py`は、全12軸の原wireと要求開始時刻を変更不可のsnapshotへ保存する。明示opt-inかつ元の絶対期限に最終writeの予算が収まる場合だけ`REUSE_14`、余裕不足は`REFRESH_26`、fault／context不一致は`BLOCK`。送信直前と送信後にも再検算する。受信・公開時刻を新しい取得時刻へ付け替えない。実行への接続前にはIMU、角度枝、追従、電圧、watchdog、generation消費の監視を合わせた検証が必要。

## オフライン比較と再現

`tools/offline_native_pair_comparison.py`は、一時ディレクトリでC++をビルドし、local socketの模擬QDDへ26要求を送る。既存2pool、出力だけpair、通常3phaseをpairへまとめた候補を比較し、原票・source全文・SHAを保存する。実モデル・実機のprepared voltage overlapは模擬していない。

最初のMac上の各100周期要求比較は、既存方式60／61周期、出力pair 0／1周期、全通常phase pair 42／43周期で途中停止した。未試行周期や未完周期は20ms成功へ数えない。完了周期の中央値14.555063／14.550188msの差は約4.9µsで、一般化できない。失敗は模擬取得や出力のwrite境界で元の20ms期限へ達したもので、ID12だけの問題とは扱わない。この失敗記録を後の結果で消さない。

その後のR11 Jetson local socket比較では、3方式すべて各100／100周期が完了し、未完周期と完了周期のhost20ms超過は0だった。中央値は既存2pool 12.691820ms、出力だけpair 12.708785ms、全通常phase pair 12.709374msで、出力pairの速度優位は示していない。実CAN、実IMU、実モデル、prepared voltage overlapのタイミングを測った結果ではない。

R11の保存された読取り専用確認はエラー0で、全12軸がmode0・current0、最低電圧36.7383918762207Vだった。これはその確認時点の状態であり、R12の開始条件や5→501周期の完了証拠へ流用しない。有効化・学習目標送信は行っていない。

R12ソースkitは759ファイル、manifest SHA256 `65bb7a03c444ca90071e3ca19769b11625287cad610f2ceeae9dd0c347f52439`。Jetsonのactive／diagnostic両libraryのビルドと239件のテストを確認した。

R12の実CAN STOP代理5周期は`COMPLETE_DIAGNOSTIC`で全原票を保存した。startupは20.611598ms、steady4周期の処理時間は中央値19.7356615ms・最大19.877696msで処理期限超過0、slot skip0だった。一方、steadyの予定完了期限には3件の持越しがあり、最小slackは−0.423598msだった。CPU設定・GC・timer・native owner・Python coordinatorの復元、実行前後の起動同一性とsource不変を確認した。有効化・Type1は0。これは予定期限をすべて満たした20ms資格成功とは扱わない。

元のR12の501周期実行は、初回開始からの予定期限持越し例外を明示したタイミング比較として実施し、最初の周期で`ABORTED`、完了0だった。出力の各busで全6軸へ17byte STOPを書き込み、前半5軸は完全返信を受けた。最後のID6／12はそれぞれ11／17byteのprefixまで受信したが、残り6byteは未確認だった。prefixのmode0／fault0だけを完全返信や健全確認へ昇格しない。1件の不完全周期原票と各busの未完byteを保存した。

native owner／coordinatorのjoin・復元、CPU・GC・IMU・timer・Python switch intervalの復元、source不変、実行前後の起動同一性は確認できた。集計の`Native pair source/join/owner restoration proof incomplete`は交換エラーを含む包括的な不成立表示で、復元失敗が起きたという意味ではない。STOPだけを送り、有効化・Type1は0。予定例外付き比較から20msやactive controllerの資格を生成しない。

読取り専用の接続情報確認では、両CANアダプタは`ch341-uart`、USB vendor/product `1a86:7523`で、`latency_timer`項目はなかった。設定は変更していない。この情報だけで不完全返信の原因を特定しない。

R13は`--native-pair-prime-before-cycles`を明示した場合だけ、policy warmup／prime・owner配置・GC延期の後、observer armとcadence epochの前に全12軸のSTOPを1 phase送る。独自の20ms絶対期限を使い、原票は`native_phase_pair_prime`へ別保存する。測定周期の26要求・元入力からの期限・実releaseからの20ms期限は変更しない。primeのtimeout／fault／context不一致は測定開始前に中断する。交換エラーとsource／join／復元証拠の不成立も区別して表示する。既定ではprime無効で、有効化・Type1の経路はない。

R13の別ソースkitは759ファイル、manifest SHA256 `6248ce8800b3a6b52c22ea4c19d9baea583d2a063540474a17ac75ad3bd3c34c`。Macの11 suiteは247件を実行してOK（246件成功、Linux専用1件skip、21.133秒）。独立したprime候補の30件は6.144秒で全件成功し、対象範囲に具体的なblockerは見つからなかった。Jetsonの両libraryもビルドし、247件すべてが33.283秒で成功した（skipなし）。R12の5周期と失敗した501周期の原票は保持する。

R13の実CANでは、測定前のSTOP primeが8.225137msで完了し、各busの6返信すべて17byte、join成功、独自期限20ms、cadence epoch前の終了を原票で確認した。primeを測定周期へ数えず、別保存する。その後の5周期も`COMPLETE_DIAGNOSTIC`で完了した。startupは20.488077ms、steady4周期の処理時間は中央値19.7511975ms・最大19.838091msで処理期限超過0、slot skip0だった。一方、steadyの予定完了期限には2件の持越しがあり、最小slackは−0.352278msだった。全5周期の最古入力から最終host writeまでは最大16.769011ms、最終返信までは最大19.590837ms。元の期限を変えていない。

native owner／coordinatorのjoin・復元、CPU・GC・timer・IMU・Python switch intervalの復元、source不変と実行前後の起動同一性を保存原票で確認した。独立した電源連続性の確認は未証明。有効化・Type1・学習目標送信は0。予定期限の持越しがあるため、この5周期を20ms資格や速度向上の証明へ昇格しない。R13の501周期も`COMPLETE_DIAGNOSTIC`で501／501周期が完了し、エラー0、周期ごとの全12軸STOP返信6012件を完全保存した。startupは20.283592ms。steady500周期のwhole iterationは中央値19.6724035ms・最大20.040134msで、20ms超過が4件（0.8%）あった。予定完了期限の持越しは6件、最小slackは−0.113332ms、slot skipは0だった。steady開始間隔499件のうち257件が厳密な20msを超え、最大20.075526msだった。

全501周期の最古入力から最終host writeまでは最大16.594462ms、最終返信までは最大19.363438msだった。これらの入力年齢の値と、後処理を含むwhole iterationや予定期限の判定は分ける。測定前primeも全12返信完了・join成功で別保存した。native owner／coordinator・CPU・GC・timer・IMU・Python switch intervalの復元、source不変、実行前後の起動同一性を確認した。全collectionのSHAを検証し、25,782,004byteを保存した。期限は変更せず、有効化・Type1・学習目標送信は0。厳密な20ms・active controller資格と速度向上は未証明。

R14はSTOP wireから返信headerへの完全一致mapと事前生成STOP batchを使う別ソース候補で、独立した候補36件は7.386秒で全件成功した。対象範囲に具体的なblockerは見つからず、STOPの許可bytes・軸順・fault／mode／source／destinationの拒否を維持している。deadline・Future・joinの再設計は選択していない。R14 kitは759ファイル、manifest SHA256 `029a18b9b93ee7d523cb02f38cca06ac1c53811e9736ea9fab6c2d8961a70474`。Macの253件はOK（252件成功、Linux専用1件skip、22.642秒）、Jetsonの253件は33.352秒で全件成功した。active／diagnostic両libraryをビルドし、C++ sourceとbinaryのSHAはR13と同じだった。R12の失敗とR13の超過を消さない。

R14の実CAN STOP primeは8.204951ms、全12返信完了・join成功で別保存した。測定5周期も完了し、startup20.246078ms、steady4周期のwhole iterationは中央値19.4869385ms・最大19.876845msで処理期限超過0、slot skip0だった。予定完了期限の持越し1件、最小slack−0.184621msを保持する。最古入力から最終host write／最終返信は最大16.547957／19.331380ms。owner／coordinator・CPU・GC・timer・IMU・Python switch intervalを復元し、source不変と実行前後の起動同一性を確認した。原票collection346,742byteのSHAを照合し、ローカル原票audit2056項目もすべて成功した。host writeは物理CAN送信完了の計測ではない。有効化・Type1・学習目標送信は0で、厳密20ms資格や速度向上の証明にはしない。R14の501周期も501／501で完了し、エラー0、steady500周期のwhole iterationは中央値19.568056ms・最大20.058959msだった。20ms超過4件（0.8%）、予定完了期限の持越し5件、最小slack−0.071927ms、slot skip0を保持する。startupは20.281687ms。steady開始間隔499件のうち245件が厳密な20msを超え、最大20.094288msだった。全501周期の最古入力から最終host write／最終返信は最大16.658759／19.418796msで、測定されたこれらの入力年齢は全周期20ms以内だった。whole iterationと予定期限の超過をこの値で取り消さない。

測定前primeは8.163243ms・全12返信完了・join成功でcadence epoch前に終了し、別保存した。setupを含む原票auditでは13,050の重複なし要求と完全返信を照合し、180,616の整合性項目はすべて成功した。collection25,782,369byteの全SHAを検証した。owner／coordinator・CPU・GC・timer・IMU・Python switch intervalの復元、source不変と実行前後の起動同一性も確認した。期限は変えず、今回の有効化・Type1・学習目標送信は0。独立した電源連続性は未証明。

R13→R14のsteady中央値は0.1043475ms低かったが、whole iterationの20ms超過は両方4件だった。単独実行間の記述値であり、因果的な改善・大きなC++速度向上・厳密なwhole iteration20ms・active controller資格を示さない。成功時の`ExchangeError`先行生成と完了済み`Future`の二重bridgeの削減は次の限定候補として挙がったが、R14時点ではその変更は行っていない。基準の成功設定は変更していない。

R15はユーザーが明示した880µs／window3の比較候補で、900µsの成功基準・既定値を変更しない。pairのprofile・PLAN・disabled sessionは整数880または900だけを受け付け、実際の選択間隔をnative sessionまで渡す。STOP bytes・fault／mode検証・元の20ms期限・join・gain／freshness capは変えていない。独立した51件は10.013秒で全件成功し、Jetsonの261件も35.570秒で全件成功した。759ファイルのmanifest SHA256は`6192279c8ad4f6195e3fab7870b7a860ef6cb64660cd3ecec247c7d6b334eab2`。両libraryをビルドし、C++ binaryはR14と同じだった。

R15直前の読取り専用確認は全12軸mode0・current0、最低35.931339263916016Vだった。独立した全12軸ゼロゲイン通信断watchdogは`COMPLETE_COMMAND_LOSS_DIAGNOSTIC`、STOP確認成功、200ms／4000tickの読戻し成功、disable返信上限の最大228.879175msだった。この別試験の最低電圧は35.8962516784668V。R14からR15へのcommissioning依存ソース21件のSHA一致を確認した。この試験だけで有効化とゼロゲインType1を送り、正ゲイン・学習目標・USB抜去試験は行っていない。直接の試験前確認と試験後の「音声あり・異常なし・状態維持」を保存した。旧physical approvalや未来の学習出力の観察へ置き換えない。

R15のSTOP primeは8.122872ms、5周期は完了した。startupは20.276906ms、予定slack−0.296363ms。steady4周期は中央値19.4605225ms・最大19.493484msで処理超過・予定期限持越し・slot skipは0だった。最古入力から最終host write／返信は最大16.647742／19.358567ms。原票346,861byteのSHAを照合し、154の重複なし完全要求・返信と2056項目の整合性を確認した。4 steady周期だけを501周期のadmissionへ昇格しない。

元のR15 501実行は予定持越し例外を付けずに行い、84周期目に`Proxy output pipeline exceeded 20 ms hard deadline at proxy output join`で中断した。完了83周期と失敗した84周期目の原票を保存した。完了steady82周期は中央値19.377653ms・最大19.775445ms、処理超過・予定期限持越し・slot skipは0。startupは20.122787ms、予定slack−0.142244msだった。未試行417周期と失敗周期を成功へ数えない。

84周期目も各busの6 STOP要求・返信はすべて17byteで完全、mode0／fault0、native owner statusは両方0だった。最終返信は元のdeadlineより0.281005ms前、両owner終了は0.280205ms前だったが、Python join開始はdeadlineより8.127µs後だった。owner終了からjoin開始まで288.332µsかかり、期限判定時に両Futureは完了済みで待機試行0だった。元のdeadlineによる失敗を保持し、返信欠損・CAN fault・復元失敗とは扱わない。この観察だけで遅延原因を断定しない。

別保存primeは8.169958ms、collection4,369,377byteのSHAを照合した。2208の重複なし要求・完全返信、partial slot0、30,467の整合性項目はすべて成功した。native owner／coordinatorのjoin・復元、CPU・GC・timer・IMU・Python switch intervalの復元とsource不変を確認した。原票整合性の成功は501周期完了やactive admissionではない。R15は新しい支持付き2秒・最大1°・Kp3／Kd0.15・mix0.005の学習出力に必要な完全501診断を満たしておらず、file-only review builderも承認を作らない。R12の不完全返信、R13／R14の20ms超過、R15のjoin失敗をすべて保持する。

R16は成功時の不要な`ExchangeError`先行生成を除き、真正な完了native `Future`を同じgenerationとjoinの証拠に結んで使う。合成`Future`の二重生成と完了結果の再変換を除いた。ready／takeoutの元の時計・deadline、STOP検証、取消・raw保存・join／復元は保ち、phase journalのdeepcopy削減は選択していない。C++ transport・900µsの既定値・成功基準は変えず、880µs／window3の明示候補を継続する。760ファイルのmanifest SHA256は`f5bbebdffc7f6d4cc2839113fdec18aa42158f58da604f02141e6568a6d06f01`。独立したwrapper36件はOK（Linux専用1件skip、3.381秒）、candidate44件は7.955秒で全件成功。Jetsonの269件も38.509秒で全件成功した。

R16のSTOP primeは8.044222ms、5周期は完了した。startup20.002774ms、steady4周期は中央値19.4146025ms・最大19.494335msで処理超過・予定期限持越し・slot skipは0。最古入力から最終host write／返信は最大16.286342／19.104332ms。原票346,907byteの7 SHA、154の完全要求・返信、2056の整合性項目を照合した。

最初のR16 501は501／501で完了した。startup20.394194ms、steady500周期は中央値19.385250ms・最大19.830200msで処理超過0、予定期限持越し1件、最小slack−0.021888ms、slot skip0だった。最古入力から最終host write／返信は最大16.677696／19.498024ms。しかしstartupのcycle endから最古入力までの実timestamp差は20.072035msで、既存の支持付きrare-jitter2秒reviewのchecked sample age20msを72.035µs超えた。全返信と完了501をこの不成立の取消に使わない。別保存prime8.080963ms、原票25,781,419byteの7 SHA、13,050の完全要求・返信、partial0、180,616の整合性項目を確認した。この最初のreview不成立を保持する。

同じR16 source・profile・driver・capsを使い、例外flagなしで別保存したrepeat501は501／501で完了した。startup19.977864ms、steady500周期は中央値19.381296ms・最大19.839822msで処理超過・予定期限持越し・slot skipは0、最小slack0.147463ms。最古入力から最終host write／返信は最大16.391674／19.107672ms、cycle endまでのchecked ageは最大19.693228msだった。strictなsteady開始間隔20msは499件中252件で超え、最大20.005170msで、周期間隔の厳密一致やactive controller50Hz資格は主張しない。prime8.053199msと測定周期を分け、原票25,781,091byteの7 SHA、13,050の完全要求・返信、partial0、180,616の整合性項目を確認した。owner／coordinator・CPU・IMU・Python switch intervalの復元、source不変と起動同一性も保存した。

repeat原票、元の角度・zero・provisional IMU比較、現在の直接確認と独立ゼロゲインwatchdogを結び、数値capsを変えないfull saved reviewが成立した。R14 commissioning依存21ソースはR16とも同じSHAで、変更されたnative wrapperはその依存に含まれない。旧physical approvalを現在の観察へ読み替えず、新profileは支持付き2秒・最大1°・Kp3／Kd0.15・mix0.005だけに限定した。全source固定・full approved loaderのfile-only PLAN後に、明示した実出力を1回だけ行った。

実出力は`COMPLETE_SUPPORTED_OUTPUT`、エラー0、正常gain-down、全12軸STOP確認だった。620 journal項目には重複なし2674要求を保存し、Type1の2304件すべてが17byteの実writeと、motor source／host FDが一致する完全Type2返信を持つ。これにはfeedback hold・setup・gain-downも含み、学習出力phaseは576要求。全Type1返信はmode2／fault0だった。wire gainはKp3／Kd0.15以下、速度・feedforward torqueはzero符号化、正ゲインtargetのinitial rawからの最大差は0.2417727148°で1°以下。これは送信targetの値で、身体の実移動や追従精度を証明しない。終了時の12 STOPは各17byteのmode0／fault0返信と一致した。

95周期の元timestampをacquisition・output wire／IMUに照合し、モデル68回はstarting20＋active48と一致、gain-down27周期では新規推論を行わない。startup20.274382msは既存first-cycle許容1回を使用し、steady94周期は中央値19.4852015ms・最大19.820743ms、20ms超過・slot skipは0だった。checked sample ageは最大19.942440ms、最古入力から最終host write／返信は最大16.334253／19.211260msで全周期20ms内。開始間隔は94件中47件で20msを超え、最大20.331055ms、21ms超過0。startupと開始間隔を保持し、厳密な全周期20msやactive50Hz資格にはしない。

実行profile・880µs／window3・release spin500µsと固定R16 source25項目を照合し、owner／coordinator・CPU・GC・timer・IMU・Python switch intervalの復元、child join、source不変、起動同一性を確認した。最低checked voltageは35.15937805175781V。実原票1,688,024byteの6 SHAを独立照合した。直接の実施後回答は待ちで、異音・振動・滑り・沈み込み・姿勢維持や荷重移行を原返信から推定しない。基準900µsは変更せず、保存された単独実行から大きなC++速度向上の因果証明も作らない。

`tools/analyze_native_pair_latency.py`は、SHA固定・上限32MiBの保存report／recordsだけを読み、完了周期と失敗周期を別集計する。元の整数timestampからnative owner終了→Python join開始、最古入力→最終host write／返信、取得できた推論wall／thread CPU時間を算出する。開始timestampがない推論wallは未確認のまま残す。失敗周期を除去せず、識別情報・私有path・自由文errorを出力へ含めない。原票整合性auditや実出力admissionの代替ではない。独立したfile-onlyテスト21件はすべて成功した。この解析toolは固定済みR16実機kitには含めていない。

R15の保存原票では、完了83周期の両owner終了→join開始は中央値261.803µs、250.506〜291.467µsだった。失敗84周期目は288.332µsで、この区間だけが外れ値とは扱わない。失敗周期の推論thread CPUは2.726144ms、推論wall開始は未保存で未確認。保存timestampからの記述値であり、遅延原因の特定や成功への読み替えではない。

```sh
python3 -B -m unittest discover -s tools -p test_analyze_native_pair_latency.py

# 保存原票のみ。SHAは実際のreport／recordsの値を指定し、出力先は未使用のファイル。
python3 -B tools/analyze_native_pair_latency.py \
  --report ./saved-report.json --report-sha256 "$REPORT_SHA256" \
  --records ./saved-records.json --records-sha256 "$RECORDS_SHA256" \
  --output ./native-pair-latency-summary.json

PYTHONPATH=runtime:runtime/tests:tools python3 -B -m unittest \
  test_native_active_phase_pair test_native_phase_pair_runtime \
  test_native_phase_pair_profile test_feedback_reuse_cadence \
  test_offline_native_pair_comparison test_offline_feedback_reuse_comparison

# 既定ではPLAN。以下はローカル模擬通信のみ。保存先は未使用のディレクトリ。
python3 -B tools/offline_native_pair_comparison.py \
  --execute-offline --cycles 100 --output ./native-pair-new-run
```

各候補の結果は、基準900µs／window3（R15の明示880µs比較は別保存）と[成功設定](jetson-best20-settings.md)、新しい実行原票の電源・角度・IMU照合へ結び、ビルド・ソフトウェアテスト・無出力STOP代理5→501を個別に判定する。MacやJetsonの模擬比較、旧ソースの成功から、新C++経路の実出力承認を生成しない。

R17は通常observerのflatなfeedback snapshotに限定した所有copyの別候補で、observerソースとその試験だけを変更した。固定761ファイルのmanifest SHA256は`c3b6a04614f0ae1bea39a5bb27b1870ad93b276bc14814e532d123ca8849ee3e`。残りのソースをR16と照合し、active／diagnosticのC++ binaryは同一SHAの既存品をcopyした。R17で新しくビルドしたという主張はしない。独立observer試験47件は1.364秒で全件成功。Jetsonのobserver関連129件は1.248秒でOK（保存CPU parity artifactを未選択の3件skip）だった。880µs／window3、元の20ms期限、profileの数値・入力context・監視値・900µsの既定値と成功基準を維持した。

最初のファイル専用PLANは、旧observer SHAを保持したprofileとの不一致で正しく拒否した。その失敗を保持し、承認なしの新しいSTOP専用profileへobserverのsource bindingだけを結び直した。その他の全profile fieldと設定は一致し、新PLANが通った。guardや監視値を緩めたのではない。R17ではwatchdog依存ソースのobserverも変わるため、R16のwatchdog資格や物理確認から新実出力の承認を作らない。

Jetson上のファイル専用component比較は、R16 repeat501の保存snapshot501件について元のtimestamp、canonical hash、内容、mutable descendantの独立所有を照合した。6 roundのAB／BA比較の初回はcopy中央値107.667→55.058µs、copy＋digest230.0715→178.582µs。初回原票も保持する。元ソースSHAのimport前照合と設定変更前からのfinally復元、CPU4読戻しを追加したR2はcopy107.652→53.970µs、copy＋digest230.1845→176.9345µsだった。両component reportのSHAと各round中央値を独立検算し、CPU配置・GC・CPU／C7／EMC復元を確認した。装置・CAN・policy modelは開いておらず、whole observerや実周期の短縮、R16資格の移転を測った結果ではない。

R17の実CAN STOP primeは7.989051ms、全12返信完了でcadence epoch前に別保存した。元のSTOP代理5周期は1周期だけ完了し、startup whole iteration20.161225msと初回20ms超過1件を保持する。2周期目のraw電圧はfront34.9488410949707V／rear35.65062713623047Vで、既存35..42Vの下限によりproxy STOP出力前に`ABORTED`となった。2原票を保存し、失敗1周期と未試行3周期を成功へ数えない。steady完了周期は0であり、この試行をタイミングの改善・悪化や5／501周期資格へ使わない。

保存report／recordsの元のJSON bytesを再構成してSHAを照合し、64の重複なし完全要求・返信を独立確認した。内訳はType0 identity12、全zero Type4 STOP48、Type17 parameter read4で、有効化Type3・Type1・正ゲイン・学習目標は0。完了周期の出力12 STOPはmode0／fault0の完全返信と対応し、失敗周期のproxy出力は0だった。native owner／Python coordinatorのjoin・復元、CPU／C7／EMC・main／worker配置・GC・timer slack・Python switch interval・IMUの復元、source不変と前後起動同一性を確認した。電源の再確認を待ち、R17の厳密20ms・active資格は未成立。R16の実出力成功と物理回答待ち、および以前の全失敗記録を保持する。
