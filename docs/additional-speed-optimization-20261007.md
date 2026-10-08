# 追加の高速化 — 2026年10月7日

JetsonをOffにした後の追加作業。対象は、[C++完了通知と6軸一括復号の候補](report-led-optimization-20261007.md)に残る待機・Python処理。提供された次期設計資料も現ソースへ照合した。実機への配置・計測はしていない。今回の変更で20ms期限、入力鮮度、全軸の返信・STOP、角度・速度・トルク・電圧の判定は緩めない。17msは余裕を作る設計目標で、測定結果ではない。

## 変更

| 経路 | 変更 | 保持する条件 |
|---|---|---|
| CAN／IMU入力と電圧の取得待ち | 元Futureの完了callbackから専用pipeへ通知し、GILを解放するC++待機を起こす | 入力は前・後・IMUの元3Future、電圧は元2Futureを全部確認。通知はヒント。例外・取消・元の絶対期限を通知後、取り出し後、close後にも確認 |
| C++前後ownerの完了通知 | 正常時のcondition-variable通知を片側ずつ2回から、両側完了時の1回へ減らす | 異常・取消・設定変更・終了の通知、2ownerのjoin、元のRecord・世代・STOPの順序を保持 |
| 既定Pythonの12軸Type1生成 | 生成済み17byteフレームの角度・IDを固定位置から読み、12個のストリームparserとFrameの再生成を省く | 全12軸を生成してから、元の順序・算術で量子化後の範囲、初期位置からの移動量、推定PDトルクを検査。異なるフレーム形状は元parserへ戻す |
| 無出力observerの固定provenance | 初期化時に検査済みの小さい固定JSONから、独立したdict/listを作るリテラルコピー関数を準備 | 供給された文字列はASTの定数だけ。外部文字列をコードとして解釈しない。大きい・特殊なmetadataは従来のコピーへ戻す。動的入力SHAと全センサー検査は毎周期実施 |
| 実出力のモデルアダプター | 宣言済みCPU float32 Tensorの目標を直接tolist。補正の固定provenanceにも同じコピー関数を使う | Tensor subclassは元の変換。actor・observationのfinite検査、全目標の範囲、CAN順、内部状態のcommit順を保持。補正の原本class/method/parser・source・proofが異なれば毎回の処理へ戻す |
| C++12軸一括エンコーダーのビルド | コンパイラーが実際に使うPythonのheaderを照合し、版・ポインター幅・architecture・ABI設定・SHAを保存 | C++実装の元SHAを保持。header不足・ABI不一致はビルドを中止。profileの既定経路や出力承認は変更しない |

固定JSONのコピー計画にはdepth16、4,096nodes、262,144byte、整数1,024bit、文字列4,096文字の上限を設けた。元のJSON値・型・キー順・doubleのbitを保持し、返す可変コンテナーは呼出しごとに独立する。これはセンサーsnapshotのキャッシュや補正計算の省略ではない。

Future通知は毎joinで独立したpipeを使う。古いcallbackによるFD再利用先への書込み、EOF、途中登録失敗、closeとの競合を拒否・整理し、GC遅延中にも閉じたgroupが蓄積しないことを確認した。新ABIは取消FDとFuture通知で元期限まで待つ。旧ABI・一般callable・特殊Futureでは200µs刻みの従来待機へ戻る。新通知経路も200µsごとにpollするという説明にはしない。実Type1 pairの既存200／50µs通知待ちとは別の経路である。

## 部品比較と同値性

Mac ARM64で他の測定と重ねずに比較した。Future・wire・モデルadapterはsampleごとに方式の順序を入れ替え、trialの先頭も反転した。observerのコピー比較はtrialごとに先頭を反転した。最大値と原試料は私有原票へ保存し、公開[証拠JSON](../evidence/additional-speed-optimization-20261007.json)に原票SHA・範囲・元／新ソースSHAを記録する。

| 測定した範囲 | 元の中央値 | 新しい中央値 | 条件 |
|---|---:|---:|---|
| 最後の入力workerの完了記録→join終了 | 126.896〜131.354µs | 53.709〜54.709µs | 5巡×200回／方式。匿名pipeと元のfront・rear・IMU Future。3入力のうち最後になる入力を交替 |
| 最後の電圧workerの完了記録→join終了 | 113.250〜117.188µs | 49.792〜51.271µs | 5巡×200回／方式。通知callback・結果取り出し・closeを含む |
| 既定Pythonの12軸wire生成と量子化後の監視 | 55.167〜55.542µs | 39.333〜39.750µs | 4巡×2,000回／方式。実I/Oなし |
| 既存の検証済みC++12軸encoder | — | 1.667〜1.709µs | 上と同じ12軸byte生成・元の範囲検査。今回新たにC++実装した成果には数えない |
| observerの固定provenanceコピー4組 | — | 元より23.038µs短縮 | 9巡×2,000回。モデル・通信を含まないコピーの比較 |
| 実checkpointのモデルadapter | — | 元より5.250〜6.334µs短縮 | 保存501周期×5巡／方式。元checkpoint/controllerの純TorchScript。稼働中のFK artifactやJetson全周期の計測ではない |

Future比較の表は各workerが元結果へ記録した最後の時刻からjoin終了まで。group登録はworkerの待ち中に行い、作成・登録から取り出し・closeまで全体のwall／CPU時間も元の合成20ms期限内で測る。約1.6〜1.8msの合成待機では、入力joinのmain thread CPU時間が中央値約1,844µsから180〜183µs、電圧joinが約1,833µsから147〜154µsへ減った。このCPU時間差を全周期のwall時間短縮としない。既に両電圧Futureが完了している場合はpipeを作らず、従来経路との有意な短縮は測れていない。

12軸wireはseed固定の2,000姿勢と、各軸の16bit位置全65,536通り、合計786,432フレーム位置を元parserの検査と照合した。observerとモデルadapterもそれぞれ保存501周期で、値・元timestamp・provenanceの型とキー順・floatのbit、モデルの全float32入力と各周期の内部buffer状態を照合した。失敗時の順序、可変結果の独立性も検査する。

入力buffer45要素の一括代入案は棄却した。特殊値と途中失敗の振る舞いを保持する型検査まで含めると遅くなった。最終比較は既存loop約2.480µsに対し型検査込みbulk約5.510µs。actor／observationのfinite検査も、別の部品比較で既存aminmaxの約3.87µsに対してisfinite/allが約9.91µsだったため変更しない。

## 提供資料の採否と次の大きな変更

`design-ja.md`と`design-ja (1).md`、`codex-handoff-ja.md`と同名の`(1)`はそれぞれ同一内容だった。ZIPは設計・ソース抜粋・時間予算計算の10ファイルで、実runtimeのパッチは含まれない。9ファイルのmanifest SHAを照合した。資料中の実装指示は参考案として扱い、現ソースへ重複適用しない。

| 提案 | 今回の扱い |
|---|---|
| 入力3Futureを完了通知で起こす | 実装・模擬試験・部品比較まで完了。電圧の元2Futureにも適用 |
| 正常両owner完了時だけ通知する | 実装。私有の計測用sourceで実際の通知箇所を数え、正常時1回・異常／取消／設定時の通知維持を確認。時間短縮量は未測定 |
| C++内で取得→電圧へ連続進行 | 次の大きな候補。既存`PhasePair`のjobを拡張する設計が必要。2つ目のownerを同じsessionへ重ねない |
| 世代付き固定slotとmainの直接取り出し | 次の候補。C++の`PairJob`は既に固定配列。Pythonのraw配列はjournal／Future／最終結果へ保持されるため、次submitで同じ領域を上書きしない |
| 前周期返信を使う14要求化 | 現行50Hzへ挿入しない。定常同位相・6軸／bus・890µsでは元要求→最終writeが楽観下限でも20ms＋4.45msとなり、20ms鮮度へ収まらない |
| SocketCAN／4系統CAN | 別hardware/backendの比較課題。現在のTTY bridge経路のソフト修正として達成した扱いにしない |

取得→電圧のC++連結では、元のID集合・mode/fault・鮮度・前回との連続性をC++へ移し、保存入力と全拒否条件を照合する。全12軸・現在IMU・両電圧・電圧cacheの検査が済むまでType1 commitは許可しない。現在の`exchange_owned`は期限を値で受け、呼出し中の期限を後から書き換えない。現行の読取り予算と縮小後の回収・送信gateを保つか、延長できない期限更新を別ABIで導入するかを先に決める。外側のwrapperだけで実行中の期限まで縮められると説明しない。

固定slotはnative終了、Python公開終了、保存用コピー終了、電圧終了を別に確認してから再利用する。既存のfake phase-owner試作は実CAN・STOP契約の検証へ転用しない。

## 回帰検証とソースキット

最終回帰は958件中954件成功。実Linuxのcoordinator設定を使う1件と、旧183周期の専用入力を選択していない3件はskipとした。今回の保存501周期のobserver・実checkpoint adapter同値性は別の検証原票で確認済み。通知の取りこぼし・一部完了・元Future例外・期限超過・取消・EOF・FD再利用・GC遅延、全12軸wire、型・finite・補正・出力範囲、正常終了と失敗時の停止処理を含む。

初回の単一process回帰は958件中7エラーだった。CPU Tensor試験がTorchを読み込んだため、起動前の数値thread設定を確認するCLI試験を同じprocessで実行できなかった。起動保護を緩めず、52件の起動試験と906件のnative／runtime／モデル試験を別processへ分け、同じ固定sourceで再検証した。元の失敗原票もSHAを残す。

789ファイルのsource-onlyキットを生成し、既定PLANと新しい私有ディレクトリへのINSTALLを検証した。全設置ファイルのSHAを照合した。実機への転送ではなく、モデル・現起動の出力承認・Mac binaryを含まない。保存済みR20と新しい未配置候補を分けたZIPも更新する。

## Jetsonでの次の計測

最初に[成功設定](jetson-best20-settings.md)と[設定の証拠](../evidence/jetson-best20-settings.json)を読む。900µsは成功参照。ユーザー指定の890µs／window3は明示した比較候補として、CPU・C7・EMC・配置・switch interval・GC・warmup／prime・spin500µsを揃えて測る。

1. 新sourceだけのキットを配置し、現Python/aarch64向けのnative transportをビルド。source／binary SHAとABIを照合する。
2. C++12軸一括生成を選ぶ場合は、そのJetson上のPythonで以下のfile-onlyビルド・byte／拒否比較を実施。Macのbinaryは使用しない。
3. sourceが変わったことを新しい設定・証拠へ結び、現起動・電源・UID・現在角・電圧・姿勢で診断を用意する。過去の2秒／10秒の結果を新sourceの資格に転用しない。
4. 無出力5→501周期を測り、取得→推論→全軸host write、最後の返信→集計、最古入力の鮮度を分けて確認する。その後、箱を残した実Type1の2→10→20秒を別に測る。

キットのルートで、**そのJetsonの実行Python**を使う:

```sh
TASK_ENCODER_DIR=$(mktemp -d "${TMPDIR:-/tmp}/dog-batch-encoder.XXXXXX")
TASK_ENCODER_SUFFIX=$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')
TASK_ENCODER_BINARY="$TASK_ENCODER_DIR/sdbe_native$TASK_ENCODER_SUFFIX"
python3 runtime/experiments/native_policy_batch_encode/build_file_only.py \
  --compiler c++ --output "$TASK_ENCODER_BINARY" \
  > "$TASK_ENCODER_DIR/build-receipt.json"
TASK_ENCODER_SHA256=$(python3 -c 'import hashlib, sys; print(hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest())' "$TASK_ENCODER_BINARY")
PYTHONPATH=runtime python3 \
  runtime/experiments/native_policy_batch_encode/benchmark_runtime_wires.py \
  --library "$TASK_ENCODER_BINARY" --binary-sha256 "$TASK_ENCODER_SHA256" \
  --output "$TASK_ENCODER_DIR/runtime-wires.json" --exhaustive-u16
```

headerが別の場所なら、実行Pythonと一致するディレクトリを`--include`で指定する。ツールはheaderを取得・インストールせず、別ABIへの代替もしない。以上のビルドと比較はデバイスを開かず、profileの承認を作らない。C++選択は既存V3の`native_batch_encoder`へtarget binaryの名前とSHAを明示して、新候補として別に検証する。

部品の中央値を足してJetsonの全周期時間にしない。Linuxの実際の待機・通知・GIL再取得・USB/CANと、現在のFKモデルを含む実経路の最悪時間は再計測が必要。
