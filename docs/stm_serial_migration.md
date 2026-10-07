# STMシリアル通信移行報告

2026-10-06 / ROS 2 Jazzy。STM側のファイル・Git履歴には変更を加えていない。
ROS側の実装とPTY上の動作は検証済み。実機での完全動作には下記のSTM依存の問題と校正確認が残る。

## 1. 調査した仕様と不一致

基準は [robocon2026_A_STM の c9cda43b5cad56aee9a7029c49e54acefd5c34ec](https://github.com/TNCT-Mechatech/robocon2026_A_STM/tree/c9cda43b5cad56aee9a7029c49e54acefd5c34ec)。
`Core/Inc/serial_packets.hpp`、`Core/Src/main.cpp`、`Core/Inc/Encoder_HAL.hpp`、
`Core/Src/Encoder_HAL.cpp`、`.gitmodules` と、そのsubmoduleのSerialBridge実装を読み取った。
ROS側の3パッケージのpacket/main/config/launch/CMake/package.xml、PTYテスト、watchdog、
restart、diagnostics、wheel odometryを調査した。
リポジトリ全体でMotorCommand/Feedback、そのPacket、旧frame IDパラメータ、
SerialBridgeのadd_frame/write/updateを検索した。旧機体通信への依存は今回の3ノードとPTYテストに存在した。
ライブラリの汎用exampleにあるID0/1は機体通信の定義ではない。

| 項目 | 旧ROS | STMに合わせたROS |
|---|---|---|
| 指令 | 4モータ一括80byte、ID0 | モータ別のInitialization/Velocity/Position/Duty |
| PID設定 | 指令にgainとboolのpid_stateを同梱 | 44byte Initialization、uint8_tのpid_state |
| feedback | 4モータ一括80byte、ID1 | ID16〜19、各32byte |
| encoder_count | float | int32_t |
| 累積位置 | wheel_rpsをdtで積分 | int64_t encoder_raw_countをCPRでradへ変換 |
| open-loop | 速度フィールドへnormalized値 | ID12〜15へsigned duty |
| 受信 | callbackあたり1frame | -1までdrain、最大256frame |
| スイッチ | 未登録 | ID20登録・受信 |
| 極性 | ROS受信後にもencoder_polaritiesを乗算 | STM set_directionに集約 |
| 接続 | zero commandのみ | 全ID登録→4初期化→モード別zero |

USART2の対向設定は230400 baud / 8N1 / flow controlなし。LinuxHardwareSerialの既存設定もこれと一致する。

### SerialBridge互換性とSTM側に残る問題

STMのsubmoduleは指定の [8bbe616dcd614c78f89a5d151a6a884700f4c5d4](https://github.com/TNCT-Mechatech/SerialBridge/tree/8bbe616dcd614c78f89a5d151a6a884700f4c5d4)。
ROS内のSerialBridge.cpp、CobsSerial.cpp、Message.cpp、COBSの形式は維持した。
Message.hppの差分はROS側の受信更新フラグの初期化であり、wire形式に影響しない。

**そのSTM参照コミットのSerialBridge.hppはSTRUCT_MAX_NUM=10である。**
STM main.cppは21種類を登録するが、add_frameの戻り値を検査していない。
そのままでは全21frameの登録に成功しない。ROS側のみ上限32とnull終端用の追加slotに変更し、
全登録結果を検査するようにした。COBS・ID+payload+checksum・パケットのABIは変えていない。
STM参照版の_str配列にも未初期化の問題があり、ROS側に既存のゼロ初期化を維持している。
**ROSの変更だけではSTM側の登録失敗は解消できない。実機に書き込まれたSTMビルドで
全21frameが登録できることを別途確認する必要がある。STM側の変更は本作業では行っていない。**

## 2–3. 変更ファイルと目的

パスはリポジトリルートからの相対パス。

| ファイル | 目的 |
|---|---|
| `SerialBridge/src/SerialBridge.hpp` | 登録上限32、lookupのnull終端slot確保 |
| `SerialBridge/src/LinuxHardwareSerial.hpp` | injectの既存diagnostics用read-only FD accessorを共有transportへ追加 |
| `src/inject2026A/SerialBridge/src/SerialBridge.hpp` | 保持しているコピーの登録容量も一致させる。ノードのビルドはroot側へ統一 |
| `src/natto_robot_hardware/include/natto_robot_hardware/serial_protocol.hpp` | 全struct・Message typedef・固定ID・sizeof/offsetof・little-endian検査の共通定義 |
| `src/natto_robot_hardware/include/natto_robot_hardware/serial_frames.hpp` | 個別packetの保持、全ID登録、短いwriteの検査、受信drain |
| `src/natto_robot_hardware/include/natto_robot_hardware/serial_initialization.hpp` | 3ノード共通の初期化パラメータ宣言・検査・設定 |
| `src/natto_robot_hardware/include/natto_robot_hardware/serial_packets.hpp` | 共通headerへのincludeに置換 |
| `src/param_checker/include/param_checker/serial_packets.hpp` | 同上、旧重複structを除去 |
| `src/inject2026A/include/inject2026A/serial_packets.hpp` | 同上、旧重複structを除去 |
| `src/natto_robot_hardware/src/serial_motor_bridge.cpp` | 初期化・モード別指令・個別feedback・watchdog適応・raw position・switch topic |
| `src/natto_robot_hardware/config/serial_motor_bridge.yaml` | 旧IDパラメータ除去、初期化設定・根拠・要校正の値を明示 |
| `src/natto_robot_hardware/CMakeLists.txt` | ABI/SerialBridgeテスト追加 |
| `src/natto_robot_hardware/package.xml` | gtest依存追加 |
| `src/natto_robot_hardware/test/test_serial_protocol.cpp` | 6種ABI、21ID往復、混在エラー・連続受信・drain上限を検証 |
| `src/natto_robot_hardware/README.md` | 新仕様・設定・極性・timeout・スイッチの運用説明 |
| `src/inject2026A/include/inject2026A/main.hpp` | 共通packet storageとswitch publisher |
| `src/inject2026A/src/main.cpp` | duty移行・初期化・個別受信・drainと既存監視の統合 |
| `src/inject2026A/config/serial_ports.yaml` | 機構ごとの初期化設定、send-only監視の選択を明示 |
| `src/inject2026A/CMakeLists.txt` | 共通header依存とroot SerialBridgeへの統一、既存の空白違反修正 |
| `src/inject2026A/package.xml` | natto_robot_hardware依存と既存XML要素順の修正 |
| `src/inject2026A/README.md` | 新通信・switch topic・監視方法の説明 |
| `src/param_checker/src/main.cpp` | PID設定と目標の分離、モード別停止、個別feedbackのVRFT/CSV処理 |
| `src/param_checker/config/serial_protocol.yaml` | VRFT用初期化・エンコーダ設定の明示 |
| `src/param_checker/launch/param_checker.launch.xml` | serial_configの読み込みとoverride用launch引数 |
| `src/param_checker/CMakeLists.txt` | 新configをinstall |
| `src/param_checker/README.md` | tuning・モード・エンコーダ設定の運用説明 |
| `src/monitoring/natto_system_monitor/test/test_serial_pty.py` | 旧80byteのmockを廃止、新ABIと12件のPTY統合検証 |
| `docs/stm_serial_migration.md` | 本報告書 |

navigation、mecanum演算、wheel odometry、debug_msgsの定義、PID保存先、
SerialWatchdog/SerialRestartPolicyの実装自体は変更していない。

## 4. 固定frame IDとABI

各範囲内の順番はmotor0〜3。共通headerに定数を定義し、すべて登録する。

| ID | packet / payloadサイズ | 主な単位・用途 |
|---|---|---|
| 0〜3 | InitializationPacket / 44byte | PID gain・エンコーダ設定・mode |
| 4〜7 | MotorCommandVelPacket / 4byte | target_motor_rps、rps |
| 8〜11 | MotorCommandPosPacket / 8byte | target_pos、int64 raw count。現在のノードは送信しない |
| 12〜15 | MotorCommandDutyPacket / 4byte | signed duty、[-1,+1] |
| 16〜19 | MotorFeedbackPacket / 32byte | rps・output・int32 count・int64 raw count・duty・dir |
| 20 | SwitchFeedbackPacket / 4byte | bool state[4] |

自然alignmentを保持し、int64のoffset16とその他の主要offsetもstatic_assertで検査する。
pack指定によるABI変更は行わない。MessageのIDとchecksum、COBS境界はpayloadサイズに含まない。

## 5. Closed-loop送信

serial open → ID0〜20登録 → ID0〜3 Initialization(pid_state=1, YAML kp/ki、kd=0)
→ ID4〜7 Velocity=0 → feedback回復とneutral操作を待つ → 通常指令。
JointStateのname対応または従来の配列順でvelocityを読み、rad/sを2πで除してrpsへ変換、
max_rpsでclamp、wheel_polaritiesを適用して個別に送信する。input_unit=rpsも維持。
停止はVelocity=0であり、Duty/Positionを混送しない。再接続でも同じ初期化手順とneutral待ちになる。

## 6. Open-loop送信

serial open → 全ID登録 → Initialization(pid_state=0) → ID12〜15 Duty=0 → 通常指令。
従来のopen_loop_full_scale_rps、4輪比率を保つ正規化、output_limit、slew_rateを維持し、
その出力をsigned dutyとして送信する。stop時はDuty=0を直ちに設定し、Velocityを送らない。
通常時は指令入力に応じて送信する。指令途絶・復帰待ちでは10ms周期でzeroを送信する。

## 7. Initializationへ設定する内容

| フィールド | 設定元・既定の方針 |
|---|---|
| vel_kp/ki/kd | 車輪は既存PID gain YAMLのkp/kiと固定kd=0、速度用param_checkerは従来どおりinitial_kp/ki/kd。位置用は保存済み速度kp/kiと固定kd=0。direct-duty機構は0 |
| pos_kp/ki/kd | 各4要素のROSパラメータ、既定0 |
| set_direction | encoder_polarities < 0 |
| stop_mode | 既存/追加ROS boolパラメータ、既定false |
| pid_state | モードから導出。closed velocity=1、open/duty=0。位置用は速度ゼロで初期化し、verifyの位置指令でSTMが位置モード2へ切り替える |
| max_motor_rps | 4要素ROSパラメータ。省略時は既存max_rps/max_wheel_rpsを使用 |
| encoder_ppr | 4要素ROSパラメータ。STM既定の2048、uint16範囲と正値を検査 |
| gear_rate | 4要素ROSパラメータ。STM既定の[71,245/6,245/6,245/6]、有限正値を検査 |
| reset_encoder_origin | 4要素bool、既定false。ROS再接続で累積位置を消さない |
| reset_encoder_origin_with_switch | 4要素bool、既定false。homingは明示的な有効化が必要 |
| encoder_origin_switch_state | 4要素bool、既定true。有効化時は実スイッチ極性を確認 |

max_motor_rpsはSTMでは位置制御の最大速度設定であり、速度指令のclampはROS max_rpsが行う。
4要素の数・極性±1・PIDの有限値等も検査する。YAMLのgear/pprは実機校正済みという意味ではない。
旧車輪コードのkd=0固定は実機運用上の意図した仕様であるとユーザーから確認された。
新Initializationでもvel_kdは常に0とし、YAMLの保存値は変更しない。
param_checkerのinitial_kdによる測定設定とVRFTのkd算出・保存は従来どおり維持する。

## 8. Encoder polarity

符号補正のsource of truthはSTM。encoder_polarities=-1をset_direction=trueへ変換する。
STM Encoder_HAL.cppのposition32 += (direction_ ? -1 : 1) * dpで補正されるため、
ROSの速度・位置にはencoder_polaritiesを再乗算しない。wheel_polaritiesは指令をSTMの
モーター座標へ変換し、受信した速度・位置をROSの車輪座標へ戻すために適用する。
車輪のwheel_polaritiesは[-1,1,-1,1]を保持し、encoder_polaritiesは[-1,-1,-1,-1]へ修正した。
FR/RRの逆回転・暴走の報告を受け、STM内のPID用エンコーダ極性とROS座標変換を分離した。
param_checkerの同梱設定は現在[-1,-1,-1,-1]で、
全輪gear_rate=245/6となっている。実機の設定に合わせて各ノードのYAMLを確認する。
inject/rollの旧packetはゼロ初期化で方向設定falseだったため既定[1,1,1,1]を維持する。
車輪の一律set_direction=trueを復元し、直接起動のコード既定も車輪YAMLに合わせた。
実機の符号は配線・旧STM動作にも依存するため、PTYで物理極性の一致を証明することはできない。

## 9. Feedbackのdrainと監視

SerialBridge/CobsSerialの実装を確認し、updateの各呼び出しが最大1frameを消費することを根拠に、
0/-2/-3なら続行し、-1で終了する。callback当たり256frameに制限し、連続入力でもcallbackが戻る。
不正・未登録frameも消費して次の正常frameへ進む。末尾の-1でバッチの正常受信を上書きしない。
混在エラーはバッチ結果とカウンタに残す。最新値とモータ別更新flagを保持し、drain後にROSへ反映する。

車輪のwatchdog受信時刻は、前回の一式から各モータの新feedbackがすべて揃ったときのみ更新する。
1モータだけ、またはswitchだけの返信は他モータの断線を隠さない。
feedback timeout、reconnect、steady_clock、neutral必須、auto restart、diagnosticsは維持した。
車輪diagnosticsにinvalid_packets/unregistered_frames/drain_budget_hits/STM timeoutを追加した。
inject/rollの既存のoptional update-result監視とsend-only運用も維持した。
STMの5frame/10msはdrainで対応し、PTYで100Hz burstと部分返信のタイムアウトを確認した。

ID20の公開方法はstd_msgs/UInt8MultiArray（4要素0/1、motor順）：
`/switch_feedback`、`/inject/switch_feedback`、`/roll/switch_feedback`。
param_checkerもID20を登録・正常受信する。

## 10. JointStateとdebug

position_rad = encoder_raw_count / (encoder_ppr * 4 * gear_rate) * 2π。
velocity_radps = measured_wheel_rps * 2π。
最初の全4モータのfeedbackが揃ってからJointStateを公開し、その後は各モータの最新値を使う。
名前・topic・rad/s単位を維持し、wheel_odometryのmecanum処理は従来どおりvelocityを読むため互換。
STM再起動や明示homingでraw countがresetされれば位置も変わる。ROS再接続では既定で原点を消さない。

feedback0〜3は実際に更新されたモータについてcurrent_rps/output/encoder_count/duty/dir/target_rpsを公開する。
int32 encoder_countは既存msgのfloat32へ変換し、msg定義を変更しない。float32の精度限界は既存interfaceに残る。
raw countを既存debug messageへ追加しない。open-loopのtarget_rpsは要求速度、実際のdutyはdiagnosticsで見られる。

## 11. Inject/roll

両方を移行した。旧コードではzero-initialized packetのpid_stateがfalseで、PID gainも設定しておらず、
0.4/0.2/0.3等は直接出力として使われていたため、Velocityではなくsigned dutyが対応する。
従来のボタン/軸対応、速度feedback topic、Joy異常時停止、command timeout、再接続とrestartを維持。
同一プロトコルの基板を接続する前提の実装であり、実機のinject/roll基板にも対象STM版が書かれているかは要確認。
既定monitor_serial_updates=falseは従来運用を維持。返信監視を有効にする場合はtrueを指定する。

## 12. Param_checker

PID gain設定はInitialization ID0〜3へ分離し、目標だけをVelocity ID4〜7へ送信する。
従来既定のpid=falseでのVRFT測定はDuty ID12〜15を用い、pid=trueなら速度目標となる。
Joyのたびに同じInitializationを再送せず、原点やPID状態への不要な影響を避ける。
同一モータへ異なるcommand種類を混送しない。停止は現在モードのzeroで、gainを消さない。
Joyの300ms deadlineを追加し、STMの通信timeoutも独立して働く。
個別feedbackをdrain後に処理し、更新のあったモータだけをVRFTへ記録する。
VRFT演算、診断、debug topics、CSV形式、PID gain YAMLの保存・既存保存パスを維持した。
既存CSVのtarget_rps列はpid=falseでは従来どおり直接出力値なので、解析時にmodeを考慮する。

## 13. Build/test

ROS 2 Jazzyで実行した。初回workspace内のsymlink-installは既存debug_msgs生成ディレクトリと衝突したため、
既存データを削除せず、/tmpの独立build/install先で検証した。
依存込み6パッケージをbuildし、最後に変更3パッケージを再build：成功。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/ros-stm-colcon-log build --symlink-install \
  --build-base /tmp/ros-stm-build --install-base /tmp/ros-stm-install \
  --packages-up-to natto_robot_hardware inject2026A param_checker
source /tmp/ros-stm-install/setup.bash
export ROS_LOG_DIR=/tmp/ros-stm-test-logs
colcon --log-base /tmp/ros-stm-colcon-log test \
  --build-base /tmp/ros-stm-build --install-base /tmp/ros-stm-install \
  --packages-select natto_robot_hardware natto_system_monitor inject2026A param_checker
colcon test-result --test-result-base /tmp/ros-stm-build --verbose
/usr/bin/python3 src/monitoring/natto_system_monitor/test/test_serial_pty.py -v
```

| 検証 | 結果 |
|---|---|
| Native ABI・21ID往復・混在エラー/drain・256frame上限 | C++ 4ケース成功 |
| SerialWatchdog/SerialRestartPolicy/ObservedSerial/health/monitor integration | 既存テストすべて成功 |
| 車輪closed/open、初期化、極性・raw位置・switch | PTY成功 |
| 5frame/10ms、部分返信断、再接続・抜き差し・FD leak・neutral復帰 | PTY成功 |
| Joy異常・command/feedback timeout・send-only・launch respawn | PTY成功 |
| Param_checker velocity/duty、gain分離、VRFT、CSV、gain保存 | PTY成功 |
| PTY suite全体 | 12ケース、45.592秒、失敗0 |
| cppcheck / CMake lint | 両パッケージ成功 |
| param_checker XML / 両launch XML | 成功 |
| uncrustify / inject package.xml | 下記の既存問題で失敗 |
| git diff --check | 成功 |

colcon test-resultの全体集計は123 tests / 0 errors / 40 failures / 38 skipped。
この集計はCTestの失敗と個別lintファイルの失敗を含む。失敗targetはuncrustify 2件とinjectのxmllint。
変更前HEADを/tmpへ展開して同じlintを実行し、param_checkerの9ファイル、injectの29ファイルの
既存書式違反とinject2026Aの大文字入りパッケージ名によるschema違反を確認した。
今回wrapper化した2つのpacket headerの書式違反は解消され、残る書式違反は同じ既存ファイル群。
既存XML要素順とCMake空白違反は修正した。命名規則を満たすためのパッケージ改名や
vendor/VRFTを含む一括整形は、launch互換性と変更範囲を保つため実施していない。
**全lint成功とは報告しない。** コンパイル警告には既存SerialBridgeのVLAや非virtual destructor等も残る。

詳細ログは `/tmp/ros-serial-build-final.log`、`/tmp/ros-serial-colcon-test.log`、
`/tmp/ros-serial-pty.log`、`/tmp/ros-serial-lint-final.log`、`/tmp/ros-serial-test-result.log`。
元workspaceのinstallを新バイナリへ上書きしていない。今回の検証バイナリは上記/tmp overlayをsourceして使用する。

## 14. 残る実機確認

1. **STMで全21frameの登録が成功するか。参照SerialBridgeの上限10ではそのまま動作しない。**
2. メカナム・inject・rollの各基板の実際のSTM版、baud、motor index・配線の対応。
3. 各機構のencoder PPRとgear_rate。STM既定値は各機体の実測校正値ではない。
4. 車輪の指令wheel_polarities、STMでのencoder補正、JointStateの正方向の一致。
   低出力で1モータずつ確認し、ROS側で追加の符号補正を入れない。
5. 設定YAMLのkp/ki、固定kd=0、stop_mode、最大速度が実機で意図した応答か。
6. limit switchのactive levelと入力順、明示的なhoming有効化時の位置reset動作。
7. 実UARTの連続5frame/10ms、エラーcounter、USB抜き差しとfeedback timeout/restart。
8. PC停止や通信途絶から300ms程度でSTMが停止するか。STM timeoutはpid=1/stop_mode=trueへ
   切り替える実装なので、Dutyで復帰するときの停止挙動とbrake解除についても確認する。
9. STM再起動/原点reset後のraw位置変化とodometry・可視化。Joy/commandの送信間隔は
   min(ROS timeout_ms, STM 300ms)より短くする。
10. inject/rollで新STMのfeedback監視を有効化する場合はmonitor_serial_updates=trueと
    serial_update_failure_limitを選び、実機の返信周期で復帰/停止を確認する。

以上の実機依存値とSTM側登録問題は未解決のまま推測で完了扱いにしていない。

## 15. param_checkerの位置制御追加

[stm32_board_pid_pos_vel_control_set の参照版](https://github.com/TNCT-Mechatech/stm32_board_pid_pos_vel_control_set/tree/78edb8b795f9315d4492b7b9716ea78b12200a4f)
の二段階VRFTに合わせ、位置用 `position_checker` を追加した。
速度用は `param_checker_velocity.launch.xml`、位置用は `param_checker_position.launch.xml` を使用する。
既存の `param_checker.launch.xml` は速度用として維持した。

- tuneは対象1台へ速度指令を送り、保存済み速度PIを用いて生カウントの応答から位置Pを同定する。
- verifyは対象1台へint64位置指令を送り、位置P＋速度PIのカスケードを確認する。
- 速度Kd・位置Ki/Kdは0固定。対象以外はゼロduty、Joy/対象feedback期限で停止する。
- 既存の速度ゲインと他wheelを保持して `wheelN.position` を追記し、無動作・不十分な適合度では上書きしない。
- 新規ノード、位置VRFTヘッダ、C++テスト、2つのlaunchを追加し、CMake/package.xml/README/USAGE/PTYテストを更新した。

起動手順、保存形式、オプション一覧は [param_checkerの使い方](../src/param_checker/USAGE.md#12-位置制御のゲイン測定と動作確認) に記載した。
位置VRFTの単体テストは3件、速度・位置のparam_checker PTYテストは11件で検証する。
実機の位置応答・校正値は未検証で、STM側登録上限についての既存の確認事項も引き続き適用される。
