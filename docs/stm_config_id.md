# STM設定適用確認（session_id / config_id）

2026-10-17。
[robocon2026_A_STM f12724ad316fd694e2bae334d4d9dada09359242](https://github.com/TNCT-Mechatech/robocon2026_A_STM/commit/f12724ad316fd694e2bae334d4d9dada09359242)の
[通信仕様](https://github.com/TNCT-Mechatech/robocon2026_A_STM/blob/f12724ad316fd694e2bae334d4d9dada09359242/docs/initialization_protocol.md)に対応。
従来の移行記録 `stm_serial_migration.md` に記載されたパケットサイズ・起動処理は、この仕様で更新する。

## 対応範囲と形式

足回り `serial_motor_bridge`、注入・ロール `inject2026A`、速度調整 `param_checker`、
位置調整 `position_checker`、手動操作 `stm32_pos_control_test` が共通処理を使用する。

| ID | 内容 | payload |
|---|---|---:|
| 0〜3 | 初期化要求。末尾にsession_id/config_id | 52 bytes |
| 4〜7 / 8〜11 / 12〜15 | 速度 / 位置 / Duty | 4 / 8 / 4 bytes |
| 16〜19 | 周期通知。末尾に適用済みsession_id/config_id | 40 bytes |
| 20 | スイッチ状態 | 4 bytes |
| 21〜24 | モーター別の初期化ACK | 20 bytes |

全25フレームを登録。little-endian、自然アラインメント、COBSとchecksumは従来どおり。
sizeof/offsetofを検査する。旧44-byte要求・32-byte通知へ自動フォールバックしない。
接続する各STMは対応する新ファームウェアを使用する。

## 処理

1. 送信元の生成時に非0のsession_idを乱数で生成する。新しい設定や再接続ではconfig_idを増加させる。
   連番を使い切る際はsession_idを変え、wrapしない。
2. 全4モーターの適用確認まで速度/Dutyは0、位置制御は停止。初期化中の操作を後から実行しない。
3. ACKの要求ID・適用済みIDが両方期待値と一致し、結果がApplied/AlreadyAppliedなら確認完了。
   周期通知の適用済みID一致でも確認できるため、ACK欠落で再適用を繰り返さない。
4. 未確認モーターだけ、**同じID・同じ設定内容**で再送する。STMの重複判定により原点・PID状態を再リセットしない。
5. Invalid/Stale/IdConflict・未知の結果・不正な予約値は失敗。確認期限を超えた場合も停止して理由を表示する。
6. 運転中に適用済みIDが変わったら操作を停止し、同じ設定を再送する。
   足回り・機構は復旧後にもニュートラル操作を要求し、ラックの原点確定も取り消す。
   手動ツールは以前の指令を破棄する。調整中なら測定を中断し、途中の測定からゲインを保存しない。

成功応答はSTMの設定適用を示し、校正の妥当性や実際の回転を保証するものではない。
駆動指令にはsession_idがないため、1ポートにつき送信元は1つとする。

## 設定と診断

| ROSパラメーター | 既定値 | 意味 |
|---|---:|---|
| initialization_retry_ms | 100 | 未確認設定の再送間隔。正値 |
| initialization_timeout_ms | 2000 | 確認期限。再送間隔より大きい値 |

既存の通信・コマンド・フィードバック期限による停止も継続する。
足回り・機構は設定確認失敗時にポートを閉じ、既存の再接続/再起動方針に従う。
調整・手動ツールはエラー終了する。初期化待ち時間は調整の測定時間に含めない。

足回り `/diagnostics` の `configuration_ready=true` が確認完了。
`session_id`、各輪の `expected_config_id_N`、`applied_session_id_N`、`applied_config_id_N`、
`configuration_confirmed_N`、`initialization_attempts_N`、`initialization_result_N` で状態を確認できる。
送信回数は初回を含む。結果 `-1` はACK未受信であり、周期通知で確認済みなら正常。
機構のHardware/Serial診断にも設定確認状態を表示する。

## 試験

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select natto_system_monitor natto_robot_hardware inject2026A param_checker stm32_pos_control_test --symlink-install
source install/setup.bash
colcon test --packages-select natto_robot_hardware inject2026A param_checker stm32_pos_control_test natto_system_monitor --return-code-on-test-failure
colcon test-result --verbose
```

`test_configuration_state`は0番欠落、同一内容の再送、ACK照合、拒否、期限、周期通知、
STM再起動、旧形式拒否を検証する。
`configuration_pty`は実際の足回りノードをPTYへ接続し、確認前の停止、0番再送、ACK欠落、
拒否時の診断、再起動後のニュートラル必須を検証する。
機構・調整・複数STM操作の既存PTY試験も新形式へ更新した。実機シリアルは使用しない。

### 今回の検証結果

- 対象5パッケージのビルド成功。
- 動作試験134件と従来通信のPTY互換試験5件、合計139件成功。
- 上記コミットから取得したSTMの実際の `InitializationState` とROSの実際のSerialBridgeを組み合わせた
  ホスト試験も成功。ABI一致、0番要求欠落、ACK欠落、重複適用防止、STM再起動、設定更新を確認した。
- 全チェックではinject2026A/param_checkerの既存flake8・uncrustifyと、inject2026Aの
  大文字を含むパッケージ名のxmllintが失敗する。作業開始時の保存コピーでも同じチェックが失敗することを確認した。
- 実機への接続・走行・STMへの書き込みは行っていない。

既存lintと分けて動作試験を再実行する場合:

```bash
colcon test --packages-select natto_robot_hardware inject2026A param_checker stm32_pos_control_test --return-code-on-test-failure --ctest-args -E '^(cppcheck|flake8|lint_cmake|pep257|uncrustify|xmllint)$'
```
