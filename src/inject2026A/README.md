# inject2026A

Ros2 Humble

`main` は1プロセス内で2つのROS 2ノードを実行し、注入機構用とロール機構用の
2台のマイコンへ、それぞれ別のUSBシリアルポートで接続します。

- Joy入力: `inject/joy`（両ノード共通）
- 注入機構フィードバック: `inject/fb_speed_topic`
- ロール機構フィードバック: `roll/fb_speed_topic`

接続先は各ノードの `serial_path` パラメータで指定します。
注入用はST-Link識別番号 `0671FF505071655067254019` の `/dev/serial/by-id/` に固定済みです。
ロール用はST-Link識別番号 `0671FF555185754867214745` の `/dev/serial/by-id/` に固定済みです。

設定ファイルを使って起動する場合:

```bash
source install/setup.bash
ros2 launch inject2026A inject.launch.xml
```

`config/serial_ports.yaml` のノード名ごとに接続先を設定します。
別の設定ファイルを使う場合は `serial_config:=/absolute/path/serial_ports.yaml` を指定します。

従来の `ros2 run inject2026A main` も使えます。この場合はコードの既定値を使います。
YAMLを渡す場合は次のように起動します。

```bash
ros2 run inject2026A main --ros-args --params-file src/inject2026A/config/serial_ports.yaml
```

この実行ファイルは注入とロールの両方に接続します。ポートを開けない場合も
プロセスを終了せず、その機構だけ再接続を試みます。接続成功・失敗のログに対象パスを表示します。
USBポートの変更では設定変更は不要ですが、基板を交換した場合は識別番号の更新が必要です。

`roll2026A` の単独ノードは同じロール用ポートを使うため、この実行ファイルと同時には起動しないでください。

`Permission denied` が出る場合は、[シリアルポートの権限設定](../natto_robot_hardware/README.md#初回のシリアルポート権限設定)を行ってください。

## シリアル通信の監視と復帰

注入・ロールは、マイコンからフィードバックが来ない構成を既定とします。
`monitor_serial_updates=false` では、未受信や `serial.update()` の非ゼロ戻り値を
理由に停止・切断・プロセス再起動しません。受信できたID=1のフィードバックは従来どおり配信します。

| パラメータ | 既定値 | 用途 |
|---|---:|---|
| `monitor_serial_updates` | false | `update()` の戻り値による受信監視を有効にするか |
| `serial_update_failure_limit` | 5 | 受信監視有効時、0以外が連続したときの再接続しきい値 |
| `timeout_ms` | 300 | Joy指令の受信期限 |
| `reconnect_interval_ms` | 1000 | 切断・接続失敗後の再試行間隔 |

回数・時間は正の値を指定します。接続直後はゼロ指令を送り、中立のJoy入力で動作を許可します。
その後の新しい非ゼロ指令を送信します。返信を待つ必要はありません。
Joy指令が `timeout_ms` 以上途絶えた場合や入力が不正な場合は停止します。
接続・再接続・指令途絶からの復帰には中立操作が必要で、古い動作指令は引き継ぎません。
待機中・指令途絶中は接続中のポートへ10ms周期で停止指令を再送します。

read/write例外や短いwriteでは停止指令の送信を試み、ポートを閉じて再接続します。
ポートを開く／設定する処理に失敗してもプロセスを終了せず再試行を続けます。
毎回 `serial_path` を開き直すので、USB再接続後にtty名が変わっても、同じ
`/dev/serial/by-id/` に戻れば接続できます。各機構の再接続は独立しています。
返信を監視しないため、書き込みが成功している間のマイコン無応答は検出できません。
USB切断やPC停止時は停止指令が届かないため、マイコン側にも指令受信ウォッチドッグが必要です。

`/diagnostics` の `receive_monitoring`、`watchdog_state`、`command_age_sec`、
`write_success_count`、`write_failure_count`、`reconnect_attempts` で状態を確認できます。
`feedback_age_sec=-1` はフィードバック未受信を示しますが、既定モードでは異常ではありません。

### 返信するファームウェアでの受信監視（任意）

`monitor_serial_updates=true` のときだけ10ms周期で戻り値を監視します。
`0` なら連続異常回数をゼロに戻し、`0` 以外なら1増やします。
`-1`（完全なフレーム未受信）、`-2`（不正パケット）、`-3`（未登録ID）は
異なる値が混在しても連続として数え、設定回数に達すると停止・再接続します。
正常判定はID=1に限定せず、登録済みフレームの `update()==0` を対象とします。
このモードでは `update()==0` の後に中立操作が必要です。

既定の5回は約50msです。`-1` は未受信周期にも出るため、マイコンの送信周期と揺らぎに
合わせて回数を設定してください。送信成功だけでは連続異常回数をリセットしません。
`serial_update_result`、`serial_update_failure_count`、`serial_update_failure_limit`、
`serial_update_age_sec` が診断に追加されます。
以前の `feedback_timeout_ms`、`startup_feedback_timeout_ms`、`feedback_recovery_count` は
注入・ロールでは使用しません。足回り側の監視設定は別です。

これらのノードはSerialBridgeのLinux用ソースをcolconで直接ビルドします。
既存の `bin/libSerialBridge.a` の手動再生成は不要です。


### 長時間の通信不良からの自動再実行

`mecanum_auto.launch.xml` と赤・青版では `auto_serial_restart:=true` が既定です。
従来の即時停止・ポート再接続を続けても復旧しない場合、通信プロセスを終了し、
通信不良が1秒続いたらlaunchが追加待機なしで再実行します。`auto_serial_restart:=false` でこの追加処理を無効化できます。
通常の単体起動では無効です。単体で `auto_restart=true` を指定する場合は、launch側にも
`respawn=true` が必要です。

- `restart_failure_count: 1`: 不良状態の最低検出回数。初回検出で回数条件を満たします。
- `restart_timeout_ms: 1000`: 最初の不良判定からの最低経過時間。回数と時間の両方が必要。
- `restart_recovery_ms: 3000`: 履歴を消すために必要な正常通信の継続時間。

既定の返信不要モードでは、ポートが開いて入出力エラーがない状態の継続を復旧と判定します。
受信監視を有効にした場合は正常受信も必要です。指令途絶・ニュートラル待ちだけでは再起動しません。
再実行後はニュートラル指令が必要で、以前の動作指令は引き継ぎません。
注入・ロールは同一プロセスなので、どちらかの長時間不良で両方が再実行されます。
足回りは別プロセスです。通信が戻らない間は1秒の判定期間ごとに再試行します（プロセスの終了・起動時間は別途かかります）。


## STM個別フレーム通信への移行

[通信移行報告](../../docs/stm_serial_migration.md)を参照してください。
従来の注入/ロールのボタン・軸マッピングは維持し、PID無効の出力をID12〜15のsigned dutyに送ります。
接続・再接続ごとにID0〜3のInitialization（pid_state=0）→Duty=0を送ります。
共通SerialBridgeを使用し、ID0〜20すべての登録結果を検査します。
ID16〜19の最新速度は従来のfb_speed_topicへ、ID20の4入力は
`inject/switch_feedback` / `roll/switch_feedback` (`std_msgs/UInt8MultiArray`) へ公開します。
受信は1回の10ms callbackでdrainし、そのバッチの正常/異常を既存watchdogに渡します。
末尾の `-1` は正常なdrain終了なので、そのバッチの受信成功を上書きしません。
`monitor_serial_updates=false` の既存send-only運用設定は維持しています。
新STMの返信を監視する運用ではtrueを指定してください。
エンコーダPPR・ギア比の設定値はSTM既定値であり、注入/ロール機構ごとに確認してください。
