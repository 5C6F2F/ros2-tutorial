# ROS 2入門教材 設計表

Nov 3, 2026 · @^д^

## 前提条件

教材はUbuntu 24.04上のROS 2 JazzyとC++17で実装し、シミュレーション上の3輪オムニを動かします。寸法などの数値は教材用の仮定値で、実機の値が決まったら差し替えます。

| 項目 | 設計値 |
| --- | --- |
| OS / ROS | Ubuntu 24.04 / ROS 2 Jazzy |
| 言語 / ビルド | C++17 / ament_cmake / colcon |
| 題材 | 3輪オムニの平面運動(x, y, yaw) |
| 測定輪 | `front`, `rear_left`, `rear_right` の3輪 |
| 基準座標 | x前方、y左方、z上方。単位はm、rad、s |
| 主な依存 | `tf2_ros`, `rclcpp_action`, `teleop_twist_keyboard`, `robot_localization` |

| 対象 | 命名規則 | 例 |
| --- | --- | --- |
| Node | 名詞 | `encoder_odometry`, `path_follower` |
| Topic | 名詞 | `encoder_counts`, `cmd_vel` |
| Service | 動詞_目的語 | `generate_path` |
| Action | 動詞_目的語 | `follow_path` |

## パッケージ構成

実機に近い責務ごとに6パッケージへ分け、ハードウェア役と運動学は配布します。読者は通信定義、自己位置推定、ナビゲーション、起動設定を実装します。

| パッケージ | 内容 | 読者の扱い |
| --- | --- | --- |
| `omni_msgs` | `EncoderCounts`, `GeneratePath`, `FollowPath` | 定義を書く |
| `omni_kinematics` | 運動学、姿勢の計算、メッセージ変換のヘッダー | 配布を利用する |
| `omni_fake_hardware` | `fake_mcu`, `fake_lidar_localizer` | 配布を利用する |
| `omni_localization` | `encoder_odometry`, `map_odom_corrector` | 実装する |
| `omni_navigation` | `path_generator`, `path_follower`, `navigation_manager` | 実装する |
| `omni_bringup` | launch、YAML、RViz2設定 | 章に沿って書く |

`omni_bringup` の主なファイルは次のとおりです。

| 種類 | ファイル | 用途 |
| --- | --- | --- |
| launch | `sim.launch.py` | ハードウェア役、自己位置推定、RViz2 |
| launch | `navigation.launch.py` | 経路生成、経路追従、統括 |
| launch | `bringup.launch.py` | 本編のシステム全体 |
| launch | `corrected.launch.py` | `lidar_pose` による手動補正 |
| launch | `ekf.launch.py` | EKFによる統合 |
| YAML | `robot.yaml`, `fake_hardware.yaml`, `navigation.yaml`, `ekf.yaml` | 機体定数と各機能の設定 |
| YAML | `localization.yaml` | オドメトリと `map → odom` 補正の設定 |
| YAML | `scenario/ideal.yaml`, `corrected.yaml`, `ekf.yaml` | 本編、8章、9章で変える値を重ねる |
| YAML | `qos/default.yaml`, `mismatch.yaml`, `recommended.yaml` | 10章で重ねるQoS |
| RViz2 | `main.rviz`, `advanced.rviz` | 本編と発展編の表示 |

`sim.launch.py` の起動引数は次のとおりです。値の設定はYAMLに置き、launchでは起動の切り替えだけを扱います。

| 引数 | 既定値 | 用途 |
| --- | --- | --- |
| `use_fake_mcu` | `true` | `fake_mcu` を起動する |
| `use_rviz` | `true` | RViz2を起動する |
| `use_sim_time` | `false` | bag再生時に `/clock` の時刻で動かす |
| `scenario` | `ideal` | `config/scenario/` から重ねるYAMLを選ぶ |
| `qos` | `default` | `config/qos/` から重ねるYAMLを選ぶ |
| `rviz_config` | `main.rviz` | `rviz/` から読み込む設定を選ぶ |

各ノードは基本のYAMLの後に `scenario` と `qos` のYAMLを読み込み、後の値が優先されます。`corrected.launch.py` と `ekf.launch.py` は、`bringup.launch.py` にシナリオと `advanced.rviz` を指定して起動し、補正系のノードを加えます。

## ノード一覧

本編は教材用の6ノードと標準ツールで構成し、発展編で自己位置の補正系を追加します。`fake_mcu` は `cmd_vel` を受けて真の姿勢を更新し、その動きから測定輪の累積カウントを作ります。

| ノード | 役割 | 区分 | 動作契機 | 登場章 |
| --- | --- | --- | --- | --- |
| `teleop_twist_keyboard` | キーボード入力を `cmd_vel` へ変換する | 標準 | キー入力 | 1 |
| `fake_mcu` | STM32、モーター、測定輪を模擬する | 配布 | 100 Hzタイマー | 1 |
| `encoder_counts_listener` | 受信したカウントをログへ表示する | 実装 | `encoder_counts` 受信 | 2 |
| `encoder_odometry` | カウントからオドメトリを計算する | 実装 | `encoder_counts` 受信 | 3 |
| `path_generator` | 開始姿勢からゴールまでの直線経路を生成する | 実装 | Service要求 | 5 |
| `path_follower` | Pure Pursuitで経路に追従する | 実装 | Goal受理後の50 Hz | 6 |
| `navigation_manager` | ゴール受信から追従完了までを統括する | 実装 | `goal_pose` と応答の受信 | 7 |
| `fake_lidar_localizer` | 真値にノイズを加えて絶対位置を発行する | 配布 | 5 Hzタイマー | 8 |
| `map_odom_corrector` | `lidar_pose` とオドメトリから `map → odom` を計算する | 実装 | `lidar_pose` 受信 | 8 |
| `ekf_local` | エンコーダーとIMUを統合し、`odom → base_link` を発行する | `robot_localization` | 50 Hzタイマー | 9 |
| `ekf_global` | LiDARも統合し、`map → odom` を発行する | `robot_localization` | 50 Hzタイマー | 9 |

## 通信インターフェース

9章までは全topicを既定のQoSで接続し、10章で `encoder_counts` と `planned_path` のQoSを、コードを変えずにYAMLで変更します。

| Topic | 型 | Publisher | Subscriber | 周期 | 登場章 |
| --- | --- | --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | `teleop_twist_keyboard`, `path_follower` | `fake_mcu` | 操作時 / 50 Hz | 1 |
| `encoder_counts` | `omni_msgs/msg/EncoderCounts` | `fake_mcu` | `encoder_odometry` | 100 Hz | 2 |
| `odom` | `nav_msgs/msg/Odometry` | `encoder_odometry` | RViz2, EKF | 100 Hz | 3 |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz2 | `navigation_manager` | ゴール指定時 | 7 |
| `planned_path` | `nav_msgs/msg/Path` | `navigation_manager` | RViz2 | 経路生成時 | 7 |
| `lidar_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `fake_lidar_localizer` | `map_odom_corrector`, EKF | 5 Hz | 8 |
| `imu/data` | `sensor_msgs/msg/Imu` | `fake_mcu` | EKF | 100 Hz | 9 |
| `odometry/local` | `nav_msgs/msg/Odometry` | `ekf_local` | 確認用 | 50 Hz | 9 |
| `odometry/global` | `nav_msgs/msg/Odometry` | `ekf_global` | 確認用 | 50 Hz | 9 |

| 種別 | 名前 | 型 | Server | Client | 登場章 |
| --- | --- | --- | --- | --- | --- |
| Service | `generate_path` | `omni_msgs/srv/GeneratePath` | `path_generator` | `navigation_manager` | 5 |
| Action | `follow_path` | `omni_msgs/action/FollowPath` | `path_follower` | `navigation_manager` | 6 |

| 10章で扱う対象 | 変更するQoS | 確認する現象 |
| --- | --- | --- |
| `encoder_counts` | Publisherを`best effort`へ変更 | `reliable` のSubscriberへ届かない。累積カウントなら欠落から復帰できる |
| `planned_path` | Publisherを`transient local`へ変更 | RViz2を後から起動しても経路を受け取れる |

`fake_mcu`、`encoder_odometry`、`navigation_manager` は該当トピックで `qos_overrides` を有効にしています。`qos/mismatch.yaml` で不一致を、`qos/recommended.yaml` で解消後の設定を再現し、`fake_mcu` の `drop_ratio` でメッセージの欠落を起こします。

## メッセージ定義

カウントは累積値で送り、経路生成に失敗したときは空の経路を返します。累積値は `int32` で約41 km分を表せるため、教材内で桁あふれは起きません。

`EncoderCounts.msg`

```
# 起動時を0とする測定輪の累積カウント
std_msgs/Header header
int32 front
int32 rear_left
int32 rear_right
```

`GeneratePath.srv`

```
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
---
nav_msgs/Path path
```

`FollowPath.action`

```
nav_msgs/Path path
---
bool success
---
float32 distance_to_goal
```

## 座標系とTF

本編は `odom` を基準に動かし、発展編で `global_frame` を `map` へ切り替えます。`path_follower` はTFだけを参照するため、自己位置の計算方法が変わってもコードを変更せずに動きます。

| 親 → 子 | 本編 | 8章 | 9章 |
| --- | --- | --- | --- |
| `odom → base_link` | `encoder_odometry` | `encoder_odometry` | `ekf_local` |
| `map → odom` | なし | `map_odom_corrector` | `ekf_global` |
| `ground_truth_frame → base_link_true` | `fake_mcu`(親は `odom`、有効時のみ) | `fake_mcu` | `fake_mcu` |

| 項目 | 本編 | 発展編 |
| --- | --- | --- |
| RViz2のFixed Frame | `odom` | `map` |
| `global_frame` | `odom` | `map` |
| `base_frame` | `base_link` | `base_link` |
| `publish_base_link_true` | `false` | `true` |
| `ground_truth_frame` | `odom` | `map` |
| `encoder_odometry.publish_tf` | `true` | 8章は`true`、9章は`false` |

`navigation_manager` は、`goal_pose` のフレームが `global_frame` と異なる場合に警告を出して無視します。開始姿勢はTFの `global_frame → base_frame` から取得します。

## 状態遷移

`navigation_manager` はコールバック内で応答を待たず、完了コールバックで状態を進めます。`PLANNING` 中に新しいゴールを受けた場合は最新のゴールを保持し、応答後に経路を生成し直します。

navigation_manager の状態遷移

| 状態 | 行う処理 | 次へ進む契機 |
| --- | --- | --- |
| `IDLE` | ゴールを待つ | `goal_pose` 受信 |
| `PLANNING` | TFから開始姿勢を取得し、`generate_path` を非同期で呼ぶ | Serviceの完了コールバック |
| `FOLLOWING` | `planned_path` を発行し、`follow_path` のGoalを送る | Actionの結果コールバック |
| `CANCELING` | 実行中のGoalへキャンセルを要求する | キャンセルの結果コールバック |

相手のノードが落ちると応答のコールバックは呼ばれないため、`navigation_manager` は100 msごとに応答を監視します。`generate_path` の応答が `planning_timeout` を超えて届かない場合と、`follow_path` のサーバーが見えなくなった場合は、次のゴールへ進むか `IDLE` へ戻ります。

`path_follower` は、空の経路をGoal受理の段階で拒否します。受理後の終了は次の3種類です。

| 終了状態 | `goal_handle` | 条件 |
| --- | --- | --- |
| `SUCCEEDED` | `succeed` | 位置と姿勢が到達判定内に入った |
| `CANCELED` | `canceled` | キャンセル要求を受け、停止指令を発行した |
| `ABORTED` | `abort` | TFの更新が途絶えた時間が `tf_timeout` を超えた |

## パラメータと数値設計

1カウントは約0.019 mmで、100 Hz周期なら本編の速度でも量子化の影響は小さく収まります。`fake_mcu` は累積変位を倍精度で保持し、発行時に丸めることで丸め誤差の蓄積を防ぎます。

測定輪は機体中心から半径 L の円周上に、前方を基準として0°、120°、240°に配置します。カウントの正方向は、機体を反時計回りに回す向きです。

```latex
\begin{aligned}
v_f &= v_y + L\omega \\
v_{rl} &= -\frac{\sqrt{3}}{2}v_x - \frac{1}{2}v_y + L\omega \\
v_{rr} &= \frac{\sqrt{3}}{2}v_x - \frac{1}{2}v_y + L\omega
\end{aligned}
```

```latex
\begin{aligned}
v_x &= \frac{v_{rr} - v_{rl}}{\sqrt{3}} \\
v_y &= \frac{2v_f - v_{rl} - v_{rr}}{3} \\
\omega &= \frac{v_f + v_{rl} + v_{rr}}{3L}
\end{aligned}
```

`fake_mcu` と `encoder_odometry` は、位置積分に区間の中点の姿勢を使います。

| パラメータ | 既定値 | 使用ノード | 意味 |
| --- | --- | --- | --- |
| `wheel_radius` | 0.025 m | `fake_mcu`, `encoder_odometry` | 測定輪の半径 |
| `base_radius` | 0.150 m | `fake_mcu`, `encoder_odometry` | 機体中心から測定輪までの距離 L |
| `counts_per_rev` | 8192 count/rev | `fake_mcu`, `encoder_odometry` | 2048 PPRの4逓倍 |
| `publish_rate` | 100 Hz | `fake_mcu` | `encoder_counts` の発行周期 |
| `cmd_vel_timeout` | 1.0 s | `fake_mcu` | 指令が途絶えたときに停止するまでの時間 |
| `slip_ratio` | 0.0 | `fake_mcu` | 測定輪の変位を減らす基準割合 |
| `slip_wheel_bias` | \[1.0, 0.6, 1.4\] | `fake_mcu` | `slip_ratio` に掛ける車輪ごとの係数 |
| `slip_noise_std` | 0.0 | `fake_mcu` | 変位に加える相対ノイズの標準偏差 |
| `enable_imu` | `false` | `fake_mcu` | `imu/data` を発行するか |
| `publish_base_link_true` | `false` | `fake_mcu` | `base_link_true` を発行するか |
| `ground_truth_frame` | `odom` | `fake_mcu` | `base_link_true` の親フレーム |
| `waypoint_spacing` | 0.05 m | `path_generator` | 経路点の位置間隔 |
| `angular_spacing` | 0.10 rad | `path_generator` | 経路点の角度間隔 |
| `control_rate` | 50 Hz | `path_follower` | 速度指令の周期 |
| `feedback_rate` | 10 Hz | `path_follower` | Feedbackの周期 |
| `lookahead_distance` | 0.30 m | `path_follower` | 先読み距離 |
| `max_linear_speed` | 0.50 m/s | `path_follower` | 並進速度の上限 |
| `max_angular_speed` | 1.00 rad/s | `path_follower` | 角速度の上限 |
| `position_gain` | 1.5 1/s | `path_follower` | ゴール付近の減速ゲイン |
| `yaw_gain` | 2.0 1/s | `path_follower` | 姿勢追従のゲイン |
| `position_tolerance` | 0.02 m | `path_follower` | 位置の到達判定 |
| `yaw_tolerance` | 0.035 rad | `path_follower` | 姿勢の到達判定 |
| `tf_timeout` | 0.5 s | `path_follower` | TFを取得できないときに中断するまでの時間 |
| `lidar_rate` | 5 Hz | `fake_lidar_localizer` | `lidar_pose` の周期 |
| `lidar_position_noise_std` | 0.02 m | `fake_lidar_localizer` | 位置ノイズの標準偏差 |
| `lidar_yaw_noise_std` | 0.02 rad | `fake_lidar_localizer` | 姿勢ノイズの標準偏差 |
| `odom_frame` | `odom` | `encoder_odometry` | オドメトリの親フレーム |
| `base_frame` | `base_link` | `encoder_odometry` | オドメトリの子フレーム |
| `publish_tf` | `true` | `encoder_odometry` | `odom → base_link` を発行するか |
| `planning_timeout` | 1.0 s | `navigation_manager` | `generate_path` の応答を待つ上限 |
| `drop_ratio` | 0.0 | `fake_mcu` | `encoder_counts` を送らない割合 |
| `imu_frame` | `base_link` | `fake_mcu` | `imu/data` のフレーム |
| `imu_gyro_noise_std` | 0.01 rad/s | `fake_mcu` | 角速度のノイズの標準偏差 |
| `lidar_latency` | 0.10 s | `fake_lidar_localizer` | 計測から結果が届くまでの遅れ |
| `twist_covariance_diagonal` | \[1.0e-4, 1.0e-4, 1.0e-2\] | `encoder_odometry` | 速度の分散 \[vx, vy, vyaw\]。EKFの重みになる |
| `correction_gain` | 0.3 | `map_odom_corrector` | 1回の測定でずれを補正する割合 |
| `max_measurement_age` | 0.5 s | `map_odom_corrector` | 計測時刻のオドメトリを待つ上限 |

発展編では `slip_ratio` を0.03、`slip_noise_std` を0.01へ変更します。車輪ごとの係数を変えることで、並進だけでなく姿勢にもドリフトを発生させます。この設定で2 m四方を自動で1周すると、オドメトリ上はゴールに到達しても、真の位置は約0.47 mずれます。

同じ経路を2周したときの真の位置との差は、補正なしで約0.9 m、`map_odom_corrector` で約2 cm、EKFで約2〜4 cmです。EKFではIMUの角速度を統合するため、`odom` 上のずれも約1.1 mから約8 cmに減ります。

## 章と実装範囲の対応

1章は完成版を動かし、2章以降は穴あき版へ章ごとに実装を追加します。各章の完了条件は、コマンドやRViz2で読者自身が確認できる動作にします。

| 章 | 読者が作るもの | 配布を使うもの | 完了条件 |
| --- | --- | --- | --- |
| 1. 完成形を動かす | なし | 完成版一式 | キーボードと2D Goal Poseでロボットが動く |
| 2. Topicとカスタムmsg | `EncoderCounts.msg`, 受信ノード | `fake_mcu` | `encoder_counts` が約100 Hzで届く |
| 3. オドメトリとTF | `encoder_odometry` | 運動学ヘッダー | キーボード操作に合わせて `base_link` が動く |
| 4. launchとbag | launch、YAML、bag操作 | `omni_bringup` の土台 | 1コマンドで起動し、bag再生で同じ軌跡を再現できる |
| 5. Service | `GeneratePath.srv`, `path_generator` | `sim.launch.py` | CLIから呼び出して等間隔の直線経路を得られる |
| 6. Action | `FollowPath.action`, `path_follower` | 経路生成までの構成 | Goal送信で到達し、途中のキャンセルで停止する |
| 7. 統合 | `navigation_manager`, launch | 走行系ノード | 2D Goal Poseで到達し、走行中の再指定にも従う |
| 8. TF | `map_odom_corrector` | `fake_lidar_localizer`, `advanced.rviz` | 滑りによるずれが、`lidar_pose` の補正で抑えられる |
| 9. EKF | `ekf.yaml`, launch | IMU出力、`robot_localization` | 同じ条件で走行し、2段のEKFがTFを発行する |
| 10. QoS | qos_overrides の有効化、QoSのYAML | drop_ratio による欠落の再現 | 通信の不一致を `ros2 topic info -v` で特定し、解消できる |
| 11. 実機への移行 | 実機接続の差し替え | `fake_mcu` の仕様 | 同じ `encoder_counts` と `cmd_vel` で接続できる |
