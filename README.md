# 3輪オムニで学ぶROS 2

Ubuntu 24.04とROS 2 Jazzyで動く教材用ワークスペースです。3輪オムニをキーボードで動かし、RViz2の2D Goal Poseで指定した場所まで自動で走らせます。発展編では、測定輪が滑ったときのずれをLiDARとEKFで補正します。

## 準備

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_tutorial
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

新しいターミナルでは、ROS 2のコマンドを使う前に毎回 `install/setup.bash` を読み込みます。

## パッケージ

| パッケージ | 内容 |
| --- | --- |
| `omni_msgs` | `EncoderCounts`, `GeneratePath`, `FollowPath` |
| `omni_kinematics` | 運動学、姿勢の計算、ROSメッセージとの変換 |
| `omni_fake_hardware` | マイコンの代役 `fake_mcu`、LiDARの代役 `fake_lidar_localizer` |
| `omni_localization` | `encoder_counts_listener`, `encoder_odometry`, `map_odom_corrector` |
| `omni_navigation` | `path_generator`, `path_follower`, `navigation_manager` |
| `omni_bringup` | launch、YAML、RViz2設定 |

```text
RViz2 ─ goal_pose ─→ navigation_manager ─ generate_path(Service)─→ path_generator
  ▲                          │
  │ planned_path             │ follow_path(Action)
  │                          ▼
  │                    path_follower ←─ TF(odom→base_link)
  │                          │ cmd_vel                ▲
  │                          ▼                        │
  │                       fake_mcu ← cmd_vel ─ teleop_twist_keyboard
  │                          │ encoder_counts         │
  │                          ▼                        │
  └── odom、TF ───── encoder_odometry ────────────────┘
```

## 完成形を動かす

```bash
ros2 launch omni_bringup bringup.launch.py
```

RViz2の上部にある `2D Goal Pose` を選び、地面をドラッグします。押した場所が目標位置、ドラッグした向きが目標の姿勢です。走行中に別の場所を指定すると、新しいゴールへ向かいます。

状態の変化は `navigation_manager` のログで確認できます。

## カウントを受信する

ターミナルを2つ開き、それぞれで次のコマンドを実行します。

```bash
ros2 run omni_fake_hardware fake_mcu
```

```bash
ros2 run omni_localization encoder_counts_listener
```

別のターミナルで、通信を確認できます。

```bash
ros2 interface show omni_msgs/msg/EncoderCounts
ros2 topic hz /encoder_counts
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

## キーボードで動かす

ターミナルを4つ開き、それぞれで次のコマンドを実行します。

```bash
ros2 run omni_fake_hardware fake_mcu
```

```bash
ros2 run omni_localization encoder_odometry
```

```bash
rviz2 -d $(ros2 pkg prefix --share omni_bringup)/rviz/main.rviz
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` で前進、`,` で後退、`j` と `l` で旋回します。Shiftを押しながら `J` や `L` を押すと左右へ平行移動します。キー入力が1秒途絶えると、`fake_mcu` は機体を止めます。

TFの親子関係は次のコマンドで確認できます。

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

## launchで起動する

```bash
ros2 launch omni_bringup sim.launch.py
```

キーボード操作だけは、別のターミナルで `teleop_twist_keyboard` を起動します。

| 引数 | 既定値 | 用途 |
| --- | --- | --- |
| `use_fake_mcu` | `true` | `fake_mcu` を起動する |
| `use_rviz` | `true` | RViz2を起動する |
| `use_sim_time` | `false` | `/clock` の時刻で動かす |
| `scenario` | `ideal` | `config/scenario/` から重ねるYAML |
| `qos` | `default` | `config/qos/` から重ねるYAML |
| `rviz_config` | `main.rviz` | `rviz/` から読み込む設定 |

`bringup.launch.py` は、`sim.launch.py` と `navigation.launch.py` をまとめて起動します。同じ引数を指定できます。

パラメータは `omni_bringup/config` のYAMLで変更します。`--symlink-install` でビルドしていれば、再ビルドは不要です。

| ファイル | 内容 |
| --- | --- |
| `robot.yaml` | 複数のノードで共有する機体定数 |
| `fake_hardware.yaml` | `fake_mcu` の設定 |
| `navigation.yaml` | 経路生成、経路追従、統括の設定 |
| `localization.yaml` | オドメトリと `map → odom` 補正の設定 |
| `ekf.yaml` | 2つのEKFの設定 |
| `scenario/*.yaml` | 本編と発展編で変える値 |
| `qos/*.yaml` | トピックごとのQoS |

各ノードは、基本のYAMLの後に `scenario` と `qos` のYAMLを読み込みます。後に読み込んだYAMLの値が優先されます。

実行中の値も変更できます。

```bash
ros2 param get /fake_mcu wheel_radius
ros2 param set /encoder_odometry wheel_radius 0.03
```

## bagで走行を再現する

システムを起動した状態で、カウントを記録します。走行が終わったら `Ctrl+C` で止めます。

```bash
ros2 bag record -o bags/run1 /encoder_counts
```

いったんシステムを止めてから、`fake_mcu` なしで起動し、記録を再生します。

```bash
ros2 launch omni_bringup sim.launch.py use_fake_mcu:=false use_sim_time:=true
```

```bash
ros2 bag play bags/run1 --clock 100
```

`encoder_odometry` は同じカウントから同じ軌跡を計算します。

## Serviceで経路を生成する

```bash
ros2 launch omni_bringup sim.launch.py
```

```bash
ros2 run omni_navigation path_generator
```

```bash
ros2 service type /generate_path
ros2 service call /generate_path omni_msgs/srv/GeneratePath "{start: {header: {frame_id: odom}, pose: {orientation: {w: 1.0}}}, goal: {header: {frame_id: odom}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}"
```

0.05 m間隔の21点の経路が返ります。開始姿勢とゴールのフレームが異なるときは、空の経路が返ります。

## Actionで経路に追従する

`sim.launch.py` を起動した状態で、`path_follower` を起動します。

```bash
ros2 run omni_navigation path_follower
```

```bash
ros2 action info /follow_path
ros2 action send_goal --feedback /follow_path omni_msgs/action/FollowPath "{path: {header: {frame_id: odom}, poses: [{pose: {orientation: {w: 1.0}}}, {pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}]}}"
```

残り距離が約10 Hzで表示され、到達すると `SUCCEEDED` になります。走行中に `Ctrl+C` を押すと、Goalがキャンセルされて機体が止まります。

| 終了状態 | 条件 |
| --- | --- |
| `SUCCEEDED` | 位置と姿勢が到達判定の範囲に入った |
| `CANCELED` | キャンセル要求を受けて停止した |
| `ABORTED` | TFを `tf_timeout` の間取得できなかった |

空の経路、`global_frame` と異なるフレームの経路、走行中に届いた別のGoalは拒否します。

## 滑りを試す

`fake_mcu` の測定輪に滑りを加えると、オドメトリが実際の位置からずれていきます。

```bash
ros2 param set /fake_mcu slip_ratio 0.03
ros2 param set /fake_mcu slip_noise_std 0.01
```

`publish_base_link_true` を `true` にすると、真の姿勢を `odom → base_link_true` のTFとして確認できます。親フレームは `base_link_true_parent_frame` で変更できます。

この状態で2 m四方を自動で2周させると、オドメトリ上はゴールに着いていても、真の位置は約0.9 mずれます。

## TFでずれを補正する(8章)

```bash
ros2 launch omni_bringup corrected.launch.py
```

`scenario/corrected.yaml` で測定輪を滑らせ、真の姿勢を `map → base_link_true` として出力します。`fake_lidar_localizer` は、真の姿勢にノイズを加えた絶対位置を5 Hz、0.1秒遅れで `lidar_pose` に出力します。

`map_odom_corrector` は `odom → base_link` を変えず、ずれの補正を `map → odom` に任せます。

```text
map ──(map_odom_corrector)──→ odom ──(encoder_odometry)──→ base_link
 └──────────────(fake_mcu)───────────────→ base_link_true
```

1. `lidar_pose` を受け取ったら、計測した時刻の `odom → base_link` が届くまで待ちます。
2. `map → odom` = `map → base_link` ∘ (`odom → base_link`)⁻¹ でずれを求めます。
3. ノイズで機体が跳ねないよう、`correction_gain` の割合だけ測定値へ近づけます。

| 確認すること | コマンド |
| --- | --- |
| TFの親子関係 | `ros2 run tf2_tools view_frames` |
| `map` 上の位置は補正のたびに動く | `ros2 run tf2_ros tf2_echo map base_link` |
| `odom` 上の位置はなめらかなまま | `ros2 run tf2_ros tf2_echo odom base_link` |
| 補正の強さ | `ros2 param set /map_odom_corrector correction_gain 1.0` |

2 m四方を2周しても、真の位置との差は約2 cmに収まります。代わりに `odom` は `map` から約1 mずれていきます。

## EKFで統合する(9章)

```bash
ros2 launch omni_bringup ekf.launch.py
```

`scenario/ekf.yaml` で `fake_mcu` のIMU出力を有効にし、`encoder_odometry` のTF発行を止めます。`robot_localization` の `ekf_node` を2つ起動し、役割を分けます。

| ノード | 入力 | 発行するTF | 出力トピック |
| --- | --- | --- | --- |
| `ekf_local` | 測定輪の速度、IMUの角速度 | `odom → base_link` | `odometry/local` |
| `ekf_global` | 上記とLiDARの絶対位置 | `map → odom` | `odometry/global` |

| センサー | 得意なこと | 苦手なこと |
| --- | --- | --- |
| 測定輪 | 並進の速度 | 滑ると誤差が蓄積する |
| IMU(ジャイロ) | 滑りに影響されない角速度 | 位置はわからない |
| LiDAR | 誤差が蓄積しない絶対位置 | 周期が遅く、結果が遅れて届く |

各センサーの重みは、メッセージの共分散で決まります。測定輪の速度の分散は `localization.yaml` の `twist_covariance_diagonal`、LiDARとIMUの分散は `fake_hardware.yaml` のノイズの標準偏差から計算されます。LiDARの遅れは `ekf.yaml` の `smooth_lagged_data` で扱います。

2 m四方を2周したときの、真の位置との差は約2〜4 cmです。IMUで角速度を補うので、`odom → base_link` のずれは約8 cmに抑えられます。

`ekf_local` と `encoder_odometry` が両方 `odom → base_link` を発行すると、TFが乱れます。`ros2 run tf2_tools view_frames` で発行元を確認できます。

## QoSを確かめる(10章)

`qos` 引数で、`config/qos/` のYAMLを重ねます。QoSは起動時に読み込まれるので、変更したらノードを起動し直します。

| ファイル | 内容 |
| --- | --- |
| `default.yaml` | 既定のQoSを明示したもの |
| `mismatch.yaml` | `encoder_counts` の送信側だけ `best_effort` にする |
| `recommended.yaml` | `encoder_counts` を `best_effort`、`planned_path` を `transient_local` にする |

### 送受信のQoSが合わない

```bash
ros2 launch omni_bringup sim.launch.py qos:=mismatch
```

キーボードで動かしても、RViz2のロボットは動きません。`ros2 topic echo /encoder_counts` では値が見えるので、送信自体は止まっていません。

```bash
ros2 topic info -v /encoder_counts
```

送信側が `BEST_EFFORT`、受信側が `RELIABLE` になっています。受信側が再送を求めても、送信側はそれに応えられないため、つながりません。ログにも `incompatible QoS` の警告が出ます。

### メッセージが欠けても位置を失わない

```bash
ros2 launch omni_bringup bringup.launch.py qos:=recommended
```

```bash
ros2 param set /fake_mcu drop_ratio 0.3
```

`encoder_counts` の3割が届かなくなりますが、カウントは累積値なので、次に届いたメッセージで差分を取り戻せます。自動で走らせても、真の位置とのずれは生じません。

### 後から起動したノードに経路を渡す

`recommended.yaml` では、`planned_path` の送信側が最後の1件を保持します。ゴールを指定した後に次のコマンドを実行すると、すぐに経路を受け取れます。

```bash
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /planned_path
```

既定のQoSでは、送信側が履歴を保持しないため、受信できません。`incompatible QoS` の警告も出ます。

## 実機へ移行する(11章)

`fake_mcu` を外し、同じトピックを扱う実機のノードに差し替えます。

```bash
ros2 launch omni_bringup bringup.launch.py use_fake_mcu:=false
```

| 方向 | トピック | 型 | 実機側で守ること |
| --- | --- | --- | --- |
| 受信 | `cmd_vel` | `geometry_msgs/msg/Twist` | 機体座標系の速度で受け取る。`cmd_vel_timeout` の間途絶えたら停止する |
| 送信 | `encoder_counts` | `omni_msgs/msg/EncoderCounts` | 起動時を0とする累積カウントを送り、計測時刻を `header.stamp` に入れる |
| 送信 | `imu/data` | `sensor_msgs/msg/Imu` | 角速度とその分散を入れ、使わない項目は共分散の先頭を-1にする |

測定輪の向きと正方向は `omni_kinematics.hpp` の冒頭に、寸法は `robot.yaml` にまとめています。

## テスト

```bash
colcon test
colcon test-result --verbose
```
