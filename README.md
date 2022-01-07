# robot3_ros
ROS

6自由度マニピュレータで中空のボールをキャッチするシミュレータ

# ライブラリ
time

numpy

# 配置
現在画像準備中

# 概要
コマンドを用いて発射台の位置、姿勢、初速を指定し、ボールを発射します。

マニピュレータは発射されたボールの最高到達点に向かって動き、最高到達点に達する時刻にハンドを閉じます。

ハンドを閉じた瞬間ターミナルにハンドの座標系から見たボールの相対座標とキャッチ出来たか否かが表示されます。また、キャッチ出来たときはrviz上のボールが停止します。

**物理シミュレータではありません**。マニピュレータの方は長さ以外の物理的パラメータは設定していません。

ボールの軌跡は放物線を描きますが、マニピュレータは重力も遠心力も受けません。

# 使い方
### 1.コマンドで発射台の位置、姿勢を指定
下記コマンドで発射台の位置、姿勢を指定します。

コマンド末尾のリスト内、**pos_x**は発射台のx座標(単位はm、-10~10以内)、**pos_z**はz座標(単位はm、-10~0以内)、**att_y**はy軸まわりの角度(弧度法、-4~4)、**att_xz**はxz平面に対する角度(弧度法、0~3.15)を入れてください。
```
rostopic pub -1 /shooter_state_input std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 4
    stride: 0
  data_offset: 0
data: [pos_x, pos_z, att_y, att_xz]"
```

### 2.コマンドでボールの初速を指定、発射
下記コマンドでボールの初速を指定し、発射します。
コマンド末尾の **v_0** に発射した瞬間の速さ(単位はm/s)を入れてください。その速さでボールが発射されます。
```
rostopic pub -1 /shoot_value std_msgs/Float32 "data: v_0"
```

# ROSノード、トピック
## ノード、トピックの関係
![nodes](./nodes.png)

## 各トピックについて
### /shooter_state_input
std_msgs/Float32MultiArray型

x座標(m)、z座標(m)、y軸まわりの角度(rad)、xz平面となす角度(rad)の4個の要素からなります。

ユーザがコマンドを用いて入力します。

### /shoot_value
std_msgs/Float32型

ボールの初速(m/s)です。

ユーザがコマンドを用いて入力します。

### /shooter_state
std_msgs/Float32MultiArray型

x座標(m)、z座標(m)、y軸まわりの角度(rad)、xz平面となす角度(rad)の4個の要素からなります。

### /ball_initial_state
std_msgs/Float32MultiArray型

発射された瞬間の時刻(s)、xyz座標(m)、速度のxyz成分(m/s)の7個の要素からなります。

### /target_arm_state
std_msgs/Float32MultiArray型

ハンドの目標となるxyz座標(m)、姿勢を表す行列、速度のxyz成分(m/s)、姿勢の微分の24個の要素からなります。

### /hand_close_reserve
std_msgs/Float32型

ハンドを閉じるべき時刻(s)です。

### /arm_ang_angv
std_msgs/Float32MultiArray型

マニピュレータの各軸の角度(rad)、角速度(rad/s)の12個の要素からなります。

### /hand_close
robot3_18c1054/msg/HandClose型

HandClose型の定義は下記の通りです。

#### HandClose.msg
```
bool close
float32 time_stamp
```

ハンドを閉じるか開けるか(閉じるならばTrue、開けるならばFalse)、publishした時刻(s)の2個の要素からなります。


## 各ノードについて
### visualizer
0.1(デフォルト)秒に1回の頻度で/joint_statesをpublishします。

/hand_closeをsubscribeすると、GetHandStateサービスを用いてハンドの位置、速度、姿勢を取得し、ボールの現在位置、速度と比べることでキャッチしたかを判定します。

### arm_controller
0.1(デフォルト)秒に1回の頻度で各軸の現在の角度、角速度を更新、それを/arm_ang_angvでpublishします。

この際、角度、角速度はPD制御で制御し、現在の角度と目標の角度の差が大きいならばDゲインは0、現在の角速度が速すぎるならばP、Dゲインともに0、どちらでもないならば適当な安定となるようなゲインを用います。

0.013(デフォルト)秒に1回の頻度でハンドを閉じる(開く)べきかを判定します。もし閉じる(開く)必要があるならばTrue(False)を/hand_closeでpublishします。

/hand_close_reserveをsubscribeしたときはトピックからハンドを閉じるべき時刻を取得し上の判定に用います。

GetHandStateサービスを要求されたときはその時点のハンドの位置、姿勢(を表す行列)、速度、姿勢の微分を返します。

### shooter_controller_simple
/shooter_state_inputをsubscribeすると、トピック内データに従い発射台の位置、姿勢を更新、/shooter_stateをpublishします。また、更新の際、動ける範囲("使い方"または"諸々のパラメータ"参照)を超えた値をsubscribeしたならば範囲を超えず、最もsubscribeした値に近い値を使用します。

/shoot_valueをsubscribeすると、
