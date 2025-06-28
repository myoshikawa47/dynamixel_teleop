# 🦾 Dynamixel Teleoperation System

このリポジトリは、**Dynamixelモータを用いたテレオペレーションシステム**を提供します。PythonおよびROSで構築されており、次のような特徴を持ちます：

- `GroupSyncRead` / `GroupSyncWrite` による高速な通信処理
- OpenManipulatorとの即時互換（モータIDと通信速度設定のみで使用可能）
- 線形補間による滑らかな追従制御
- 柔軟な設定変更：YAMLファイルでモータ構成や制御モードを切り替え可能

---

## 🖧 システム概要（System Overview）

### 🔗 接続構成
```
PC ⇄ U2D2 ⇄ Leader Arm
   ⇄ U2D2 ⇄ Follower Arm
```

### 📡 ROS通信トピック

| ノード名       | 購読トピック                 | 配信トピック                   |
|----------------|------------------------------|--------------------------------|
| `leader_node`  | -                            | `/leader/joint_state`          |
| `interpolation_node` | `/leader/joint_state`        | `/leader/online_joint_state`   |
| `follower_node`| `/leader/online_joint_state` | `/follower/joint_state`        |

<p align="center">
  <img src="assets/system_overview.png" width="60%">
</p>

---

## 🔧 ハードウェア設定

### 🧰 ドライバ & 権限設定

```bash
# Dynamixel Wizardインストール
cd ~/Downloads/
wget -O DynamixelWizard2Setup_x64 "https://www.dropbox.com/s/csawv9qzl8m8e0d/DynamixelWizard2Setup-x86_64?dl=1"
chmod +x DynamixelWizard2Setup_x64
./DynamixelWizard2Setup_x64

# USBアクセス許可
sudo usermod -aG dialout "$USER"

# udevルール設定
wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules
sudo mv 99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 🔢 モータID割り当て

- 通信速度：**4 Mbps（全モータ共通）**

| 関節         | Leader ID | Follower ID |
|--------------|-----------|-------------|
| arm/joint1   | 11        | 21          |
| arm/joint2   | 12        | 22          |
| arm/joint3   | 13        | 23          |
| arm/joint4   | 14        | 24          |
| arm/joint5   | 15        | 25          |

### 🔌 udevによるデバイス名固定
現在のデバイスパス（例：/dev/ttyUSB0）を取得し、シリアル番号を取得

#### シンボリックリンクの例
- `/dev/ttyDXL_leader`
- `/dev/ttyDXL_follower`

#### 設定手順

```bash
# シリアル番号を確認
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial

# udevルール作成
sudo nano /etc/udev/rules.d/99-fixed-dynamixel.rules
```

次のような行を追加：
```bash
SUBSYSTEM=="tty", ATTRS{serial}=="<serial_leader>", SYMLINK+="ttyDXL_leader"
SUBSYSTEM=="tty", ATTRS{serial}=="<serial_follower>", SYMLINK+="ttyDXL_follower"
```

```bash
# 設定の反映
sudo udevadm control --reload
sudo udevadm trigger

# 動作確認
ls /dev/ttyDXL_*
```

---

## 💻 ソフトウェアインストール

### ① ROS Noetic（Ubuntu 20.04）
[公式インストールガイド](https://wiki.ros.org/noetic/Installation/Ubuntu) に従ってセットアップ

### ② Dynamixel SDK

```bash
pip3 install -U pip
pip3 install dynamixel_sdk
```

### ③ ROSパッケージのビルド

```bash
cd ~/catkin_ws/src/
git clone https://github.com/ogata-lab/dynamixel_teleop.git
cd ~/catkin_ws
catkin build
```

---

## 🚀 実行方法（テレオペレーション）

`rosparam`に登録された制御周期に基づいて`interpolation_node`が線形補間を行うため、必ずリーダアーム、フォロワアームの順に実行すること

```bash
# Leader アーム
roslaunch leader_controller leader_bringup.launch

# Follower アーム
roslaunch follower_controller follower_bringup.launch
```

---

## ⏱ 通信確認コマンド
制御周期が設定通りであるか確認

```bash
rostopic hz /leader/joint_states          # 10 Hz 程度
rostopic hz /leader/online_joint_states   # 100 Hz 程度
rostopic hz /follower/joint_states        # 100 Hz 程度
```

---

## 🔧 カスタマイズ方法

### ✅ 設定ファイルの編集（例：YAML）
制御方式（例：Position Control, Current-based Pos. Controlなど）やモータIDは`yaml`ファイルで設定

```yaml
device: '/dev/ttyDXL_leader'
baudrate: 4000000
control_freq: 10

arm/joint1:
  id: 11
  operating_mode: 3  # Position Control
```

### ✅ Launchファイル切替
新しい設定ファイル `follower_new.yaml` を使用する場合、`load_config.py`の引数`param_name`を変更

```xml
<launch>
  <node pkg="robot_description" type="load_config.py"
        args="--param_name=follower --config=$(find robot_description)/config/follower_new.yaml"/>
  <node pkg="follower_controller" type="follower_node.py" output="screen"/>
  <node pkg="follower_controller" type="interpolation_node.py" output="screen"/>
</launch>
```

---

## 💡 応用：オンライン動作生成
深層予測学習や基盤モデルを用いてオンライン動作生成する場合、以下のように実装できます。
なお、モデルの推論周期 `freq` とinterpolation_nodeの `leader_freq` を一致させることを忘れないでください。

```python
import rospy
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import numpy as np

class RTControl:
    def __init__(self, exptime=10, freq=10):
        self.rate = rospy.Rate(freq)
        self.loop = int(freq * exptime)
        self.bridge = CvBridge()

        rospy.Subscriber("/follower/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/leader/joint_states", JointState, queue_size=1)
        self.msg = JointState()

    def joint_callback(self, msg):
        self.current_position = np.array(msg.position)

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def run(self):
        for _ in range(self.loop):
            pred = model(self.current_position, self.current_image)
            self.msg.header.stamp = rospy.Time.now()
            self.msg.position = pred
            self.pub.publish(self.msg)
            self.rate.sleep()
```