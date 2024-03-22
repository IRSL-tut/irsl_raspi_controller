# build手順

## make rspberry pi image
以下設定でイメージを作成
- select RASPBERRYPI4
- Other general-purpose OS
    - Ubuntu
        - Ubuntu Server 20.04.5 LTS (64-bit)


次へで設定を編集するでuserとsshの編集を行い，書き込みを行う．
- 一般
    - ユーザー名とパスワードを設定するでuser, passwordを設定する
        - 以下ではuser name:irsl, password:irslが前提
- サービス
    - SSHを有効化する
        - パスワード認証を使う

## ライブラリ等のinstall 
- 必要ソフトウェアインストール(apt)
    以下はIPがわかっていない場合は実機（CUI）で実施する．
    終了後再起動(```sudo reboot```)することで，GUI操作が可能．
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install ubuntu-desktop libyaml-cpp-dev build-essential python3-smbus python-is-python3 screen python3-pip git vim libeigen3*
    ```
- ROS(noetic)インストール
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-ros-base python3-rosdep
    sudo rosdep init
    rosdep update
    ```
- 必要ソフトウェアインストール(pip)
    ```
    pip3 install smbus2 imufusion
    pip3 install numpy --upgrade --ignore-install
    ```
## roscoreの自動立ち上げ
### bash_profileの作成
```cp .bash_profile ~/.```

or

```
DEV_IP=`ip addr show eth0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*`
echo ${DEV_IP}
```

DEV_IPの中身を確認したのち以下を実行
```
echo "source ${HOME}/.bashrc" >> ~/.bash_profile
echo "source /opt/ros/noetic/setup.bash" >> ~/.bash_profile
echo "" >> ~/.bash_profile
echo "export ROS_MASTER_URI='http://${DEV_IP}:11311/'" >> ~/.bash_profile
echo "export ROS_IP=${DEV_IP}" >> ~/.bash_profile
```
### bash_rcの作成
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
### roscore service fileのコピー
```sudo cp roscore.service /etc/systemd/system/.```
### サービススタート
```sudo systemctl start roscore```
### ステータスの確認
```systemctl status roscore```
### 自動起動設定
```sudo systemctl enable roscore```

## cps関係ソフトウェア設定
```
mkdir -p ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/IRSL-tut/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/IRSL-tut/dynamixel_irsl.git
git clone https://github.com/Hiroaki-Masuzawa/sensor_pi.git -b use_rosparam
cd ~/catkin_ws
catkin_make
```

## choreonoidをインストール
全部で２時間弱かかるので注意．
### 依存ツールインストール
```
sudo apt install python3-wstool
```
### ソースダウンロード
```
sudo mkdir /choreonoid_ws
sudo chmod 777 /choreonoid_ws/
cd /choreonoid_ws
wstool init src https://raw.githubusercontent.com/IRSL-tut/irsl_choreonoid/main/config/dot.rosinstall 
wstool set choreonoid_ros      https://github.com/IRSL-tut/choreonoid_ros.git      -y -t src -v stable --git
wstool set irsl_choreonoid_ros https://github.com/IRSL-tut/irsl_choreonoid_ros.git -y -t src           --git
wstool update -t src
patch -d src -p0 < src/irsl_choreonoid/config/osqp-cpp.patch
patch -d src -p1 < src/irsl_choreonoid/config/choreonoid_closed_ik.patch
```
### 必要ライブラリダウンロード
```
apt update -q -qq
src/choreonoid/misc/script/install-requisites-ubuntu-20.04.sh
sudo apt install -q -qq -y python3-catkin-tools libreadline-dev ipython3
rosdep update -y -q -r
rosdep install -y -q -r --ignore-src --from-path src/choreonoid_ros src/irsl_choreonoid_ros
```
### コンパイル
```
sed -i -e 's@\(#if defined(__x86_64) || defined(_WIN64)\)@\1 || defined(__aarch64__)@g' src/choreonoid/src/AISTCollisionDetector/Opcode/OPC_OptimizedTree.cpp
catkin config --cmake-args -DBUILD_TEST=ON && catkin config --install && catkin build irsl_choreonoid irsl_choreonoid_ros --no-status --no-notify -p 1 && catkin clean -d -b --logs -y
```
### 別ワークスペース再ビルド
```
cd ${HOME}/catkin_ws
rm -rf devel build
catkin_make
```