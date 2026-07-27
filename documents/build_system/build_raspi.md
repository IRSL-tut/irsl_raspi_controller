# build手順

Ubuntu 20.04 を Rspberry Pi にインストールして、
dynamixelのコントロール、及び、センサーの読み込み(ROSへのパブリッシュ)をネイティブで行うための環境構築である。

## make rspberry pi image
<!-- 昔はGUIからイメージを選択できたが今は不可能 -->
<!--
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
-->

[ここ](https://old-releases.ubuntu.com/releases/20.04.0/)から`ubuntu-20.04.4-preinstalled-server-arm64+raspi.img.xz`をダウンロードして，
Raspberry Pi Imagerの「Use custom」でイメージを書き込む．


書き込み終了後，メディアがマウントされるがその時にuser-dataを書き換える．
これにより初回起動時に以下が設定が実行され，再起動が行われる．
- ユーザirslを追加(password:irsl)
- irslをグループに追加
- SSHを有効化

```
#cloud-config
chpasswd:
  expire: false
  list:
    - irsl:irsl
ssh_pwauth: true
groups:
  - irsl
users:
  - name: irsl
    gecos: IRSL
    primary_group: irsl
    groups: [adm, sudo]
    shell: /bin/bash
    lock_passwd: false
write_files:
  - path: /etc/default/crda
    permissions: '0644'
    content: |
      REGDOMAIN=JP
##Reboot after cloud-init completes
power_state:
  mode: reboot
runcmd:
 - gpasswd -a irsl dialout
 - gpasswd -a irsl video
```



## ライブラリ等のinstall 
- 必要ソフトウェアインストール(apt)
    以下はIPがわかっていない場合は実機（CUI）で実施する．
    また無線LANがつながっていない場合は有線LANを接続すること．
    終了後再起動(```sudo reboot```)することで，GUI操作が可能．
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ubuntu-desktop libyaml-cpp-dev build-essential python3-smbus python-is-python3 screen python3-pip git vim libeigen3* wget
    ```
- 本リポジトリのクローン
    ```bash
    cd ~
    git clone https://github.com/IRSL-tut/irsl_raspi_controller.git
    ```
- ROS(noetic)インストール
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-ros-base python3-rosdep ros-noetic-usb-cam python3-catkin-tools ros-noetic-image-transport ros-noetic-angles ros-noetic-controller-manager ros-noetic-joint-limits-interface ros-noetic-transmission-interface python3-vcstool ros-noetic-ros-control ros-noetic-ros-controllers
    sudo rosdep init
    rosdep update
    ```
- 必要ソフトウェアインストール(pip)
    ```bash
    pip3 install smbus2
    pip3 install numpy --upgrade --ignore-install
    ```

## IPアドレスの固定
（必要に応じて）IPアドレスを固定する．
固定後問題ないかを再起動した後に確認する．

例：ターミナルで`ip a`とコマンドを実行するとインターフェイス毎のIPアドレスがわかる

参考
- https://gihyo.jp/admin/serial/01/ubuntu-recipe/0708
    - GUIによる設定方法

## bashの設定

### .ros_rcの作成
.ros_rcを作成し，ROBOT_IPで指定しているIPアドレスを適宜raspbeery piのIPに書き換える
```
source /opt/ros/noetic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash

export ROBOT_IP=xxx.xxx.xxx.xxx
export ROS_MASTER_URI="http://${ROBOT_IP}:11311/"
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP}
```

### bash_rcの作成

```bash
echo "source ~/.ros_rc" >> ~/.bashrc
```

## cps関係ソフトウェア設定
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/IRSL-tut/sensor_pi.git
cd ~/catkin_ws
wget https://raw.githubusercontent.com/IRSL-tut/irsl_ros_control_shm/refs/heads/main/test/install.noetic.vcs
(cd src; vcs import --recursive < ../install.noetic.vcs)

source /opt/ros/noetic/setup.bash
catkin init
catkin config --install
catkin build irsl_dynamixel_hardware_shm irsl_ros_control_shm sensor_pi
# 一部ビルドがうまくいっていない部分があるので一部削除して再度ビルド
rm -rf build/irsl_ros_control_shm
catkin build irsl_dynamixel_hardware_shm irsl_ros_control_shm sensor_pi
```

## supervisorの追加
### supervisorのインストール
```bash
sudo apt install supervisor
```
### supervisorの設定
```bash
cd ~/irsl_raspi_controller/documents/build_system
sudo cp supervisord.conf /etc/supervisor/supervisord.conf
sudo cp exec_robot.conf /etc/supervisor/conf.d/.
```

### supervisorのコマンド起動
```sudo service supervisor start```


## (任意) choreonoidをインストール
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
sudo apt install ros-noetic-image-transport ros-noetic-angles ros-noetic-controller-manager ros-noetic-joint-limits-interface ros-noetic-transmission-interface ros-noetic-ros-control ros-noetic-ros-controllers
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
catkin build
```
