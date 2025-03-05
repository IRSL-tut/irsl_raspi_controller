# build手順

Ubuntu 20.04 を Rspberry Pi にインストールして、
dynamixelのコントロール、及び、センサーの読み込み(ROSへのパブリッシュ)をネイティブで行うための環境構築である。

## make rspberry pi image
以下設定でイメージを作成
- select RASPBERRYPI5
- Other general-purpose OS
    - Ubuntu
        - Ubuntu Server 24.04.2 LTS (64-bit)

<!-- 次へで設定を編集するでuserとsshの編集を行い，書き込みを行う．
- 一般
    - ユーザー名とパスワードを設定するでuser, passwordを設定する
        - 以下ではuser name:irsl, password:irslが前提
- サービス
    - SSHを有効化する
        - パスワード認証を使う -->
## init settings
- user nameとpasswordを設定する
    - 以下ではuser name:irsl, password:irslが前提



## ライブラリ等のinstall
- 必要ライブラリのインストール
    - 以下コマンドを実行する
        ```
        sudo apt update
        sudo apt upgrade
        sudo apt install git
        ```
- ros-oのインストール
    - [ここ](https://ros.packages.techfak.net/)に記載の通りインストールする．
        ```
        sudo apt install curl
        sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list
        echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
        
        sudo apt update
        sudo apt install python3-rosdep
        sudo rosdep init

        echo "yaml https://ros.packages.techfak.net/ros-one.yaml ubuntu" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
        rosdep update

        sudo apt install ros-one-desktop
        ```

## ライブラリ等のinstall 
<!-- - 必要ソフトウェアインストール(apt)
    以下はIPがわかっていない場合は実機（CUI）で実施する．
    終了後再起動(```sudo reboot```)することで，GUI操作が可能．
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install ubuntu-desktop libyaml-cpp-dev build-essential python3-smbus python-is-python3 screen python3-pip git vim libeigen3*
    ``` -->
- 本リポジトリのクローン
    ```
    cd ~
    git clone https://github.com/IRSL-tut/irsl_raspi_controller.git
    ```
<!-- - ROS(noetic)インストール
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-ros-base python3-rosdep ros-noetic-usb-cam python3-catkin-tools
    sudo rosdep init
    rosdep update
    ``` -->
- 必要ソフトウェアインストール(apt)
    ```
    apt install ssh python3-pip
    ```

- 必要ソフトウェアインストール(pip)
    ```
    pip3 install smbus2 imufusion --break-system-packages
    ```
    <!-- ```
    pip3 install numpy --upgrade --ignore-install
    ``` -->


## cps関係ソフトウェア設定
```
mkdir -p ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b noetic
git clone https://github.com/IRSL-tut/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git -b noetic
git clone https://github.com/IRSL-tut/dynamixel_irsl.git
git clone https://github.com/IRSL-tut/sensor_pi.git
cd ~/catkin_ws
catkin build
```

## bashの設定

### .ros_rcの作成
.ros_rcを作成し，ROBOT_IPで指定しているIPアドレスを適宜raspbeery piのIPに書き換える
```
source /opt/ros/one/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash
```
<!-- export ROBOT_IP=XXX.XXX.XXX.XXX
export ROS_MASTER_URI='http://${ROBOT_IP}:11311/'
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP} -->

### bash_rcの作成

```
echo "source ~/.ros_rc" >> ~/.bashrc
```

## choreonoidをインストール
全部で２時間弱かかるので注意．
### 依存ツールインストール
```
sudo apt install python3-vcstool
```
### ソースダウンロード
<!-- ```
sudo mkdir /choreonoid_ws
sudo chmod 777 /choreonoid_ws/
cd /choreonoid_ws
wstool init src https://raw.githubusercontent.com/IRSL-tut/irsl_choreonoid/main/config/dot.rosinstall 
wstool set choreonoid_ros      https://github.com/IRSL-tut/choreonoid_ros.git      -y -t src -v stable --git
wstool set irsl_choreonoid_ros https://github.com/IRSL-tut/irsl_choreonoid_ros.git -y -t src           --git
wstool update -t src
patch -d src -p0 < src/irsl_choreonoid/config/osqp-cpp.patch
patch -d src -p1 < src/irsl_choreonoid/config/choreonoid_closed_ik.patch
``` -->
```
sudo mkdir /choreonoid_ws
sudo chmod 777 /choreonoid_ws/
cd /choreonoid_ws
mkdir src
wget https://raw.githubusercontent.com/IRSL-tut/irsl_choreonoid/main/config/dot.rosinstall
cat <<- _DOC_ >> dot.rosinstall
### IRSL settings >>> ###
- git:
    local-name: choreonoid_ros
    uri: https://github.com/IRSL-tut/choreonoid_ros.git
    version: stable
- git:
    local-name: irsl_choreonoid_ros
    uri: https://github.com/IRSL-tut/irsl_choreonoid_ros.git
    version: main
- git:
    local-name: cnoid_cgal
    uri: https://github.com/IRSL-tut/cnoid_cgal.git
- git:
    local-name: irsl_sim_environments
    uri: https://github.com/IRSL-tut/irsl_sim_environments.git
- git:    
    local-name: irsl_ros_msgs
    uri: https://github.com/IRSL-tut/irsl_ros_msgs.git
- git:    
    local-name: irsl_raspi_controller
    uri: https://github.com/IRSL-tut/irsl_raspi_controller.git
### IRSL settings <<< ###
_DOC_
```
```
cd /choreonoid_ws/src
vcs import --recursive < ../dot.rosinstall
```

### パッチを当てる
```
cd /choreonoid_ws/src
patch -d src -p1 < src/irsl_choreonoid/config/choreonoid_closed_ik.patch 
find /choreonoid_ws/src/prioritized_qp /choreonoid_ws/src/ik_solvers /choreonoid_ws/src/qp_solvers -name CMakeLists.txt -exec sed -i -e s@-std=c++[0-9][0-9]@-std=c++17@g {} \;
```
### CGALダウンロード
```
cd /choreonoid_ws/src
mkdir cgal
wget https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6.tar.xz -O - | tar Jxf - --strip-components 1 -C cgal
wget https://raw.githubusercontent.com/IRSL-tut/irsl_docker_irsl_system/refs/heads/main/files/cgal_package.xml -O cgal/package.xml
```
## ext追加(必要なら)
```
cd /choreonoid_ws/src/choreonoid/ext
git clone https://github.com/IRSL-tut/robot_assembler_plugin.git
git clone https://github.com/IRSL-tut/jupyter_plugin.git
```
```
sudo mkdir /build_xeus
sudo chmod 777 /build_xeus
sudo apt install -q -qq -y wget cmake g++ git openssl pkg-config libzmq5-dev uuid-dev libssl-dev libsodium-dev lsb-release
wget https://raw.githubusercontent.com/IRSL-tut/irsl_docker_xeus/refs/heads/main/local_build.sh
sed -i.bk s/v3.11.2/v3.11.3/g local_build.sh 
sudo bash local_build.sh
```
### 必要ライブラリダウンロード
```
cd /choreonoid_ws
sudo apt update -q -qq
src/choreonoid/misc/script/install-requisites-ubuntu-24.04.sh
sudo apt install -q -qq -y python3-catkin-tools libreadline-dev ipython3
# エラーが出るので強制インストール
sudo dpkg -i --force-overwrite /var/cache/apt/archives/python3-catkin-tools_0.9.4-5_all.deb
sudo apt install -q -qq -y python3-catkin-tools libreadline-dev ipython3
```
force over writeでいいのかは要検証．
```
cd /choreonoid_ws
rosdep update -y -q -r
rosdep install -y -q -r --ignore-src --from-path src/choreonoid_ros src/irsl_choreonoid_ros

sed -i -e 's@\(#if defined(__x86_64) || defined(_WIN64)\)@\1 || defined(__aarch64__)@g' src/choreonoid/src/AISTCollisionDetector/Opcode/OPC_OptimizedTree.cpp

source /opt/ros/${ROS_DISTRO}/setup.bash
catkin config --cmake-args -DBUILD_TEST=ON
catkin config --install 
catkin build irsl_choreonoid irsl_choreonoid_ros cnoid_cgal irsl_sim_environments irsl_detection_msgs irsl_raspi_controller --no-status --no-notify -p 1
## jupyterを使うときは環境変数込みでコンパイル
# PATH=/opt/xeus3/bin:$PATH LD_LIBRARY_PATH=/opt/xeus3/lib:$LD_LIBRARY_PATH catkin build irsl_choreonoid irsl_choreonoid_ros cnoid_cgal irsl_sim_environments irsl_detection_msgs irsl_raspi_controller --no-status --no-notify -p 1
```

### choreonoid起動
```
source /choreonoid_ws/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 choreonoid
```

### 別ワークスペース再ビルド
```
cd ${HOME}/catkin_ws
rm -rf devel build
catkin build
```

## supervisorの追加
### supervisorのインストール
```
sudo apt install supervisor
```
### supervisorの設定
```
sudo cp supervisord.conf /etc/supervisor/supervisord.conf
sudo cp exec_robot.conf /etc/supervisor/conf.d/.
```

### supervisorのコマンド起動
```sudo service supervisor start```


<!-- 
sudo mkdir /build_xeus
sudo chmod 777 /build_xeus
sudo apt install -q -qq -y wget cmake g++ git openssl pkg-config libzmq5-dev uuid-dev libssl-dev libsodium-dev lsb-release
wget https://raw.githubusercontent.com/IRSL-tut/irsl_docker_xeus/refs/heads/main/local_build.sh
sed -i.bk s/v3.11.2/v3.11.3/g local_build.sh 
sudo bash local_build.sh
-->