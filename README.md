[こちら](https://github.com/IRSL-tut/irsl_raspi_controller) は新しいリポジトリです．

https://github.com/IRSL-tut/cps_rpi_docker が古いリポジトリになります．

# CPS用Raspbberyセットアップおよび使用方法例

## 概要
CPS用の環境を Raspberry Pi 上に作成する方法、及び、簡単な使い方である．
<!-- [cps_rpi_docker](https://github.com/IRSL-tut/cps_rpi_docker)では，
実ロボット用の設定ファイルをリモートに配置し，dockerを使用して実行していたが，
不便な点が多いためそれを解消するためのものである． -->

## 使用方法

以下操作はjupyter上で実施する

jupyerのBashで以下を実行する （シミュレーションを実行する時に行っているかもしれない）

- ```<yourbodyfile>.body```は、作成したbodyファイル名に置き換えてください．

```
rosrun irsl_choreonoid_ros generate_settings.sh --body <yourbodyfile>.body
```

### モータ用configの設定
`dynamixel_config.yaml`のIDの項目をコメントにあるJoint名を参考に書き換える．

### センサ用configの設定
`sensor_setting.yaml`を自分の使用するセンサに合わせて書き換える．
- (例1) TOFセンサ1つの場合
    ```
    I2CHubPublisher:
        'address': '0x70'
        '0':
            address: '0x29'
            name: TOFPublisher
            topic_name: TOFsensor0/value
    ```
- (例2) TOFセンサ2つの場合
    ```
    I2CHubPublisher:
        'address': '0x70'
        '0':
            address: '0x29'
            name: TOFPublisher
            topic_name: TOFsensor0/value
        '1':
            address: '0x29'
            name: TOFPublisher
            topic_name: TOFsensor1/value
    ```
- (例3) カラーセンサ1つの場合
    ```
    I2CHubPublisher:
        'address': '0x70'
        '0':
            address: '0x29'
            name: ColorSensorPublisher
            topic_name: color_value/value
    ```
- (例4) カラーセンサ2つの場合
    ```
    I2CHubPublisher:
        'address': '0x70'
        '0':
            address: '0x29'
            name: ColorSensorPublisher
            topic_name: color_value0/value
        '1':
            address: '0x29'
            name: ColorSensorPublisher
            topic_name: color_value1/value
    ```

### 組み立て手順
1. 使用するモーターの初期位置合わせを行う．
    1. dynamixel_config.yamlのIDを使用するモータのIDに書き換える．
    1. 使用するモータ間をつなげる
    1. USB-serial - dynamixelおよびdynamixel間の配線をする．
    1. 電源を配線するときには電源 - 非常停止付きケーブル - 電源分岐ケーブルの順に配線する．
    1. 非常停止ボタンを解除して電源を入れる（モータのLEDが光ることを確認する）
    1. [本リポジトリ](https://github.com/IRSL-tut/irsl_raspi_controller) のsamples/ipynb/send_settings.ipynb　を使い設定を送信する．
    1. http://XXX.XXX.XXX.XXX:9999 に(XXX.XXX.XXX.XXXはraspberry piのIPアドレス)アクセスしてrun_robotをstartさせる
    1. [本リポジトリ](https://github.com/IRSL-tut/irsl_raspi_controller) のsamples/ipynb/start_ri.ipynbを使い０点にする．
    1. 0点を出したら非常停止ボタンを押す
    1. http://XXX.XXX.XXX.XXX:9999 に(XXX.XXX.XXX.XXXはraspberry piのIPアドレス)アクセスしてrun_robotをstopさせる
1. ロボット組み立て
    - 工具を使ってロボットを組み立てる．
    - USB-serial - dynamixelおよびdynamixel間の配線忘れがちなので注意．
1. 動作確認を行う
    - 動作前に再度0点出しを一緒に行うとよい．	
    - RI(Robot Interface)を使うのでシミュレーションで動かしているのであればそのまま動くはず．
    - [CPS-lectureのRIに関するサンプル](https://github.com/IRSL-tut/CPS-lecture/blob/main/notebooks/cps_lecture_robot_interface.ipynb)だとMASTERとIPの引数を適切に指定しないと実機が動かないので注意．



### サンプルコード
[サンプルコード(send_settings)](samples/ipynb/send_settings.ipynb)

[サンプルコード(start_ri)](samples/ipynb/start_ri.ipynb)

サンプル内の、ロボットの名前（namespace）を確認する

サンプル内の、ロボットに使っている PC(Raspberry pi)のIPアドレス(XXX.XXX.XXX.XXX) と ホストPCのIPアドレス(YYY.YYY.YYY.YYY) を書き換える

RPIControllerのオブジェクトを作成，`send_settigs()`でファイルを送信，`start_robot()`で動作する．

## ハードウェアの作り方
- [Raspberry piのセットアップ方法](documents/build_system/build_raspi.md)
- [Dynamixel ハードウェア準備](documents/build_system/build_hardware.md)
- [センサ用ケーブル作成](documents/sensor_cable/build.md)

