こちらは新しいリポジトリです．

[こちら](https://github.com/IRSL-tut/cps_rpi)が新しいリポジトリになります．

[こちら](https://github.com/IRSL-tut/cps_rpi_docker)が古いリポジトリになります．


# CPS用Raspbberyセットアップおよび使用方法例
## 概要
CPS用の環境を作成する方法および簡単な使い方である．
<!-- [cps_rpi_docker](https://github.com/IRSL-tut/cps_rpi_docker)では，
実ロボット用の設定ファイルをリモートに配置し，dockerを使用して実行していたが，
不便な点が多いためそれを解消するためのものである． -->

## 準備
- [Raspberry piのセットアップ方法](build_rpi/build.md)
- [センサ用ケーブル作成](sensor_cable/build.md)

## 使用方法
以下操作はjupyter上で実施する

### センサ用configの設定
`robot_sensor.yaml`を自分の使用するセンサに合わせて書き換える．
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

### モータ用configの設定
`controller_config.yaml`と`dynamixel_config.yaml`を書き換える．
1. controller_config.yaml
    - (例1) アーム型ロボットの場合
    ```
    ## controlelr settings
    dxl_read_period: 0.01
    dxl_write_period: 0.01
    publish_period: 0.01
    ```
    - (例2) 車輪型ロボットの場合
    ```
    ## controlelr settings
    dxl_read_period: 0.01
    dxl_write_period: 0.01
    publish_period: 0.01
    ## wheel controller settings
    mobile_robot_config:
      actuator_id: # 自分の使用するモータのIDを記載する
      - X 
      - Y
      - Z
      - W
      actuator_mounting_angle: # 回転軸の方向
      - -1.57079632679 
      - 0.0
      - 1.57079632679
      - 3.14159265359
      omni_mode: true
      radius_of_wheel: 0.024
      seperation_between_wheels: 0.16
    ```
1. dynamixel_config.yaml
   - (例1) アーム型ロボットの場合
    ```
    LINK_0:             # 該当するJointName
        ID: X           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 3
    LINK_1:             # 該当するJointName
        ID: Y           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 3
    LINK_2:             # 該当するJointName
        ID: Z           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 3
    LINK_3:             # 該当するJointName
        ID: W           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 3
    ```
   - (例2) 車輪型ロボットの場合
   ```
    WHEEL_0:            # 該当するJointName
        ID: X           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 1
    WHEEL_1:            # 該当するJointName
        ID: Y           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 1
    WHEEL_2:            # 該当するJointName
        ID: Z           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 1
    WHEEL_3:            # 該当するJointName
        ID: W           # モータID
        Return_Delay_Time: 0
        Operating_Mode: 1
   ```
### サンプルコード
[サンプルコード](userdir/sample_supervisor.ipynb)
RPIControllerのオブジェクトを作成，`send_settigs()`でファイルを送信，`start_robot()`で動作する．
