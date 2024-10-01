
# ハードウェア準備
- 必要な物品
    - [Dynamixel contorller](https://www.besttechnology.co.jp/modules/onlineshop/index.php?fct=photo&p=291)
        - これは代替品でも構わない
    - Dynamixel 電源供給基盤部品
        - ユニバーサル基盤
        - [JSTコネクタ](https://e-shop.robotis.co.jp/product.php?id=373)
            -購入先は一例
        - 電源用コネクタ
            - XT60を推奨
        - 配線
    - Sensor接続用基盤部品
        - [2 x 20 pin コネクタ](https://jp.misumi-ec.com/vona2/detail/222000486813/?HissuCode=PS-40SD-D4C2)
        - [コネクタ用コンタクト](https://jp.misumi-ec.com/vona2/detail/222000484057/?HissuCode=030-51307-001)
        - [レベルコンバータIC](https://ssci.to/2375)
        - 配線
- 組み立て
    - Dynamixel 電源供給基盤部品
        - JSTコネクタ1番ピンをGND,2番ピンに電源を供給するようにする．
    - Sensor接続用基盤部品
        - 以下通りに配線を行う．
            | 2x20 pin コネクタ| レベルコンバータIC | Grove Connector|
            | ---- | ---- | ---- |
            | pin 1 (3.3V) | pin 1 (VCC A) | - |
            | pin 5 (SDL) | pin 3 (SCLA) | - |
            | pin 3 (SDA) | pin 2 (SDAA) | - |
            | pin 2 (5V) | pin 8 (VCCB) | pin 3 (VCC) |
            | - | pin 7 (SDLB) | pin 1 (SDL) |
            | - | pin 6 (SDAB) | pin 2 (SDA) |
            | pin 6 (GND) | pin 4 (GND) | pin 4 (GND)|
            |-|pin 5 (EN)|-|

