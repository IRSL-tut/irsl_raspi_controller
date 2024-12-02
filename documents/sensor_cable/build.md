# センサー系統の準備

## 必要物品

- [2 x 20 pin コネクタ](https://jp.misumi-ec.com/vona2/detail/222000486813/?HissuCode=PS-40SD-D4C2)
- [コネクタ用コンタクト](https://jp.misumi-ec.com/vona2/detail/222000484057/?HissuCode=030-51307-001)
- [レベルコンバータIC](https://ssci.to/2375)

## ケーブル結線図

TBD

## ピンアサイン
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
