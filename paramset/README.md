# Paramset

## 機能  
- 関連パラメータの一括変更を行う

## 使用例
### パラメータ構造
~~~
pshift_genpc:
  camera:
    ExposureTime: 8400
    Gain: 0
  projector:
    ExposureTime: 20
    Intencity: 120
    Interval: 40
    Mode: 1
  paramset:
    camera:
      ExposureTime: [8400,8400,8400,16800,16800]
      Gain: [0,0,0,0,0]
    projector:
      Intencity: [40,120,255,100,255]
      Interval: [40,40,40,60,60]
~~~
上のような"pshift_genpc"以下のパラメータ構造にて、直下に"paramset"以下の構造を追加する。ここには"pshift_genpc"以下のパラメータの選択可能な値リストが記述される。
### paramset.py  
paramsetは、"pshift_genpc"直下にparamsetNの整数型パラメータを追加する。paramsetNに入力された数値によって、値リストから一つを選び対応するパラメータの値を一括で変更する。  
#### paramset.py起動書式
~~~
paramset.py set:=[pshift_genpc]
~~~
直下にparamsetが存在する、パラメータを指定する。通常このような構造は複数存在するで、リストで与える。
