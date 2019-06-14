# RoVI Utilities

## Contents

- config_tf/...  
カメラの構成をTFに設定する
- r-calib/...  
カメラとロボットのキャリブレーション
- cropper/..  
点群合成とクロッピング処理
- searcher/..  
マスタ／シーンのマッチング 
- floats3pc  
Numpyデータを任意の座標系のPointCloudに変換

## Build準備  
- json11  
~~~
git clone https://github.com/dropbox/json11
~~~

## Build
標準手順どおり(catkin_make)

## カメラの初期設定  
カメラを設置したときは、以下の手順にて初期設定が必要です。

1. config_tf  
構成の設定
2. r-calib  
カメラとアームのキャリブレーション
