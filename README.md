# RoVI Utilities

## Contents

|ユーテリティ名|説明|
|:----|:----|
|config_tf|カメラの構成をTFに設定する|
|r-calib|カメラとロボットのキャリブレーション|
|cropper|点群合成とクロッピング処理|
|searcher|マスタ／シーンのマッチング|
|floats3pc|Numpyデータを任意の座標系のPointCloudに変換|
|tk_message|メッセージボックス|
|tf_lookup|任意のフレーム間のTransformを表示|

## Build準備  
- json11  
~~~
git clone https://github.com/dropbox/json11
~~~

## Build  
標準手順どおり(catkin_make)

## その他準備  
- Python-Tkinter
~~~
sudo apt-get install python-tk
~~~
- ローカルライブラリのインストール  
lib/python2.7以下のPythonスクリプトをPYTHONPATHの場所にコピーします。例えば
~~~
cp lib/python2.7/*.py ~/catkin_ws/devel/lib/python2.7/dist-packages
~~~

## カメラの初期設定  
カメラをロボットに設置したときは、まずはr-calib(カメラとアームのキャリブレーション)を行います。ロボットドライバーを起動して  
[カメラとアームのキャリブレーション](r-calib/)
