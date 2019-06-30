# RoVI Utilities

## Contents

|ユーテリティ名|説明|
|:----|:----|
|config_tf|カメラの構成をTFに設定する|
|r-calib|カメラとロボットのキャリブレーション|
|cropper|点群合成とクロッピング処理|
|searcher|マスタ／シーンのマッチング|
|tk_message|メッセージボックス|
|tf_lookup|任意のフレーム間のTransformを表示|
|launch_manager|roslaunchの管理|
|recipe_manager|recipeの管理|

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

## rovi_utils-19.10 release note
- **new**
    - launch_manager
    - picker 干渉チェック機能
- searcher
    - Parameter::torelanceを追加。許容範囲を超えるFrameはpublishしない
    - Frame::solve...をマスターとの距離でソート
- recipe_manager
    - 「名前を変えて保存」機能追加
