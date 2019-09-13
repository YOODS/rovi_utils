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
- tkFileDialog
~~~
pip install tkfilebrowser --user
~~~
### Open3D  
Open3Dは0.7以上が必要です。これより古い場合は以下にてアップデートします。
~~~
pip install open3d-python==0.7.0 --user
~~~
インストールにはPipのバージョン9.0.1以上が必要です、これより古い場合は以下にてアップデートします。
~~~
pip install pip==9.0.3
~~~

## カメラの初期設定  
カメラをロボットに設置したときは、まずはr-calib(カメラとアームのキャリブレーション)を行います。ロボットドライバーを起動して  
[カメラとアームのキャリブレーション](r-calib/)

## rovi_utils-19.10 release note
- **new** picker  
solverが出力した候補から、ひとつを選ぶ。干渉チェック機能を含む。
