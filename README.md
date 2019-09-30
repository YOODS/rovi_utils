# RoVI Utilities

## Contents

|ユーテリティ名|アウトライン|ドキュメント|
|:----|:----|:----|
|config_tf|カメラの構成をTFに設定する|[Link](config_tf/README.md)|
|rcalib|カメラとロボットのキャリブレーション|[Link](r-calib/README.md)|
|cropper|点群合成とクロッピング処理|[Link](cropper/README.md)|
|searcher|マスタ／シーンのマッチング|[Link](searcher/README.md)|
|picker|マッチング評価|[Link](picker/README.md)|
|tf_lookup|任意のフレーム間のTransformを表示するツール|[Link](#tf_lookup)|
|tf_euler|任意の回転順序のEuler角⇔Quaternion変換ツール|[Link](#tf_euler)|

## 追加インストール
### Open3D  
Open3Dは0.7以上が必要です。これより古い場合は以下にてアップデートします。
~~~
pip install open3d-python==0.7.0 --user
~~~
インストールにはPipのバージョン9.0.1以上が必要です、これより古い場合は以下にてアップデートします。
~~~
pip install pip==9.0.3
~~~

## Build  
標準手順どおり(catkin_make)

## ツール
<a name="tf_lookup">
## tf_lookup  
1. 使用法
~~~
tf_lookup.py
~~~
にて起動し、標準入力から入力された座標フレームのTransformを標準出力に出力します。
入力の座標フレームは１〜３個を与えることができます。  
2. 入出力例  
フレーム名が１つのときは、world基準のTransformを出力します。以下はworld基準のcameraフレームを出力します。
~~~
camera
~~~
フレーム名が２つのときは、１つ目のフレーム基準の２つ目のフレームのTransformを出力します。以下はbase基準のcameraフレームを出力します。
~~~
base camera
~~~
フレーム名が３つのときは、１つ目のフレーム基準の２つ目のフレームのTransformと同じになる、３つ目のフレームに対する基準フレームを出力します。以下は、base基準のcameraフレームのTransformと、出力されるTransform基準のsolveフレームのそれとが同じとなります。
~~~
base camera solve
~~~

<a name="tf_euler">
## tf_euler
