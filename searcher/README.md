## Searcher

### 要件
- 入力(モデルおよびシーン)点群を解析し、モデルとシーン間の座標変換を求める
- 求めた座標変換をTFに送る
- 解析には様々なアルゴリズム(Open3D,Halcon)が使えるように、アルゴリズム部を外部モジュール化する

### UML
![ユースケース図](uml/usecase.png)

## 準備
### Open3D
~~~
pip install open3d-python --user
~~~
OPen3D-0.6からPipのバージョンが9.0.1以上が必要となりました。これより古いときは
~~~
pip install pip==9.0.3
~~~

### 単体テスト
Open3Dソルバー(o3d_solver.py)の単体テストは、このモジュールを単体でRUNします。
~~~
./o3d_solver.py
~~~

## Parameter
### ~Config

|name|type|description|
|:----|:----|:----|
|register_frame_id|string[]|登録時にファイル保存するフレーム名。別名のマスターフレーム(*master0..n*)が生成される|
|place_frame_id|string|解析結果フレーム(*search0..n*)を配置するフレーム名|

設定例(ハンドアイピッキング)
~~~
Config={
  "lines":["surface"],
  "solver":"o3d_solver",
  "register_frame_id":["camera/capture0"],
  "place_frame_id":"camera/capture0"
}
~~~
設定例(位置合わせ)
~~~
Config={
  "lines":["surface","marker","edge"],
  "solver":"???_solver",
  "register_frame_id":["flange/capture0","flange/capture1","flange/capture2"],
  "place_frame_id":"flange"
}
~~~

### ~  
ローカルパラメータ領域にSolverパラメータを配します。

### Example  
ハンドカメラで撮影した点群を、ベース座標系に変換し出力する。ソルバーは標準ソルバー(o3d_solver.py)を使う。
~~~
  <node ... args="input:=c parent:=m output:=b solver:=$(find rovi_searcher)/o3d_solver" />
~~~
メカニカルインタフェース座標系の点群を、同座標系で出力する。ソルバーはパッケージwpcのソルバー(wpc/solver.py)を使う。
~~~
  <node ... args="input:=m solver:=$(find wpc)/solver" />
~~~

## Topics
### To subscribe

|Topic|タイプ|説明|
|:----|:----|:----|
|~in/*/floats|Floats|シーン点群。*はlines argumentで与えた点群の系列名|
|~clear|Bool||
|~solve|Bool||
|~load|Bool||
|~save|Bool||

### To publish

|Topic|タイプ|説明|
|:----|:----|:----|
|~master/*/float|Floats|モデル点群。モデルをロードしたとき、およびシーンをsubscribeしたとき初期位置の点群をpublishする|