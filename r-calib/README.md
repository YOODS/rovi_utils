# Robot Calibration

## 要件

|#|項目|手段|
|:----|:----|:----|
|1|パラメータ更新処理|解析結果を既定のパラメータを上書き|
|2|TF更新処理|解析結果を所定のTFにSend|
|3|TF復帰処理|.yamlから電源断前のTFを復旧する|
|4|キャプチャ処理|物体の位置姿勢とロボット位置姿勢を記憶|
|5|解析処理|キャプチャされたデータからTransform(bTc,mTc)を算出|
|6|ロボットインタフェース|座標はTFから取得。リクエスト&ノーティフィハンドシェイク必要|
|7|TFフレームの指定|基準フレーム名、該フレーム名をlaunchのargumentで指定|
|8|Hand/固定カメラ選択|launchのargumentで指定|
|9|結果保存|1の結果をrqt_param_managerの保存機能にて所定.yamlファイルに保存|

## Install

1. VISPスタックをインストール
https://visp.inria.fr/

~~~
sudo apt-get install ros-kinetic-visp
sudo apt-get install ros-kinetic-visp-hand2eye-calibration
~~~

## Launch

RoVIの起動後、以下を追加起動します。

~~~
roslaunch rovi_utils rcalib.launch attach:=Tf_to_attach mount:={world...}
~~~

## Parameter
以下パラメータによる

|name|type|description|
|:----|:----|:----|

## Topics  

launch後のTopicは以下のようにアサインされる。

1. To subscribe

|name|type|description|remapped from|
|:----|:----|:----|:----|
|/rovi/left/image_rect|Image|基準カメラ(左)のレクティファイ画像|gridboard/image_in|
|/robot/clear|Bool|取得したデータをクリアする。|~rcalib/clear|
|/robot/capture|Bool|ロボット座標をTFから得て、物体のTransformとのペアをバッファにストアする。|~rcalib/capture|
|/robot/solve|Bool|ストアされたデータから、機械端からカメラへの座標変換を算出し、パラメータ~rcalib/mTcおよびbTcに出力する。|~rcalib/solve|

2. To publish

|name|type|description|remapped from|
|:----|:----|:----|:----|
|/gridboard/image_out|Image|キャリブ板の認識結果|
|/robot/cleared|Bool|clear処理完了でアサートされます|~rcalib/cleared|
|/robot/captured|Bool|capture処理完了でアサートされます|~rcalib/captured|
|/robot/solved|Bool|solve処理完了でアサートされます|~rcalib/solved|
