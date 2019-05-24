# Robot Calibration

## 要件

|#|項目|手段|
|:----|:----|:----|
|1|パラメータ更新処理|解析結果を既定のパラメータを上書き|
|2|TF更新処理|解析結果を所定のTFにSend|
|3|TF復帰処理|.yamlから電源断前のTFを復旧する|
|4|キャプチャ処理|Topicからトリガを取得<br>物体の位置姿勢とロボット位置姿勢を記憶<br>完了をTopicへ通知|
|5|解析処理|Topicからトリガを取得<br>完了をTopicへ通知|
|6|ロボット座標|TFから取得|
|7|TFフレームの指定|1〜4,6の処理では、基準フレーム名、該フレーム名、SendするTransform(bTc,mTc)をArgumentで指定|
|8|結果保存|1の結果をrqt_param_managerの保存機能にて所定.yamlファイルに保存|

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
roslaunch rovi_utils rcalib.launch attach:=Tf_to_attach saveto:=file_name
~~~

"saveto:="は結果を書き込むパラメータファイル(yaml)です。このファイルには予め保存したいパラメータ名が存在しないと書き込まれないので注意のこと。例えばbTc(固定カメラのワールドからの座標)を保存たいときは、以下のような内容のyamlファイルを用意しておく。
このファイルは再起動時TFを復旧するにも必要です。
~~~
rcalib:
 translation:
   x: 0
   y: 0
   z: 0
 rotation:
   x: 0
   y: 0
   z: 0
   w: 0
~~~
"robot"のところは"robot:="で指定した名前となります。  
以降も同様です。

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

