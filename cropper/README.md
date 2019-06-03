# Cropper

### 要件  
- 入力点群を単一座標系に合成し出力する
- 入力点群のクロッピング処理(後述)を行う
- クロッピングパラメータはそのシーンのカメラ座標系を基準とする
- 撮像と同時に、そのときのカメラ位置、またはロボット位置を、保存する(TFへ)

## Parameter
### ~config

|arg|type|description|
|:----|:----|:----|
|capture_frame_id|string|撮像時に生成するフレーム(座標系)名。カメラまたはワークがロボットと共に移動する装置構成では、撮像データの座標系を揃える必要があります。このために撮像時のロボット座標を保存し、これをスタティックなフレームとしてTFに追加します。以下の例を参考|
|source_frame_id|string|撮像点群のフレーム(座標系)名。このフレーム名はcapture_frame_idにて指定したフレーム名から生成されるものを含みます。|
|frame_id|string|出力点群のフレーム(座標系)名。|
|relay|string|captureをsubscribeしたとき発行するトピック名。これは上位からの撮像要求をセンサー等のドライバ層へリレーするために使います。|

#### capture_frame_idフレーム名の取り扱い  
capture_frame_idパラメータで指定したフレーム名は、撮像と同時に以下の命名規則に従って**world**に追加されます。
~~~
"capture_frame_idパラメータ"/capture"撮像数"
~~~
例えば、capture_frame_idパラメータが"camera"で３回目の撮像に対して、以下のフレームIDが**world**に追加されます。
~~~
camera/capture3
~~~
#### 撮像データの座標変換規則  
撮像データは、source_frame_idからframe_idへ座標変換されますが、これは次のような規則で行わえます・・・


### ~

|arg|type|description|
|:----|:----|:----|
|cropZ|Z-Crop距離|
|cropR|円柱Crop半径|
|mesh|Voxelサイズ|

## Topic
### to subscribe
|Topic|タイプ|説明|
|:----|:----|:----|
|~clear|Bool|点群データをクリアします。撮像座標を保存しているフレームをTFから削除します。|
|~capture|Bool|現在のkeepフレームをTFに保存し、ドライバ層へ撮像要求を再発行します(configパラメータ”relay”で指定したトピック)|
|~setcrop|Bool|Cropパラメータ初期化<br>50%の点群を含む範囲で初期化|
|~param|String|cropperパラメータを出力に反映。パラメータ変更はparam_watcherから通知|
|~in/floats|Floats|シーン点群データ。取り込みと同時にVoxel(down sampling)処理されます。|

### to publish  
|Topic|タイプ|説明|
|:----|:----|:----|
|~cleared|Bool|clear完了で常にTrueを発行します。|
|~captured|Bool|capture完了にて撮像の成否を真偽値として発行します。|
|~raw/floats|Floats|Crop前の合成点群。点群はconfigパラメータ"output"で指定された座標系に変換されます。|
|~out/floats|Floats|Crop後の合成点群|

### to TF
|フレーム名|基準フレーム|説明|
|:----|:----|:----|
|$keep/capture$N|world|N回目撮像時に保存された座標|
