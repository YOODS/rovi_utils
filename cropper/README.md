## Cropper

- デフォルト名前空間：/cropper  ※以下のTopic、Parameterで先頭に"/"がついていないものは、/cropper/が付いているものとみなす。
- camera座標系にて処理
- 合成時は指定された出力座標系に揃える
- subscribeにてシーン点群受信時には、その時点のロボット座標とシーン点群をファイルで保存し、全てのシーン点群を合成した点群
  （クロップ前）をpublishする。
  zcropパラメータが設定されている場合には、全てのシーン点群をクロップ・合成し、一枚目撮影時のロボット座標をpublishし、
  合成した点群（指定された座標系に変換したもの）をpublishする。
- subscribeにてパラメータ変更通知を受信した際にも、全てのシーン点群をクロップ・合成し、一枚目撮影時のロボット座標をpublishし、
  合成した点群（指定された座標系に変換したもの）をpublishする。
- Cropping領域は***それぞれの***点群のカメラ座標系で与える

### 処理の定義  
|名前|説明|PublishされるTopic|
|:----|:----|:----|
|Crop|入力点群(Crop処理前点群)から、そのシーンのカメラ座標系で与えられた領域外の点を削除した新たな点群(Crop処理後点群)を作る。Crop処理前点群およびCrop処理後点群はそれぞれ配列にappendする||
|合成|Crop処理前およびCrop処理後の点群配列から、***出力座標系<sup>(1)</sup>に合成***したそれぞれの新たな点群を作る。|scene/floats<br>cropped_scene/floats<br>source/tf|
|Clear|点群配列をクリアする|scene/floats<br>cropped_scene/floats|

### 出力座標系  
カメラとワークの状態により4つの組み合わせがある。それぞれの合成点群の出力座標系については下記のように定める（出力座標系は、後段のFinderでの変換がなるべく小さくなるように定めているつもり）。

|#|カメラ|ワーク|出力座標系|
|:----|:----|:----|:----|
|1|固定|固定|カメラ|
|2|ロボット|固定|カメラ|
|3|固定|ロボット|ロボット|
|4|ロボット|ロボット|ロボット|
| 5    | 固定 | 固定 | ロボット   |
| 6    | ロボット | 固定 | ロボット   |

### Topics(to subscribe)  
|Topic|タイプ|説明|
|:----|:----|:----|
|/clear|Bool|Clear処理。|
|/robot/tf|Transform|ロボットからロボット座標を受け取る。|
|/robot/euler|Transform|ロボット座標を受け取る（オフライン用 rqtから呼ばれる）。|
|dummy_euler|Transform|ロボット座標を受け取る（オフライン用 rqt_param_managerから呼ばれる）。|
|/rovi/ps_floats|Floats|シーン点群データ。受信した点群データを**Crop処理**する。その後**合成処理**する。<br>Topic名はrosparamの　source に設定されているものを使う。（デフォルト：/rovi/ps_floats）|
|zcrop/set|Bool|Z-Cropパラメータ初期化。<br>Cropされた点群重心からカメラ座標系Z方向66%点を含む範囲。|
|sphere/set|Bool|球Cropパラメータ初期化。<br>Cropされた点群重心から66%点を含む範囲。|
|param|string|パラメータ変更通知(zcrop,sphare)。**変更あるとCrop前点群配列のCopyを作り、Crop前点群およびCrop後点群配列共初期化する。次にCopy配列の全要素に対してCrop処理を行い、最後に合成処理を行う**。|
|clear_param|Bool|パラメータクリア通知(zcrop,sphere,voxel)。（オフライン用 rqtから呼ばれる）|

### Topics(to publish)  
|Topic|タイプ|説明|
|:----|:----|:----|
|source/tf|Transform|シーン生成時ロボット座標(合成時は1枚目の位置)。シーン点群を受信する毎にpublishsされる。masterteachに通知する。|
|org_scene/floats|Floats|シーン点群（現状特に使っていないので削除するかも）|
|scene/floats|Floats|Crop前の合成点群(カメラ座標)。rvizで使用する。|
|cropped_scene/floats|Floats|Crop&Voxel後の合成点群(出力座標系で指定された座標)。rviz、masterteachに通知する。|
|/result_ps|String|phase shift及びクロップの結果通知。masterteachに通知する。|
|/solver/mts|Transform| socketからtfを受信した際の応答(待ちをいれるだけで値を使うことはない)。|
|/solver/message|String| 処理結果メッセージ。Industrial Robot I/F,rqt_param_managerに通知する。|
|/solver/error|String| 処理結果コード。Industrial Robot I/F,rqt_param_managerに通知する。|




### Parameters(this)
|Parameter|説明|
|:----|:----|
|zcrop|Z-Crop座標(near,far)<br>zcrop/setにて初期化|
|sphere|球Crop座標(x,y,r)<br />sphere/setにて初期化<br />※x,y平面の円クロップ|
|voxel|Voxelサイズ|
|frame_id|出力座標系名|


### Parameters(ref.)
|Parameter|説明|
|:----|:----|
|/rovi/camera/mTc|キャリブレーション|
|source|シーン点群データTopic名|
