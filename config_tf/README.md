# config_tf

## カメラの設置手順  
カメラがどこに設置されているか、また設置されている座標(ロボットキャリブレーション結果)をTFへ送るユーティリティがconfig_tfです。  
### 起動
~~~
roslaunch rovi_utils config_tf.launch
~~~
以下のパネルより装置の構成を設定します。
<img src="img/panel.png" />

|ラベル|説明|パラメータ名|
|:----|:----|:----|
|フランジフレーム名|ロボットのメカニカルインタフェースのフレーム名|/config_tf/alias/flange|  
|カメラマウントフレーム名|カメラが取り付けられているフレーム名。固定カメラであれば"world"となります|/config_tf/alias/mount|
|カメラマウント/カメラ変換|いわゆるロボットキャリブレーションの値|/config_tf/camera/transform|
「保存」を押すとconfig_tf.yamlファイルに書き込まれます。内容を変更した場合は再起動が必要です。  
この設定にてconfig_tfを起動したときのTF_TREEは下図のようになります。
<img src="img/frames.png" /><br clear="all" />
次のロボットキャリブレーションを含め、**ロボットとRoVIを連携する場合は、必ず先にconfig_tfを起動しておきます**
