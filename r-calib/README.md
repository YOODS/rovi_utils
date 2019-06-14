# Robot Calibration

## Preparation

1. VISPスタックをインストール
https://visp.inria.fr/

~~~
sudo apt-get install ros-kinetic-visp
sudo apt-get install ros-kinetic-visp-hand2eye-calibration
~~~

2. config_tf.yamlの編集  
ロボットドライバから発行されるTFに合わせて、フレーム名を編集します。cameraとflangeフレームが必須です。  
このファイルは、本ユーテリティを起動するとパネルから変更可能です。パネルから変更した場合は「保存」ボタンを押した後、一旦ユーテリティを終了して再起動しなければなりません。  
またこのファイルはmaster_teachなど他のアプリケーションでも共用する重要なファイルですので、フレーム名など間違いがないことを確認してください。  
ファイルロケーションはrovi_utils直下です。  
**注意**)固定カメラを使うときは"カメラマウントフレームID"(/config_tf/camera/parent_frame_id)をworldにします。ソフトウェアはcameraがworldの直下にある場合を固定カメラと判断しているので、この間に中間的なフレームを介さないようにしてください。

3. param.yamlの編集  
使用するキャリブ板に合わせてparam.yamlを編集します。このファイルもユーティリティ起動後はパネルから変更可能です。　 
ファイルロケーションはrovi_utils/r-calibです。

4. キャリブ板  
こちらから[キャリブ板](gridboard.pdf)を印刷し、平らな板に貼り付けてキャリブ板を作ります。

## Launch

1. [RoVIの起動](https://github.com/YOODS/rovi#%E8%B5%B7%E5%8B%95)

2. ロボットドライバ起動

3. ロボットキャリブレーションユーティリティの起動

~~~
roslaunch rovi_utils rcalib.launch
~~~

4. 確認  
~~~
rosrun rqt_tf_tree rqt_tf_tree
~~~
などでTF構成を確認します。正しく設定されていれば下図のような構成になります(J5,J6はロボットドライバによって異なります)。  

![tf tree](frames.png)

## Operation  
ロボットキャリブレーションは、この図の"camera"フレームとそのベースフレーム(図中"J5"フレーム)の座標変換を求める、ことです。  
次の手順にて行います。

1. 撮影調整  
床などに置いたキャリブ板を様々な視点から撮影することで、キャリブレーションのデータを収集します。キャリブ板を認識すると、下図のようにハイライトされます。  
この状態が保たれるよう、*R-Calibパネル*の"ライブ調整”や照明を調整します。  
![fig1](fig1.png)
2. 視点移動  
黄色枠にキャリブ板が合うようにロボット姿勢を動かします。
またこのとき*R-Calibパネル*の"再投影誤差"が*0.2*以下であることを確認します。  
![fig3](fig2.png)
3. 取り込み  
最初はR-Calibパネルの"カウント”が０であることを確認します。０でなければ「カウントリセット」を押して０にします。  
「取り込み」ボタンを押すことで、データ(ロボット座標とキャリブ板座標)が取り込まれ、黄色枠は次のロボット姿勢の指示に変わります。2&rarr;3を20箇所程度繰り返します。
4. 解析  
取り込んだデータを解析します。結果の妥当性は、R-Calibパネルの"結果”&rarr;
"誤差"にて判断します。誤差が1mm(0.001m)以下であれば妥当です。  
やり直す場合は、「カウントリセット」して収集したデータを破棄してから
5. 保存  
解析結果のTransformは*ConfigTFパネル*の"カメラ/カメラマウント変換"に表示されています。誤差が妥当であれば「保存」ボタンを押してconfig_tf.yamlに書き込みます。
6. 利用  
このキャリブレーション結果を他のアプリケーションで利用するときには、そのアプリケーションのlaunchの最初でconfig_tfを起動するようにします。以下に例を示します。
~~~
  <rosparam command="load" file="$(find rovi_utils)/config_tf.yaml" />
  <node pkg="rovi_utils" type="config_tf.py" name="config_tf" />
~~~


## Appendix

launch後のTopicは以下のようにアサインされる。

1. To subscribe

|name|type|description|
|:----|:----|:----|
|/rovi/left/image_rect|Image|基準カメラ(左)のレクティファイ画像|
|/request/clear|Bool|取得したデータをクリアする。|
|/request/capture|Bool|ロボットとキャリブ板の座標をTFから得て、バッファにストアする。|
|/request/solve|Bool|ストアされたデータから、カメラマウントからカメラへの座標変換を算出し、パラメータに書き込む|

2. To publish

|name|type|description|
|:----|:----|:----|
|/gridboard/image_out|Image|キャリブ板の認識結果|
|/response/cleare|Bool|clear処理完了でアサートされます|
|/response/capture|Bool|capture処理完了でアサートされます|
|/response/solve|Bool|solve処理完了でアサートされます|
