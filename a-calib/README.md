# ロボット(アーム)キャリブレーション

<img src="a-calib.png" />

## 概要
1. ロボット(カメラ)キャリブレーションと同様の手順にて、ロボット(アーム)のキャリブレーションを行うパッケージがある http://wiki.ros.org/robot_calibration
2. 推定される誤差は関節角の原点オフセットである(**関節原点**として試験成績書などに記録があると思います)
3. robot_calibrationパッケージを使うには、少なくとも**URDF**が必要
4. その他、キャリブレーションボードが限定されるような(実施経験者を探している)
5. 算出された誤差をロボットに反映させる方法は、ロボットメーカに問い合わせが必要

---

### URDF  
ロボットのリンク機構を記述したテキストファイル。Fanucのロボットは ROS-Industrial
 http://wiki.ros.org/fanuc がサポートしているようであるが、正しいかどうかは不明(Fanucは関知していない)。  

---

### アームの誤差  
ベース座標系基準のメカニカルインタフェース座標系の座標変換を<i><sup>b</sup>T<sub>m</sub></i>とする。 <i><sup>b</sup>T<sub>m</sub></i>はロボットコントローラより<i>(x,y,z,w,p,r)</i>などの表記にて得られる位置変数から一意的に求められる。 <i><sup>b</sup>T<sub>m</sub></i>はロボットコントローラの内部ではリンク機構の幾何学モデルより

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}=f(\theta_1,...,\theta_6)" />---(1)

のような式で関節の回転角から算出できる。**ROSではロボットのURDF定義からTfによってリアルタイムに求められる。**  
ただし式(1)は誤差を含まない理想モデルのため、現実には様々な誤差がTfに含まれる。主な誤差要因は
1. 関節の原点オフセット
2. リンク機構の製作精度(ジョイント間Tfの誤差)

である。そこで式(1)これらの誤差を含んだ式に書き換えると

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}=f(\theta_1+\Delta\theta_1,...,\theta_6+\Delta\theta_6)+\Delta{f}(\theta_1,...,\theta_6)" />---(2)

右辺第1項が誤差要因1、第2項が要因2、による誤差を示す。ロボット(アーム)キャリブレーションの目的は、ロボット(カメラ)キャリブレーションと同様の手順により、最適化計算によって、これらの誤差(&Delta;&theta;,&Delta;f)を推定することである。  
実際には、双方(カメラとアーム)の未定定数12個が推定される。

---
---

## (備考)記号定義
- 座標変換行列:<img src="https://latex.codecogs.com/gif.latex?{}^{A}T_{B}" />は、基準座標系をAとしたときの、座標系Bの変換行列
- 座標変換行列Tは4&times;4の正方行列

<img align="center" src="https://latex.codecogs.com/gif.latex?T=\left[\begin{array}{ccc|c}&&&\\&\smash{\huge{R}}&&\smash{\huge{S}}\\&&&\\\hline0&0&0&1\end{array}\right]" />

- 3&times;3の部分行列Rは回転行列
- 3&times;1の部分行列Sは座標原点の平行移動

<img align="center" src="https://latex.codecogs.com/gif.latex?S=\left[\begin{array}{rrr}x\\y\\z\end{array}\right]" />

- 例題(1) 座標系Bにおける点Pの座標<i><sup>B</sup>P</i>を座標系Aでの座標に変換する

<img src="https://latex.codecogs.com/gif.latex?{}^{A}P={}^{A}T_{B}\cdot{}^{B}P" />

- 例題(2) 座標系Bにおける剛体MのPose(座標+姿勢)<i><sup>B</sup>M</i>を座標系Aに変換する

<img src="https://latex.codecogs.com/gif.latex?{}^{A}M={}^{A}T_{B}\cdot{}^{B}M" />
