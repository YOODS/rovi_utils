# ロボットキャリブレーション

<img src="a-calib.png" />  

## 目的



## 記号定義
<dd>座標変換行列:$${}^{A}T_{B}$$は、基準座標系をAとしたときの、座標系Bの変換行列
<dd>座標変換行列Tは4&times;4の正方行列
<img src="https://latex.codecogs.com/gif.latex?T=\left[\begin{array}{ccc|c}&&&\\
&\smash{\huge{R}}&&\smash{\huge{S}}\\
&&&\\ \hline
0&0&0&1
\end{array}\right]" />
<dd>3&times;3の部分行列Rは座標系の回転を表す
<dd>3&times;1の部分行列<img src="https://latex.codecogs.com/gif.latex?Q=\left[\begin{array}{rrr} q_x \\ q_y \\ q_z \end{array} \right]" />は座標系原点の平行移動を表す
<dd>回転行列は、xyz回転表記とする(回転順序がX軸&rarr;Y軸&rarr;Z軸)
<dd>
</dl>
<dl>
<dt>例題
<dd>1)座標系Bにおける点Pの座標<img src="https://latex.codecogs.com/gif.latex?{}^{B}P=\left[\begin{array}{rrr} x \\ y \\ z \\ 1 \\ \end{array} \right]" />
を座標系Aでの座標に変換する<br>
<img src="https://latex.codecogs.com/gif.latex?{}^{A}P={}^{A}T_{B}{}^{B}P" />
<dd>2)座標系Bにおける剛体MのPose(座標+姿勢)<img src="https://latex.codecogs.com/gif.latex?{}^{B}M=\left[
\begin{array}{ccc|c}
&&&\\
&\smash{\huge{R_M}}&&\smash{\huge{Q_M}}\\
&&&\\ \hline
0&0&0&1
\end{array}
\right]" />のとき、これを座標系Aに変換する<br>
<img src="https://latex.codecogs.com/gif.latex?{}^{A}M={}^{A}T_{B}{}^{B}M" />
</dl>
