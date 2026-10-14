# 人体3姿勢マッピングとIKの計算式

作成日：2026-09-30

この資料は、本プロジェクトで作成したコードに基づいて、人体の手の位置・向きからロボットの関節指令を生成する計算を整理したものである。保存時点ではコマンド実行環境のエラーによりファイルの再読込・コンパイル確認ができていないため、この会話で反映した実装を基準としている。

## 1. 処理の全体像

```text
mocopiの右手位置・向き
    ↓ 入力座標の変換
登録した上・前・右横との対応から手先目標を生成
    ↓
目標位置・姿勢と現在の手先姿勢の誤差を計算
    ↓
減衰最小二乗法（DLS）で関節角を反復更新
    ↓ 関節制限・到達精度の確認・指令速度制限
ArticulationBody.xDrive.targetへ適用
```

「人体から手先目標を作るマッピング」と「手先目標から関節角を求める逆運動学（IK）」は別の処理である。

対象コード：

- `Assets/Scripts/HandPoseIkController.References.cs`：指定3姿勢からの目標生成。
- `Assets/Scripts/HandPoseIkController.Workspace.cs`：姿勢登録・開始条件・範囲校正の管理。
- `Assets/Scripts/ArmPoseSolver.cs`：順運動学・ヤコビ行列・DLS反復計算。
- `Assets/Scripts/HandPoseIkController.cs`：入力処理・追従・関節指令の適用。

以下の3姿勢マッピングは、mocopiの対応づけを有効にした場合を対象とする。マーカーテストはマーカーの位置・向きを直接IK目標とし、3姿勢マッピングを通らない。

## 2. 記号と単位

| 記号 | 意味 | 単位 |
|---|---|---|
| $\mathbf h$ | 操作座標に変換した現在の人の手の位置 | m（Unityワールドのスケールを前提） |
| $\mathbf h_0,\mathbf h_1,\mathbf h_2$ | 登録した上・前・右横の手の位置 | m |
| $\mathbf r_0,\mathbf r_1,\mathbf r_2$ | 対応するロボットの手先位置 | m |
| $\mathbf p^*,Q^*$ | ロボット手先の目標位置・姿勢 | m、Quaternion |
| $\mathbf q$ | 6関節の角度ベクトル | IK内部ではrad |
| $\mathbf p(\mathbf q),Q(\mathbf q)$ | 順運動学で求める手先位置・姿勢 | m、Quaternion |
| $\mathbf J$ | 重み付きの幾何ヤコビ行列 | $6\times6$ |
| $\lambda$ | DLSの減衰係数 | 実装係数 |

Inspectorと関節指令は度を使い、IK内部ではラジアンを使う。

## 3. ロボット側の指定基準姿勢

関節角の並びはJ1〜J6である。

| 人の登録姿勢 | ロボットの基準関節角（度） |
|---|---|
| 上 | $\mathbf q_0=(0,0,0,90,90,0)$ |
| 前 | $\mathbf q_1=(0,-90,0,90,90,0)$ |
| 右横 | $\mathbf q_2=(-90,-90,0,90,90,0)$ |

指定関節角を順運動学へ入力し、各手先位置・姿勢を得る。

$$
(\mathbf r_i,Q_{r,i})=FK(\mathbf q_i),\qquad i\in\{0,1,2\}
$$

指定関節角が関節制限を超える場合、黙って丸めず基準設定を拒否する。

これらは対応づけ用の基準姿勢であり、ロボットの最大可動域そのものではない。工具点は`Tool Center Point`を指定していればその点、未指定ならJ6リンクの基準点を使用する。

## 4. 人体入力の座標変換

計測開始時の手の位置を $\mathbf p_{h,\mathrm{ref}}$、現在の手のワールド位置を $\mathbf p_h$ とする。計測開始時のAnimatorのyawを除去する回転を $C$ として、

$$
\mathbf h=C\left(\mathbf p_h-\mathbf p_{h,\mathrm{ref}}\right)
$$

とする。操作座標のXは右、Yは上、Zは前を想定し、ロボット側のUnityワールド座標へ対応させる。これはロボットベースへ自動整列する座標変換ではなく、シーンの配置・アバターの軸設定に依存する。

上・前・右横も同じ固定座標で登録する。追従開始直前に選ぶneutralとは別に、マッピング用の計測基準位置を保持する。

## 5. 3姿勢から目標位置を作る

### 5.1 人の3点が作る平面へ分解する

$$
\mathbf u=\mathbf h_1-\mathbf h_0,\qquad
\mathbf v=\mathbf h_2-\mathbf h_0,\qquad
\mathbf d=\mathbf h-\mathbf h_0
$$

平面の単位法線は、

$$
\mathbf n_h=\frac{\mathbf u\times\mathbf v}{\|\mathbf u\times\mathbf v\|}
$$

である。現在位置を、

$$
\mathbf d=a\mathbf u+b\mathbf v+d_n\mathbf n_h
$$

と分解する。平面内の係数は、次の2元連立方程式から求める。

$$
\begin{bmatrix}
\mathbf u\cdot\mathbf u & \mathbf u\cdot\mathbf v\\
\mathbf u\cdot\mathbf v & \mathbf v\cdot\mathbf v
\end{bmatrix}
\begin{bmatrix}a\\b\end{bmatrix}
=
\begin{bmatrix}\mathbf u\cdot\mathbf d\\\mathbf v\cdot\mathbf d\end{bmatrix}
$$

実装は行列式を使って直接解く。

$$
D=(\mathbf u\cdot\mathbf u)(\mathbf v\cdot\mathbf v)-(\mathbf u\cdot\mathbf v)^2
$$

$$
a=\frac{(\mathbf d\cdot\mathbf u)(\mathbf v\cdot\mathbf v)-(\mathbf d\cdot\mathbf v)(\mathbf u\cdot\mathbf v)}{D}
$$

$$
b=\frac{(\mathbf d\cdot\mathbf v)(\mathbf u\cdot\mathbf u)-(\mathbf d\cdot\mathbf u)(\mathbf u\cdot\mathbf v)}{D}
$$

$$
d_n=\mathbf d\cdot\mathbf n_h
$$

ほぼ一直線の3点では安定に計算できないため、登録点の距離と方向の分離を検査する。

### 5.2 ロボット側へ変換する

$$
\mathbf u_r=\mathbf r_1-\mathbf r_0,\qquad
\mathbf v_r=\mathbf r_2-\mathbf r_0
$$

$$
\mathbf n_r=\frac{\mathbf u_r\times\mathbf v_r}{\|\mathbf u_r\times\mathbf v_r\|}
$$

平面外方向の倍率を、

$$
s=\frac{\|\mathbf u_r\|+\|\mathbf v_r\|}{\|\mathbf u\|+\|\mathbf v\|}
$$

とすると、手先目標位置は、

$$
\boxed{\mathbf p^*=\mathbf r_0+a\mathbf u_r+b\mathbf v_r+s\,d_n\mathbf n_r}
$$

となる。登録した3点では、

$$
\mathbf h_0\mapsto\mathbf r_0,\quad
\mathbf h_1\mapsto\mathbf r_1,\quad
\mathbf h_2\mapsto\mathbf r_2
$$

が成り立つ。

平面内はアフィン変換である。平面外を辺長比で換算する部分は設計上の仮定であり、3姿勢から計測された全方向の最大範囲ではない。三角形の外では位置を外挿するため、到達可能性は後段のIKでも確認する。

## 6. 目標姿勢の生成

### 6.1 基準姿勢を補間する

位置計算から得られる重みは、

$$
(w_0,w_1,w_2)=(1-a-b,a,b)
$$

である。姿勢補間では負の重みを0にし、正規化する。

$$
\hat w_i=\frac{\max(0,w_i)}{\sum_{j=0}^{2}\max(0,w_j)}
$$

Quaternionの補間は逐次Slerpで実装している。$t=\hat w_0+\hat w_1>0$ の場合、

$$
Q_{01}=\operatorname{Slerp}(Q_0,Q_1,\hat w_1/t)
$$

$$
\bar Q=\operatorname{Slerp}(Q_{01},Q_2,\hat w_2)
$$

とする。$t=0$ の場合は $\bar Q=Q_2$。これは補間順序を持つ方法であり、一般的な対称Quaternion平均とは異なる。

ロボットの登録姿勢から $\bar Q_r$、人の登録時の手の向きから $\bar Q_h$ をそれぞれ求める。

### 6.2 現在の手の回転差を反映する

現在の人の手のQuaternionを $Q_h$ として、

$$
\boxed{Q^*=C\left(Q_h\bar Q_h^{-1}\right)C^{-1}\bar Q_r}
$$

とする。人の登録時と同じ位置・向きなら、対応するロボット基準の向きが目標になる。基準に対して追加で手を回した分も反映する。

向き追従OFFでは、この計算を使わず校正時の工具姿勢 $Q_{\mathrm{tool,ref}}$ を固定目標とする。その場合は、指定3姿勢の向きまで一致させるものではない。

## 7. 順運動学の実装

数値IKでは、現在のArticulationのアンカー位置・軸・関節角を取得し、仮想的に関節角の差分を適用する。

関節 $i$ の基準角を $q_{i,\mathrm{ref}}$、候補角を $q_i$、現在の仮想回転軸を $\mathbf z_i$、中心を $\mathbf o_i$ とする。

$$
R_i=\operatorname{Rot}\left(\mathbf z_i,q_i-q_{i,\mathrm{ref}}\right)
$$

手先位置・向きを、

$$
\mathbf p' = \mathbf o_i+R_i(\mathbf p-\mathbf o_i),\qquad Q'=R_iQ
$$

で更新する。同時に、それより先の関節中心と回転軸も更新する。これをJ1からJ6まで行う。

計算中に実際のリンクTransformを試行的に移動させることはない。

## 8. 位置・姿勢誤差

位置誤差は、

$$
\mathbf e_p=\mathbf p^*-\mathbf p(\mathbf q)
$$

である。姿勢誤差は、

$$
Q_{\mathrm{err}}=Q^*Q(\mathbf q)^{-1}
$$

を回転軸 $\mathbf a$ と回転角 $\theta$ に変換して、

$$
\mathbf e_R=\theta\mathbf a
$$

とする。$\theta$ はradで、短い方の回転を採用する。Quaternionの符号をそろえてから軸角へ変換し、微小角ではゼロとする。

位置と姿勢をまとめた重み付き誤差は、

$$
\boxed{\mathbf e=\begin{bmatrix}\mathbf e_p\\\ell\mathbf e_R\end{bmatrix}},\qquad \ell=0.15
$$

である。$\ell$ は姿勢誤差を位置誤差と組み合わせるための長さスケールに相当する。0.15は実装で選んだ係数であり、最適性を実験で確認した値ではない。

## 9. ヤコビ行列

すべての対象関節は回転関節である。関節 $i$ の中心 $\mathbf o_i$ と単位回転軸 $\mathbf z_i$ を使い、ヤコビ行列の第 $i$ 列を、

$$
\mathbf J_i=\begin{bmatrix}
\mathbf z_i\times(\mathbf p-\mathbf o_i)\\
\ell\mathbf z_i
\end{bmatrix}
$$

とする。

$$
\mathbf J=[\mathbf J_1\ \mathbf J_2\ \cdots\ \mathbf J_6]
$$

は $6\times6$ 行列で、微小な関節変化と手先変化の局所的な関係を表す。

## 10. 減衰最小二乗法によるIK

関節角の修正量 $\Delta\mathbf q$ は、次の問題に対応するDLS式で求める。

$$
\min_{\Delta\mathbf q}\left\|\mathbf J\Delta\mathbf q-\mathbf e\right\|^2
+\lambda^2\left\|\Delta\mathbf q\right\|^2
$$

$$
\boxed{
\Delta\mathbf q=\mathbf J^{\mathsf T}
\left(\mathbf J\mathbf J^{\mathsf T}+\lambda^2\mathbf I\right)^{-1}\mathbf e
},\qquad\lambda=0.025
$$

実装は逆行列を明示的に作らず、

$$
(\mathbf J\mathbf J^{\mathsf T}+\lambda^2\mathbf I)\mathbf y=\mathbf e
$$

を部分ピボット付きの消去法で解き、$\Delta\mathbf q=\mathbf J^{\mathsf T}\mathbf y$ とする。

減衰項は特異姿勢付近での過大な修正を抑える。ただし、全目標への収束、障害物回避、最適なIK解の選択を保証しない。

## 11. 関節制限とステップ選択

各成分をまず制限する。

$$
\Delta\bar q_i=\operatorname{clamp}(\Delta q_i,-0.15,0.15)
$$

候補角を、

$$
q_{i,\mathrm{candidate}}=
\operatorname{clamp}\left(q_i+\alpha\Delta\bar q_i,q_{i,\min},q_{i,\max}\right)
$$

で作る。$\alpha$ は $1,1/2,1/4,1/8,1/16$ の順で試す。

$$
E(\mathbf q)=\|\mathbf e_p\|^2+\ell^2\|\mathbf e_R\|^2
$$

が小さくなる候補だけを採用し、どの候補も改善しなければその反復を終了する。関節制限は制約付き最適化として厳密に解くのではなく、候補のクリップによって適用している。

通常は最大40反復。位置誤差1mm未満かつ姿勢誤差0.015rad未満なら早期終了する。

## 12. 解の検査と再試行

3姿勢対応モードでは、候補関節角の順運動学で、

$$
\|\mathbf p(\mathbf q)-\mathbf p^*\|\leq0.003\ \mathrm{m}
$$

$$
\operatorname{Angle}(Q(\mathbf q),Q^*)\leq2^\circ
$$

を確認する。通常の初期値で満たせなければ、上・前・右横の指定関節角を初期値としてそれぞれ最大70反復で再試行し、条件を満たす最初の解を使う。

解が得られない場合は新しい指令を適用せず、直前の指令を維持する。これは到達不能の厳密な証明ではなく、この数値探索で採用できる解が得られなかったことを意味する。

IKは手先位置・向きを対象としているため、同じ手先姿勢に対して指定基準とは異なる関節角の解になることがある。

## 13. 実際の関節指令

解を直ちに関節へ設定せず、前の指令 $q_{i,\mathrm{cmd}}$ からの変化速度を制限する。度単位で、

$$
q_{i,\mathrm{cmd,next}}=
\operatorname{clamp}\left(
q_{i,\mathrm{cmd}}+
\operatorname{clamp}(q_{i,\mathrm{IK}}-q_{i,\mathrm{cmd}},-v_{\max}\Delta t,v_{\max}\Delta t),
q_{i,\min},q_{i,\max}\right)
$$

標準設定は $v_{\max}=45^\circ/\mathrm{s}$。適用先は`ArticulationBody.xDrive.target`である。

これは目標角の変化速度であり、物理的な関節速度の保証ではない。物理シミュレーションではDriveの追従遅れや衝突などの影響があるため、数値IKの誤差と実際の手先誤差は区別する。

## 14. 主要パラメータ

| 項目 | 実装値・動作 |
|---|---|
| 姿勢誤差の重み $\ell$ | 0.15 |
| DLS減衰係数 $\lambda$ | 0.025 |
| 通常の最大反復数 | 40 |
| 基準姿勢から再試行する場合 | 各最大70反復 |
| 1反復の関節修正制限 | ±0.15rad |
| ステップ係数 | 1から1/16まで半減 |
| 内部早期終了 | 位置1mm未満、姿勢0.015rad未満 |
| 3姿勢モードの候補解受入れ | 位置3mm以内、姿勢2°以内 |
| 指令角の変化速度 | 標準45°/秒 |
| Drive stiffness / damping / forceLimit | 10000 / 100 / 1000 |

## 15. neutralの意味と限界

最新仕様では、3姿勢登録後の楽な姿勢をneutralとして保存する。以前の位置5cm・角度10°への復帰条件は使わない。

ただし、指定3点との対応を保つため、neutral設定で位置マッピング全体を移動させることはしない。追従開始時には、現在の手に対応する目標へロボットが動く可能性がある。neutralは比較表示用の基準であり、指定3姿勢を上書きする4つ目の対応点ではない。

この実装で未保証の事項：

- 人とロボットの全方向の最大可動域の一致。
- 全目標でのIK収束と、関節解の連続性。
- 自己衝突・環境との衝突回避。
- mocopi受信途絶の検出。
- 実機の追従精度と停止動作。

研究・評価では、3姿勢マッピングの設計、DLSソルバの精度、Unityの物理追従、実入力の誤差を分けて確認する。
