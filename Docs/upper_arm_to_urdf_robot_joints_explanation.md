# `UpperArmToUrdfRobotJoints.cs` 解説

このドキュメントは、`Assets/Scripts/UpperArmToUrdfRobotJoints.cs` が何をしているか、Unity Inspector で何を設定する必要があるか、動作確認時にどこを見るべきかを説明します。

## 目的

`UpperArmToUrdfRobotJoints` は、Humanoid Avatar の右腕の動きを読み取り、Unity 上の URDF ロボットアームの J1-J6 に対応する `ArticulationBody` を動かすためのスクリプトです。

このスクリプトは直接 myCobot 実機を制御しません。実機への UDP 送信は `MyCobotUdpJointSender.cs` 側に分離されています。

## 全体の処理

大まかな流れは以下です。

1. Humanoid の `Animator` を取得する。
2. `RightUpperArm` / `RightLowerArm` / `RightHand` の Transform を取得する。
3. 右上腕・前腕・手首の向きから角度を計算する。
4. キャリブレーション時の姿勢との差分を取る。
5. 差分を J1-J6 のロボット関節角に変換する。
6. 角度を可動範囲に制限する。
7. スムージングする。
8. 各 `ArticulationBody.xDrive.target` に角度を設定する。

## Inspector 設定項目

### Humanoid

`humanoidAnimator` には、mocopi などで実際に動いている Humanoid Avatar の `Animator` を設定します。

未設定の場合、`Awake()` で同じ GameObject または子オブジェクトから `Animator` を探します。ただし、確実に動かすには Inspector で明示的に設定する方が安全です。

この `Animator` は以下を満たす必要があります。

- `Animator.isHuman` が true
- `RightUpperArm` が存在する
- `RightLowerArm` が存在する
- `RightHand` が存在する

条件を満たさない場合、Console にエラーを出して処理を止めます。

### URDF Robot Joints

`joint1` から `joint6` には、URDF ロボット側の各関節に対応する `ArticulationBody` を設定します。

設定する対象は、見た目のメッシュではなく、実際に動かしたい関節の `ArticulationBody` です。

このスクリプトが動かせる関節タイプは以下です。

- `RevoluteJoint`
- `PrismaticJoint`

それ以外の関節タイプが設定されている場合、警告を1回だけ出して、その関節は動かしません。

### Robot Base Angles

`robotBaseJ1` から `robotBaseJ6` は、キャリブレーション時のロボット側の基準角です。

人間の基準姿勢を取ったとき、ロボットをどの角度から開始したいかをここで指定します。

### Mapping

各関節には以下の調整値があります。

- `jNScale`: 人間側の角度変化をロボット側へ何倍で反映するか
- `jNOffset`: 変換後の角度に加える固定オフセット
- `invertJN`: 回転方向を反転するか

例えば、人間が腕を右へ動かしたときにロボットの J1 が逆方向へ動く場合は、`invertJ1` を有効にします。

動きが大きすぎる場合は `jNScale` を小さくします。基準位置がずれている場合は `jNOffset` を調整します。

### Joint Limits

`jNMin` / `jNMax` は、スクリプト内部で計算したロボット関節角の上限・下限です。

計算結果はこの範囲に `Mathf.Clamp()` されます。ロボットの安全な可動範囲より広くしない方が安全です。

J1-J5 は初期値で `-165` から `165` 度、J6 は `-180` から `180` 度です。

### Stability

`yawUpdateThreshold` は、腕がほぼ垂直になったときの yaw の暴れを抑えるための閾値です。

上腕や前腕の水平成分が小さすぎる場合、yaw を再計算せず、前回値を使います。これにより、腕を真上・真下に近づけたときの角度ジャンプを避けます。

`smoothing` はロボット関節角の補間率です。

- `0` に近いほど動きが遅く滑らか
- `1` に近いほど入力に即追従

### Articulation Drive

`applyDriveSettings` が有効な場合、各関節の `xDrive` に以下を設定します。

- `driveStiffness`
- `driveDamping`
- `driveForceLimit`

`Start()` 時と、毎フレームの `SetJointTarget()` 時に反映されます。

`logJointDiagnosticsOnStart` が有効な場合、開始時に各関節の名前、関節タイプ、Drive の target、limit、stiffness、damping、forceLimit を Console に出します。

### Calibration

`autoCalibrateOnStart` が有効な場合、Play 開始後 `autoCalibrateDelay` 秒待ってから自動で `Calibrate()` を実行します。

`driveOnlyAfterCalibration` が有効な場合、キャリブレーションが終わるまでロボット関節を動かしません。

キャリブレーションでは、現在の右腕姿勢を基準値として保存します。

- `baseYaw`
- `basePitch`
- `baseElbow`
- `baseForearmYaw`
- `baseHandPitch`
- `baseHandRoll`

以後は、現在角度と基準角度の差分をロボットへ反映します。

### Debug

`showDebugGui` が有効な場合、Play 画面左上に現在の角度情報が表示されます。

`showDebugLog` が有効な場合、`debugLogInterval` 秒ごとに Console へ詳細ログを出します。

## 角度計算の内容

### J1: 上腕の水平 yaw

`RightUpperArm` から `RightLowerArm` へのベクトルを `armDir` として計算します。

```csharp
armDir = lowerArm.position - upperArm.position;
armDir.Normalize();
```

その X-Z 平面上の向きから yaw を計算します。

```csharp
rawYaw = Mathf.Atan2(armDir.x, armDir.z) * Mathf.Rad2Deg;
```

この yaw のキャリブレーション差分が J1 に使われます。

### J2: 上腕の pitch

上腕ベクトルの水平成分と Y 成分から pitch を計算します。

```csharp
float horizontalLength = new Vector2(armDir.x, armDir.z).magnitude;
rawPitch = Mathf.Atan2(armDir.y, horizontalLength) * Mathf.Rad2Deg;
```

この pitch のキャリブレーション差分が J2 に使われます。

### J3: 肘角度

`RightLowerArm` から `RightHand` へのベクトルを `forearmDir` として計算します。

```csharp
forearmDir = rightHand.position - lowerArm.position;
forearmDir.Normalize();
```

上腕 yaw に沿う垂直平面へ上腕・前腕方向を投影し、その平面上の符号付き角度を計算します。
これにより、腕全体を左右へ振ったときの yaw 成分が J3 の肘曲げ角へ混ざりにくくなります。

```csharp
Vector3 yawForward = Quaternion.AngleAxis(rawYaw, Vector3.up) * Vector3.forward;
Vector3 yawRight = Vector3.Cross(Vector3.up, yawForward).normalized;
Vector3 planarArmDir = ProjectDirectionOnPlane(armDir, yawRight, armDir);
Vector3 planarForearmDir = ProjectDirectionOnPlane(forearmDir, yawRight, forearmDir);
rawElbow = Vector3.SignedAngle(planarArmDir, planarForearmDir, yawRight);
```

この肘角度のキャリブレーション差分が J3 に使われます。

### J4: 前腕 yaw

前腕ベクトルの X-Z 平面上のワールド yaw を計算したうえで、上腕 yaw との差分を取ります。
J4 には前腕の絶対方位ではなく、上腕に対する前腕の相対 yaw を使います。

```csharp
rawForearmWorldYaw = Mathf.Atan2(forearmDir.x, forearmDir.z) * Mathf.Rad2Deg;
rawForearmYaw = Mathf.DeltaAngle(rawYaw, rawForearmWorldYaw);
```

この前腕 yaw のキャリブレーション差分が J4 に使われます。

### J5 / J6: 手首 pitch / roll

前腕 yaw だけを取り除いた右手回転から Euler 角を取り、X を pitch、Z を roll として扱います。
右手のワールド Euler 角をそのまま使うと、腕を左右へ振っただけでも pitch/roll の分解結果が変わるためです。

```csharp
Quaternion forearmYawFrame = Quaternion.AngleAxis(rawForearmWorldYaw, Vector3.up);
Vector3 handEuler = (Quaternion.Inverse(forearmYawFrame) * rightHand.rotation).eulerAngles;
rawHandPitch = NormalizeAngle(handEuler.x);
rawHandRoll = NormalizeAngle(handEuler.z);
```

`NormalizeAngle()` は `Mathf.DeltaAngle(0f, angle)` を使い、角度を扱いやすい差分表現にします。

## ロボット関節角への変換

`UpdateRobotAngles()` で、キャリブレーション基準との差分を取ります。

```csharp
deltaYaw = Mathf.DeltaAngle(baseYaw, rawYaw);
deltaPitch = rawPitch - basePitch;
deltaElbow = rawElbow - baseElbow;
deltaForearmYaw = Mathf.DeltaAngle(baseForearmYaw, rawForearmYaw);
deltaHandPitch = Mathf.DeltaAngle(baseHandPitch, rawHandPitch);
deltaHandRoll = Mathf.DeltaAngle(baseHandRoll, rawHandRoll);
```

その後、各 J1-J6 に対して以下の式でロボット角度を作ります。

```text
robotJN = robotBaseJN + delta * jNScale * sign + jNOffset
```

`sign` は `invertJN` が有効なら `-1`、無効なら `1` です。

その後、`jNMin` / `jNMax` に収まるように制限し、`Mathf.Lerp()` でスムージングします。

## ArticulationBody への反映

`SetJointTarget()` は、指定された `ArticulationBody` の `xDrive.target` に角度を設定します。

```csharp
ArticulationDrive drive = joint.xDrive;
drive.target = targetDeg;
joint.xDrive = drive;
```

Drive に lower / upper limit が設定されている場合は、その範囲にも収めます。

```csharp
if (drive.lowerLimit < drive.upperLimit)
{
    targetDeg = Mathf.Clamp(targetDeg, drive.lowerLimit, drive.upperLimit);
}
```

つまり、最終的な角度制限は2段階あります。

1. スクリプト側の `jNMin` / `jNMax`
2. `ArticulationBody.xDrive` 側の lower / upper limit

## 外部から取得できる値

他のスクリプトから現在の計算結果を取得するため、いくつかの public property と method が用意されています。

### 現在の平滑化済み関節角

- `SmoothedJ1`
- `SmoothedJ2`
- `SmoothedJ3`
- `SmoothedJ4`
- `SmoothedJ5`
- `SmoothedJ6`

### 現在の raw / delta 角度

- `RawYaw`
- `RawPitch`
- `RawElbow`
- `RawForearmYaw`
- `RawHandPitch`
- `RawHandRoll`
- `DeltaYaw`
- `DeltaPitch`
- `DeltaElbow`
- `DeltaForearmYaw`
- `DeltaHandPitch`
- `DeltaHandRoll`

### UDP送信用の関節角取得

`MyCobotUdpJointSender.cs` などから使うため、以下の method が用意されています。

```csharp
public float[] GetRobotJoints()
```

戻り値は以下の順序です。

```text
[J1, J2, J3, J4, J5, J6]
```

J1/J2 だけ欲しい場合は以下も使えます。

```csharp
public Vector2 GetRobotJ1J2()
```

## 操作時の確認ポイント

### Play開始直後

`logJointDiagnosticsOnStart` が有効なら、Console に J1-J6 の情報が出ます。

ここで以下を確認します。

- `joint1` から `joint6` が未設定でないか
- 関節タイプが `RevoluteJoint` または `PrismaticJoint` か
- Drive limit が想定範囲か

### キャリブレーション

Play 開始後、`autoCalibrateDelay` 秒間は基準姿勢を維持します。

自動キャリブレーション後、Console に `calibrated` ログが出ます。

手動でやり直したい場合は、Inspector のコンポーネントメニューから `Calibrate Upper Arm` を実行するか、Game 画面左上の `Calibrate` ボタンを押します。

### 動きの調整

動きが逆方向の場合:

- 対応する `invertJN` を切り替える

動きが大きすぎる / 小さすぎる場合:

- 対応する `jNScale` を調整する

基準位置がずれている場合:

- 対応する `jNOffset` を調整する
- または基準姿勢を取り直して `Calibrate()` する

動きが震える場合:

- `smoothing` を下げる
- `yawUpdateThreshold` を少し上げる
- mocopi 側の入力が安定しているか確認する

## よくある問題

### `requires a Humanoid Animator` が出る

`humanoidAnimator` が未設定です。mocopi で動いている Avatar の `Animator` を Inspector に割り当てます。

### `requires an Animator with a Humanoid Avatar` が出る

対象モデルが Humanoid として認識されていません。FBX の Rig 設定で `Animation Type` が `Humanoid` になっているか確認します。

### `could not find RightUpperArm, RightLowerArm, or RightHand bones` が出る

Humanoid Avatar のボーン割り当てが不完全です。FBX の Rig 設定で Avatar Configure を確認します。

### 関節が動かない

以下を確認します。

- `joint1` から `joint6` に正しい `ArticulationBody` が設定されているか
- 設定した `ArticulationBody` が `RevoluteJoint` または `PrismaticJoint` か
- `driveOnlyAfterCalibration` が有効で、まだキャリブレーション前ではないか
- `xDrive` の lower / upper limit が狭すぎないか
- `driveForceLimit` が小さすぎないか

### J1/J4 の角度が急に飛ぶ

腕や前腕がほぼ垂直になると、X-Z 平面上の向きが不安定になります。

この対策として `yawUpdateThreshold` があり、水平成分が小さいときは前回の yaw を使います。まだ不安定な場合は、`yawUpdateThreshold` を少し大きくします。

### 腕を左右へ振ると J3/J4/J5 が不自然に動く

J3/J4/J5 がワールド座標の角度を直接使うと、腕全体を左右へ振っただけでも以下の成分が混ざります。

- J3: 3D 空間上の単純な `Vector3.Angle()` では、肘曲げ以外の横方向成分も角度に入る
- J4: 前腕のワールド yaw では、肩・上腕の yaw と前腕の相対 yaw を分離できない
- J5: 右手のワールド Euler X は、体の向きや腕 yaw によって pitch/roll の分解結果が変わる

現在の実装では、J3 は上腕 yaw に沿う垂直平面上の符号付き角度、J4 は上腕 yaw に対する前腕 yaw、J5/J6 は前腕 yaw を取り除いた手首回転から計算します。

## このスクリプトの位置づけ

このスクリプトは、mocopi などで動く Humanoid の右腕姿勢を、Unity 内の URDF ロボットモデルへ反映する中間変換層です。

役割は以下に限定されています。

- Humanoid の右腕姿勢を読む
- J1-J6 の角度に変換する
- Unity 上の `ArticulationBody` を動かす
- 他スクリプトへ J1-J6 の角度を渡す

実機制御、UDP通信、myCobot API 呼び出しはこのスクリプトの責務ではありません。
