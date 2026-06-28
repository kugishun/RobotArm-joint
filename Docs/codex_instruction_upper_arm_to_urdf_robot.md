# Codex指示書：Unity Humanoid右腕をURDFロボットアームのJ1〜J6へ反映する実装

## 目的

Unity上でHumanoidモデルの右上腕方向を取得し、その動きをUnity上に読み込んだURDFロボットアームへ反映する。

Humanoidモデルは、mocopi Receiverアドオンで受信したモーションによって動くモデルを想定する。  
このスクリプトはmocopi Receiverそのものを置き換えず、mocopi Receiverが更新したHumanoidボーン位置を読み取ってJ1〜J6へ変換する。

最初の目標は、実機のロボットアームを直接動かすことではなく、Unity上のURDFロボットモデルを使ってシミュレーションすることである。

具体的には、以下の対応をUnity上で確認する。

- 人間の腕の水平方向の回転 → URDFロボットアームの第1軸 J1
- 人間の腕の垂直方向の回転 → URDFロボットアームの第2軸 J2
- 人間の肘の曲げ角 → URDFロボットアームの第3軸 J3
- 人間の前腕方向 → URDFロボットアームの第4軸 J4
- 人間の手首pitch → URDFロボットアームの第5軸 J5
- 人間の手首roll → URDFロボットアームの第6軸 J6

このシミュレーションが完了し、動作確認ができた後に、同じJ1〜J6角度をUnityからUDP通信でmyCobot制御側へ送信する段階に進む。

---

## 開発フェーズ

この実装は以下の2段階に分ける。

### Phase 1：Unity内シミュレーション

まずはUnity内で完結させる。

- Humanoidモデルから上腕方向ベクトルを取得する
- 上腕方向ベクトルから yaw / pitch を計算する
- yaw をURDFロボットのJ1へ反映する
- pitch をURDFロボットのJ2へ反映する
- Unity上でロボットアームの動きが人間の腕方向に追従することを確認する

この段階ではUDP通信は実装しなくてよい。

### Phase 1.5：A方式でJ3〜J6を追加検証

Phase 1でJ1/J2が動作確認できた後、人間の右腕関節・手首姿勢をロボット各軸へ直接マッピングするA方式でJ3〜J6を追加する。

- J3：肘角度
- J4：前腕方向yaw
- J5：右手pitch
- J6：右手roll

これはIKではなく近似マッピングである。ロボットと人体の関節構造は一致しないため、各軸に対して `scale` / `offset` / `invert` / `min` / `max` をInspectorで調整できる設計にする。

### Phase 2：UnityからmyCobotへのUDP送信

Phase 1/Phase 1.5でUnity上のURDFロボットが正しく動くことを確認した後、UDP送信を追加する。

- Unityで計算したJ1〜J6角度をJSON形式にする
- `Assets/Scripts` 配下にUnity側UDP送信スクリプトを追加する
- `UpperArmToUrdfRobotJoints.GetRobotJoints()` から取得した各ジョイント角度を送信対象にする
- 送信先はmyCobot制御側のUDP受信IP/Portにする
- `main_controller_v1.py` の送信payloadと互換性があるように、JSONのトップレベルキーは `angles` を使う
- グリッパーは現時点では未実装とし、将来的にON/OFF設定で `gripper` 配列を送信対象に含められる設計にする

今回の次段階では、Unity上で確認済みのJ1〜J6角度をmyCobot側へUDP送信するところまでを優先する。

---

## 今回の実装対象

今回作成するスクリプトは、以下を担当する。

```text
mocopi Receiver
↓
Humanoidの右腕ボーン取得
↓
yaw / pitch / elbow / forearm yaw / hand pitch / hand roll 変換
↓
キャリブレーション差分計算
↓
J1〜J6角度への変換
↓
Unity上のURDFロボットアームの関節へ反映
```

Phase 2では、このJ1〜J6角度をUnity側UDP送信スクリプトから参照してmyCobot側へ送信する。

グリッパー値は現時点では送信しない。ただし、後でON/OFF設定によって `gripper` 配列の送信を有効化できるよう、UDP送信スクリプト側には拡張余地を残す。

---

## 想定ファイル名

```text
UpperArmToUrdfRobotJoints.cs
```

---

## 前提条件

- mocopi ReceiverアドオンがUnityプロジェクトに導入されている
- mocopi Receiverで受信したモーションがHumanoidモデルへ反映されている
- Unityプロジェクト内にHumanoidモデルが存在する
- Humanoidモデルには `Animator` が設定されている
- HumanoidモデルのAvatarはHumanoidとして正しく設定され、`Animator.isHuman` がtrueになる
- Unity上にURDF Importer等で読み込んだロボットアームモデルが存在する
- ロボットアームにはJ1〜J6に相当する関節オブジェクトがある
- まずUnity上のロボットモデルのJ1/J2を確認し、その後A方式でJ3〜J6を追加検証する
- Phase 2では実機接続前に、Unityから送信されるUDP JSONをログで確認する

---

## mocopi Receiverとの接続方針

mocopi Receiverアドオンは、Unity内のHumanoidモデルを動かす入力元として扱う。

`UpperArmToUrdfRobotJoints.cs` は、mocopi Receiverが更新したHumanoidモデルの `Animator` から `RightUpperArm`、`RightLowerArm`、`RightHand` を取得する。  
そのため、Inspectorの `humanoidAnimator` には、mocopi Receiverによって実際に動いているHumanoidモデルの `Animator` を割り当てる。

この段階では、mocopiの通信処理や受信データ形式には直接依存しない。  
スクリプト側は「HumanoidボーンがPlay中に更新されている」ことだけを前提にする。

確認ポイント：

- Play中にmocopi Receiverが接続済みであること
- Humanoidモデルの右腕がmocopiの動きに追従していること
- `humanoidAnimator` にReceiver側で動いているモデルの `Animator` が割り当てられていること
- 受信開始直後の姿勢が安定してから `Calibrate()` すること

---

## Humanoid側の取得対象

Humanoidから以下のボーンを取得する。

```csharp
HumanBodyBones.RightUpperArm
HumanBodyBones.RightLowerArm
HumanBodyBones.RightHand
```

取得した2点の位置から、上腕方向ベクトルを計算する。

```csharp
Vector3 armDir = (lowerArm.position - upperArm.position).normalized;
```

この `armDir` をもとに、上腕がどちらを向いているかを判断する。

J3〜J6追加時は、右手位置から前腕方向も計算する。

```csharp
Vector3 forearmDir = (rightHand.position - lowerArm.position).normalized;
```

---

## 角度計算

### J1用：水平方向 yaw

Unity座標系でX-Z平面上の向きを求める。

```csharp
float yaw = Mathf.Atan2(armDir.x, armDir.z) * Mathf.Rad2Deg;
```

この `yaw` を、ロボットアームのJ1相当の角度に変換する。

### J2用：垂直方向 pitch

上腕方向のY成分と水平成分から垂直角を求める。

```csharp
float horizontalLength = new Vector2(armDir.x, armDir.z).magnitude;
float pitch = Mathf.Atan2(armDir.y, horizontalLength) * Mathf.Rad2Deg;
```

この `pitch` を、ロボットアームのJ2相当の角度に変換する。

### J3用：肘角度

上腕方向と前腕方向のなす角を求める。

```csharp
float elbow = Vector3.Angle(armDir, forearmDir);
```

この `elbow` を、キャリブレーション時の基準肘角との差分としてJ3に反映する。

### J4用：前腕方向 yaw

前腕方向のX-Z平面上の向きを求める。

```csharp
float forearmYaw = Mathf.Atan2(forearmDir.x, forearmDir.z) * Mathf.Rad2Deg;
```

この `forearmYaw` を、キャリブレーション時の基準前腕yawとの差分としてJ4に反映する。

### J5/J6用：右手姿勢

右手のワールド回転からpitch/rollを取得し、キャリブレーション時との差分として扱う。

```csharp
Vector3 handEuler = rightHand.rotation.eulerAngles;
float handPitch = Mathf.DeltaAngle(0f, handEuler.x);
float handRoll = Mathf.DeltaAngle(0f, handEuler.z);
```

この方式はmocopiの手首姿勢推定とHumanoid Avatarのリターゲット品質に依存する。J5/J6は特に `scale` / `offset` / `invert` を調整する前提で扱う。

---

## URDFロボット側への反映

### 重要

URDFロボットアームをUnityに読み込んだ場合、関節の動かし方はプロジェクト構成によって変わる。

想定される方式は以下のどちらかである。

1. 関節オブジェクトの `Transform.localRotation` を直接変更する方式
2. Unity Robotics / URDF Importer の `ArticulationBody` を使い、`xDrive.target` などで関節角を指定する方式

可能であれば、`ArticulationBody` を使う実装を優先する。

---

## ArticulationBodyを使う場合

J1〜J6に相当する `ArticulationBody` をInspectorから指定できるようにする。

```csharp
[SerializeField] private ArticulationBody joint1;
[SerializeField] private ArticulationBody joint2;
[SerializeField] private ArticulationBody joint3;
[SerializeField] private ArticulationBody joint4;
[SerializeField] private ArticulationBody joint5;
[SerializeField] private ArticulationBody joint6;
```

計算した角度をArticulationBodyのdrive targetに反映する。

例：

```csharp
private void SetJointTarget(ArticulationBody joint, float targetDeg)
{
    if (joint == null) return;

    var drive = joint.xDrive;
    drive.target = targetDeg;
    joint.xDrive = drive;
}
```

J1〜J6へ反映する。

```csharp
SetJointTarget(joint1, smoothedJ1);
SetJointTarget(joint2, smoothedJ2);
SetJointTarget(joint3, smoothedJ3);
SetJointTarget(joint4, smoothedJ4);
SetJointTarget(joint5, smoothedJ5);
SetJointTarget(joint6, smoothedJ6);
```

ただし、URDFから読み込んだ関節の軸方向によって、`xDrive` ではなく別の軸設定が必要になる可能性がある。  
その場合でも、Inspectorで符号反転・オフセット・スケールを調整できる設計にする。

---

## Transform.localRotationを使う場合

ArticulationBodyではなくTransformで制御する場合は、J1〜J6の関節TransformをInspectorから指定できるようにする。

```csharp
[SerializeField] private Transform joint1Transform;
[SerializeField] private Transform joint2Transform;
[SerializeField] private Transform joint3Transform;
[SerializeField] private Transform joint4Transform;
[SerializeField] private Transform joint5Transform;
[SerializeField] private Transform joint6Transform;
```

ただし、回転軸はロボットモデルによって異なるため、Inspectorから軸を選べるようにする。

```csharp
public enum RotationAxis
{
    X,
    Y,
    Z
}

[SerializeField] private RotationAxis joint1Axis = RotationAxis.Y;
[SerializeField] private RotationAxis joint2Axis = RotationAxis.X;
```

角度反映例：

```csharp
private void SetLocalRotationByAxis(Transform joint, RotationAxis axis, float angleDeg)
{
    if (joint == null) return;

    Vector3 euler = Vector3.zero;

    switch (axis)
    {
        case RotationAxis.X:
            euler.x = angleDeg;
            break;
        case RotationAxis.Y:
            euler.y = angleDeg;
            break;
        case RotationAxis.Z:
            euler.z = angleDeg;
            break;
    }

    joint.localRotation = Quaternion.Euler(euler);
}
```

ただし、URDFロボットではArticulationBodyの物理制御とTransform直接操作が衝突する可能性があるため、基本はArticulationBody方式を優先する。

---

## 必須パラメータ

以下はInspectorから調整できるようにする。

```csharp
[Header("Humanoid")]
[SerializeField] private Animator humanoidAnimator;

[Header("URDF Robot Joints")]
[SerializeField] private ArticulationBody joint1;
[SerializeField] private ArticulationBody joint2;
[SerializeField] private ArticulationBody joint3;
[SerializeField] private ArticulationBody joint4;
[SerializeField] private ArticulationBody joint5;
[SerializeField] private ArticulationBody joint6;

[Header("Robot Base Angles")]
[SerializeField] private float robotBaseJ1 = 0f;
[SerializeField] private float robotBaseJ2 = 0f;
[SerializeField] private float robotBaseJ3 = 0f;
[SerializeField] private float robotBaseJ4 = 0f;
[SerializeField] private float robotBaseJ5 = 0f;
[SerializeField] private float robotBaseJ6 = 0f;

[Header("Mapping")]
[SerializeField] private float j1Scale = 1.0f;
[SerializeField] private float j2Scale = 1.0f;
[SerializeField] private float j3Scale = 1.0f;
[SerializeField] private float j4Scale = 1.0f;
[SerializeField] private float j5Scale = 1.0f;
[SerializeField] private float j6Scale = 1.0f;
[SerializeField] private float j1Offset = 0f;
[SerializeField] private float j2Offset = 0f;
[SerializeField] private float j3Offset = 0f;
[SerializeField] private float j4Offset = 0f;
[SerializeField] private float j5Offset = 0f;
[SerializeField] private float j6Offset = 0f;
[SerializeField] private bool invertJ1 = false;
[SerializeField] private bool invertJ2 = false;
[SerializeField] private bool invertJ3 = false;
[SerializeField] private bool invertJ4 = false;
[SerializeField] private bool invertJ5 = false;
[SerializeField] private bool invertJ6 = false;

[Header("Joint Limits")]
[SerializeField] private float j1Min = -165f;
[SerializeField] private float j1Max = 165f;
[SerializeField] private float j2Min = -165f;
[SerializeField] private float j2Max = 165f;
[SerializeField] private float j3Min = -165f;
[SerializeField] private float j3Max = 165f;
[SerializeField] private float j4Min = -165f;
[SerializeField] private float j4Max = 165f;
[SerializeField] private float j5Min = -165f;
[SerializeField] private float j5Max = 165f;
[SerializeField] private float j6Min = -180f;
[SerializeField] private float j6Max = 180f;

[Header("Stability")]
[SerializeField] private float yawUpdateThreshold = 0.1f;
[SerializeField, Range(0f, 1f)] private float smoothing = 0.2f;

[Header("Calibration")]
[SerializeField] private bool autoCalibrateOnStart = true;
[SerializeField] private float autoCalibrateDelay = 2.0f;

[Header("Debug")]
[SerializeField] private bool showDebugGui = true;
[SerializeField] private bool showDebugLog = false;
```

---

## 変換式

Humanoidから取得した角度を基準姿勢との差分として扱う。

```csharp
float deltaYaw = Mathf.DeltaAngle(baseYaw, rawYaw);
float deltaPitch = rawPitch - basePitch;
float deltaElbow = rawElbow - baseElbow;
float deltaForearmYaw = Mathf.DeltaAngle(baseForearmYaw, rawForearmYaw);
float deltaHandPitch = Mathf.DeltaAngle(baseHandPitch, rawHandPitch);
float deltaHandRoll = Mathf.DeltaAngle(baseHandRoll, rawHandRoll);
```

J1〜J6への変換式は以下とする。

```csharp
float j1Sign = invertJ1 ? -1f : 1f;
float j2Sign = invertJ2 ? -1f : 1f;
float j3Sign = invertJ3 ? -1f : 1f;
float j4Sign = invertJ4 ? -1f : 1f;
float j5Sign = invertJ5 ? -1f : 1f;
float j6Sign = invertJ6 ? -1f : 1f;

float robotJ1 = robotBaseJ1 + deltaYaw * j1Scale * j1Sign + j1Offset;
float robotJ2 = robotBaseJ2 + deltaPitch * j2Scale * j2Sign + j2Offset;
float robotJ3 = robotBaseJ3 + deltaElbow * j3Scale * j3Sign + j3Offset;
float robotJ4 = robotBaseJ4 + deltaForearmYaw * j4Scale * j4Sign + j4Offset;
float robotJ5 = robotBaseJ5 + deltaHandPitch * j5Scale * j5Sign + j5Offset;
float robotJ6 = robotBaseJ6 + deltaHandRoll * j6Scale * j6Sign + j6Offset;
```

その後、角度制限を行う。

```csharp
robotJ1 = Mathf.Clamp(robotJ1, j1Min, j1Max);
robotJ2 = Mathf.Clamp(robotJ2, j2Min, j2Max);
robotJ3 = Mathf.Clamp(robotJ3, j3Min, j3Max);
robotJ4 = Mathf.Clamp(robotJ4, j4Min, j4Max);
robotJ5 = Mathf.Clamp(robotJ5, j5Min, j5Max);
robotJ6 = Mathf.Clamp(robotJ6, j6Min, j6Max);
```

最後にスムージングする。

```csharp
smoothedJ1 = Mathf.Lerp(smoothedJ1, robotJ1, smoothing);
smoothedJ2 = Mathf.Lerp(smoothedJ2, robotJ2, smoothing);
smoothedJ3 = Mathf.Lerp(smoothedJ3, robotJ3, smoothing);
smoothedJ4 = Mathf.Lerp(smoothedJ4, robotJ4, smoothing);
smoothedJ5 = Mathf.Lerp(smoothedJ5, robotJ5, smoothing);
smoothedJ6 = Mathf.Lerp(smoothedJ6, robotJ6, smoothing);
```

---

## キャリブレーション

`Calibrate()` メソッドを実装する。

キャリブレーション時の `rawYaw` / `rawPitch` / `rawElbow` / `rawForearmYaw` / `rawHandPitch` / `rawHandRoll` を基準として保存する。

```csharp
public void Calibrate()
{
    baseYaw = rawYaw;
    basePitch = rawPitch;
    baseElbow = rawElbow;
    baseForearmYaw = rawForearmYaw;
    baseHandPitch = rawHandPitch;
    baseHandRoll = rawHandRoll;
    isCalibrated = true;
}
```

キャリブレーション後は、基準姿勢との差分でJ1〜J6を動かす。

目的：

- Humanoidモデルの初期姿勢の違いを吸収する
- mocopi Receiverの受信開始時の初期姿勢差を吸収する
- mocopiセンサー装着ズレや人体モデルの初期向きの違いを吸収する
- ロボットの初期姿勢と人間の初期姿勢を対応づける

mocopi Receiverを使う場合、Play開始直後は受信接続や姿勢推定が安定していない可能性がある。  
自動キャリブレーションを使う場合は、`autoCalibrateDelay` を少し長めにし、Humanoidの腕姿勢が安定してから基準値を保存する。

---

## yaw不安定対策

腕が真上または真下に近い場合、水平成分が小さくなり、yawが不安定になる。

そのため、以下を実装する。

```csharp
float horizontalLength = new Vector2(armDir.x, armDir.z).magnitude;

if (horizontalLength > yawUpdateThreshold)
{
    rawYaw = Mathf.Atan2(armDir.x, armDir.z) * Mathf.Rad2Deg;
    previousRawYaw = rawYaw;
}
else
{
    rawYaw = previousRawYaw;
}
```

---

## 外部取得用メソッド

Unity側UDP送信スクリプトから参照できるように、現在のJ1〜J6角度を外部から取得できるメソッドを実装する。

```csharp
public Vector2 GetRobotJ1J2()
{
    return new Vector2(smoothedJ1, smoothedJ2);
}

public float[] GetRobotJoints()
{
    return new[] { smoothedJ1, smoothedJ2, smoothedJ3, smoothedJ4, smoothedJ5, smoothedJ6 };
}
```

また、UDP送信用JSONに変換しやすいように、値の意味が分かるプロパティも用意する。

```csharp
public float SmoothedJ1 => smoothedJ1;
public float SmoothedJ2 => smoothedJ2;
public float SmoothedJ3 => smoothedJ3;
public float SmoothedJ4 => smoothedJ4;
public float SmoothedJ5 => smoothedJ5;
public float SmoothedJ6 => smoothedJ6;
public float RawYaw => rawYaw;
public float RawPitch => rawPitch;
public float RawElbow => rawElbow;
public float RawForearmYaw => rawForearmYaw;
public float RawHandPitch => rawHandPitch;
public float RawHandRoll => rawHandRoll;
public float DeltaYaw => deltaYaw;
public float DeltaPitch => deltaPitch;
public float DeltaElbow => deltaElbow;
public float DeltaForearmYaw => deltaForearmYaw;
public float DeltaHandPitch => deltaHandPitch;
public float DeltaHandRoll => deltaHandRoll;
```

---

## Unity側UDP送信スクリプト

Phase 2では `Assets/Scripts` 配下に、UnityからmyCobot制御側へJ1〜J6角度を送信するスクリプトを追加する。

想定ファイル名：

```text
MyCobotUdpJointSender.cs
```

送信元は `UpperArmToUrdfRobotJoints` とし、`GetRobotJoints()` で取得した6軸の角度を送信する。

`main_controller_v1.py` ではmyCobot側へ以下の形式でUDP送信しているため、Unity側スクリプトもこのpayload形式に合わせる。

```json
{
  "angles": [90.0, 90.0, 0.0, -90.0, -90.0, 90.0]
}
```

通信仕様：

- 送信プロトコル：UDP
- 送信形式：UTF-8 JSON
- 送信キー：`angles`
- 値：J1〜J6の6要素配列
- 送信周期：myCobot側の処理速度に合わせて20Hz（0.05秒間隔）を標準にし、Inspectorから調整可能にする
- 送信先IP/Port：Inspectorから設定可能にする
- 実機接続前に、送信JSONをConsoleログまたは受信側ログで確認する

グリッパーは現時点では未実装でよい。将来的にはInspectorのON/OFF設定で `gripper` キーをpayloadへ追加できるようにする。

`gripper` はboolではなく、`angles` と同様に複数値を格納する配列として扱う。

将来の拡張payload例：

```json
{
  "angles": [90.0, 90.0, 0.0, -90.0, -90.0, 90.0],
  "gripper": [0.0, 0.0, 0.0]
}
```

ただし、現段階で送信する必須データは `angles` のみとする。

---

## 今回実装しないもの

以下はPhase 1/Phase 1.5の必須実装に含めない。

- Python側受信プログラム
- myCobotへの `send_angles()` 実行
- グリッパー制御
- IK制御

ただし、Phase 2でUnity側UDP送信を追加できるように、J1〜J6の角度取得メソッドは用意する。

---

## 期待する成果物

以下を出力すること。

1. `UpperArmToUrdfRobotJoints.cs` の完全なコード
2. Unity上でのセットアップ手順
3. Humanoid側で指定するもの
4. URDFロボット側で指定するもの
5. ArticulationBodyを使う場合の注意点
6. J1〜J6の向きが逆だった場合の補正方法
7. 動作確認手順

---

## Unity上でのセットアップ手順も説明すること

説明には以下を含める。

### Humanoid側

- mocopi Receiverアドオンを導入し、Unityでmocopiのモーションを受信できる状態にする
- mocopi Receiverで動かすHumanoidモデルをSceneに配置する
- Humanoidモデルに `Animator` があることを確認する
- AvatarがHumanoidとして設定されていることを確認する
- Play中にHumanoidモデルの右腕がmocopiの動きに追従することを確認する
- スクリプトの `humanoidAnimator` に、mocopi Receiverが動かしているHumanoidモデルの `Animator` を割り当てる

### URDFロボット側

- URDF Importer等でロボットアームをUnityに読み込む
- J1〜J6に相当する関節オブジェクトを確認する
- J1〜J6の `ArticulationBody` をスクリプトに割り当てる
- Play中にHumanoidの腕・肘・手首を動かし、URDFロボットのJ1〜J6が動くか確認する

---

## 動作確認

### 0. mocopi Receiver確認

Play開始後、mocopi ReceiverがHumanoidモデルへモーションを反映していることを確認する。

期待：

```text
mocopiで腕を動かすと、Unity上のHumanoidモデルの右腕が動く
```

この確認ができてから、J1〜J6への変換確認に進む。

### 1. キャリブレーション

Play開始後、腕を基準姿勢にする。

例：

```text
右腕を前方向に軽く伸ばした状態
```

その状態で `Calibrate()` を実行する。  
または、自動キャリブレーションを使う。

mocopi Receiverを使う場合、自動キャリブレーションは受信と姿勢が安定してから実行される必要がある。  
基準姿勢を取り直したい場合は、Play中のデバッグGUIまたはInspectorのContext Menuから `Calibrate()` を再実行する。

### 2. 腕を左右に振る

期待：

```text
URDFロボットのJ1が左右方向に動く
```

逆向きなら `invertJ1` を変更する。

### 3. 腕を上下に動かす

期待：

```text
URDFロボットのJ2が上下方向に動く
```

逆向きなら `invertJ2` を変更する。

### 4. 可動範囲を確認する

期待：

```text
J1〜J6が設定した min/max を超えない
```

### 5. 肘・前腕・手首を動かす

期待：

```text
肘を曲げるとJ3、前腕方向を変えるとJ4、手首pitch/rollでJ5/J6が動く
```

逆向きなら `invertJ3`〜`invertJ6` を変更する。

### 6. 動きが大きすぎる場合

以下を調整する。

```text
j1Scale
j2Scale
j3Scale
j4Scale
j5Scale
j6Scale
```

### 7. 初期位置がずれている場合

以下を調整する。

```text
robotBaseJ1
robotBaseJ2
robotBaseJ3
robotBaseJ4
robotBaseJ5
robotBaseJ6
j1Offset
j2Offset
j3Offset
j4Offset
j5Offset
j6Offset
```

---

## 完了条件

以下を満たせば完了とする。

- mocopi Receiverで動くHumanoidモデルを入力元として使用できる
- Humanoidの右上腕・右前腕・右手姿勢を取得できる
- 上腕方向から yaw / pitch、前腕・手首からJ3〜J6用角度を計算できる
- キャリブレーションにより初期姿勢との差分を取れる
- yawをURDFロボットのJ1へ反映できる
- pitchをURDFロボットのJ2へ反映できる
- 肘角度をURDFロボットのJ3へ反映できる
- 前腕yawをURDFロボットのJ4へ反映できる
- 手首pitch/rollをURDFロボットのJ5/J6へ反映できる
- Unity上で人間の右腕・肘・手首動作にURDFロボットが追従する
- 符号反転、オフセット、スケール、角度制限をInspectorから調整できる
- Phase 2のUDP送信前提として、`GetRobotJoints()` で現在のJ1〜J6角度を取得できる

---

## 重要な設計方針

Phase 1/Phase 1.5の実装は、実機ロボットを直接動かすものではない。

まずUnity上のURDFロボットアームで、人間の右腕・肘・手首の動きをJ1〜J6へ近似的に反映できるかを検証する。

Phase 2では、このシミュレーションで確認したJ1〜J6角度をUnityからUDPでmyCobot制御側へ送信する。実機制御に接続する前に、送信payloadと角度範囲をログで確認する。

コメントにも以下の意図を明記すること。

```text
This script maps Humanoid right-arm motion to a URDF robot arm inside Unity.
UDP communication to the myCobot controller is implemented as Phase 2 after simulation validation.
```
