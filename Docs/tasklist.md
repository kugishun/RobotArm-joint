# 進捗管理タスクリスト：Humanoid右腕 → URDFロボットJ1〜J6反映

## 0. 作業方針

- [x] 今回の実装範囲を「Unity内シミュレーション」に限定する
- [x] UDP送信は今回実装しないことを確認する
- [x] 実機ロボット制御は今回実装しないことを確認する
- [x] Phase 1ではJ1/J2のみを対象にして動作確認する
- [ ] Phase 1.5ではA方式でJ3〜J6を追加検証する
- [x] Humanoidの入力元はmocopi Receiverアドオンで動かされるHumanoidモデルを想定する

---

## 1. Unityプロジェクト確認

- [x] Unityプロジェクトを開ける
- [x] mocopi ReceiverアドオンがUnityプロジェクトに導入されている
- [x] mocopi Receiverで受信したモーションがHumanoidモデルへ反映される
- [x] HumanoidモデルがScene内に存在する
- [x] Humanoidモデルに `Animator` が設定されている
- [x] AvatarがHumanoidとして正しく設定されている
- [x] `Animator.isHuman` がtrueになるモデルを使用している
- [x] URDFロボットアームがUnity上に読み込まれている
- [x] URDFロボットのJ1/J2に相当する関節を特定できる

---

## 1.5. mocopi Receiver確認

- [x] mocopiアプリまたは送信元からUnityへモーションが届いている
- [x] Play中にHumanoidモデルの右腕がmocopiの動きに追従する
- [x] mocopi Receiverが制御しているHumanoidモデルの `Animator` を特定できる
- [x] `UpperArmToUrdfRobotJoints.cs` の `humanoidAnimator` にはmocopi Receiver側の `Animator` を割り当てる
- [x] 受信開始直後の姿勢が安定してからキャリブレーションする
- [ ] mocopi装着ズレは `Calibrate()` と `j1Offset` / `j2Offset` で吸収する

---

## 2. スクリプト作成

- [x] `UpperArmToUrdfRobotJoints.cs` を作成する
- [x] `MonoBehaviour` を継承したクラスを作成する
- [x] `humanoidAnimator` をInspectorから指定できるようにする
- [x] J1/J2用の `ArticulationBody` をInspectorから指定できるようにする
- [x] 必要なInspector調整パラメータを追加する

---

## 3. Humanoidボーン取得

- [x] `RightUpperArm` を取得する
- [x] `RightLowerArm` を取得する
- [x] mocopi Receiverによって更新されたボーン位置をPlay中に取得できる
- [x] ボーンが取得できない場合のエラー処理を実装する
- [x] Play中にボーン取得が成功していることを確認する

---

## 4. 上腕方向ベクトル取得

- [x] `RightUpperArm.position` を取得する
- [x] `RightLowerArm.position` を取得する
- [x] `lowerArm.position - upperArm.position` から上腕方向を計算する
- [x] `normalized` で正規化する
- [x] `armDir` をデバッグ表示できるようにする

---

## 5. yaw / pitch計算

- [x] `armDir.x` と `armDir.z` から yaw を計算する
- [x] `armDir.y` と水平成分から pitch を計算する
- [x] `rawYaw` をデバッグ表示する
- [x] `rawPitch` をデバッグ表示する
- [x] 腕を左右に動かしたときに `rawYaw` が変化することを確認する
- [x] 腕を上下に動かしたときに `rawPitch` が変化することを確認する

---

## 6. yaw不安定対策

- [x] `horizontalLength` を計算する
- [x] `yawUpdateThreshold` をInspectorから調整可能にする
- [x] `horizontalLength` が小さい場合は前回のyawを使う
- [ ] 腕が真上・真下付近にあるときにyawが暴れにくいことを確認する

---

## 7. キャリブレーション

- [x] `baseYaw` を保存する
- [x] `basePitch` を保存する
- [x] `Calibrate()` メソッドを実装する
- [x] `isCalibrated` を管理する
- [x] `deltaYaw = rawYaw - baseYaw` を計算する
- [x] `deltaPitch = rawPitch - basePitch` を計算する
- [x] Play開始後に基準姿勢でキャリブレーションできることを確認する
- [x] 自動キャリブレーションを実装する
- [x] `autoCalibrateDelay` をInspectorから調整できるようにする
- [ ] mocopi Receiverの受信とモデル姿勢が安定した後に自動キャリブレーションされるよう `autoCalibrateDelay` を調整する

---

## 8. J1/J2角度変換

- [x] `robotBaseJ1` を追加する
- [x] `robotBaseJ2` を追加する
- [x] `j1Scale` を追加する
- [x] `j2Scale` を追加する
- [x] `j1Offset` を追加する
- [x] `j2Offset` を追加する
- [x] `invertJ1` を追加する
- [x] `invertJ2` を追加する
- [x] `deltaYaw` から `robotJ1` を計算する
- [x] `deltaPitch` から `robotJ2` を計算する
- [x] 腕の左右動作がJ1へ反映されることを確認する
- [x] 腕の上下動作がJ2へ反映されることを確認する

---

## 9. 角度制限

- [x] `j1Min` / `j1Max` を追加する
- [x] `j2Min` / `j2Max` を追加する
- [x] `Mathf.Clamp` でJ1/J2を制限する
- [ ] J1/J2が設定範囲を超えないことを確認する
- [ ] 実機接続前提で安全な範囲になっているか確認する

---

## 10. スムージング

- [x] `smoothing` を追加する
- [x] `smoothing` を `Range(0f, 1f)` にする
- [x] `Mathf.Lerp` で `smoothedJ1` を計算する
- [x] `Mathf.Lerp` で `smoothedJ2` を計算する
- [ ] 急激な動作が抑えられていることを確認する
- [ ] 動きが遅すぎる場合に `smoothing` を調整する

---

## 11. URDFロボットへの反映

- [x] J1用の `ArticulationBody` をInspectorに割り当てる
- [x] J2用の `ArticulationBody` をInspectorに割り当てる
- [x] `SetJointTarget()` を実装する
- [x] `joint.xDrive.target` に角度を設定する
- [x] Play中にURDFロボットのJ1が動くことを確認する
- [x] Play中にURDFロボットのJ2が動くことを確認する
- [x] 軸方向が逆の場合は `invertJ1` / `invertJ2` で補正する
- [x] 初期角度がずれる場合は `robotBaseJ1` / `robotBaseJ2` / `j1Offset` / `j2Offset` で補正する
- [x] 動きが大きすぎる場合は `j1Scale` / `j2Scale` で補正する

---

## 12. デバッグ表示

- [x] `showDebugGui` を追加する
- [x] `showDebugLog` を追加する
- [x] `armDir` を表示する
- [x] `rawYaw` を表示する
- [x] `rawPitch` を表示する
- [x] `deltaYaw` を表示する
- [x] `deltaPitch` を表示する
- [x] `robotJ1` を表示する
- [x] `robotJ2` を表示する
- [x] `smoothedJ1` を表示する
- [x] `smoothedJ2` を表示する
- [x] 毎フレームログが大量に出ないようにする

---

## 13. 外部取得用API

- [x] `GetRobotJ1J2()` を実装する
- [x] `SmoothedJ1` プロパティを実装する
- [x] `SmoothedJ2` プロパティを実装する
- [x] `SmoothedJ3`〜`SmoothedJ6` プロパティを実装する
- [x] `RawYaw` プロパティを実装する
- [x] `RawPitch` プロパティを実装する
- [x] `RawElbow` / `RawForearmYaw` / `RawHandPitch` / `RawHandRoll` プロパティを実装する
- [x] `DeltaYaw` プロパティを実装する
- [x] `DeltaPitch` プロパティを実装する
- [x] `DeltaElbow` / `DeltaForearmYaw` / `DeltaHandPitch` / `DeltaHandRoll` プロパティを実装する
- [x] 後でUDP送信スクリプトから参照できる設計になっているか確認する

---

## 14. 動作確認

### キャリブレーション確認

- [x] mocopi ReceiverでHumanoidモデルが動いている状態にする
- [x] Play開始後、基準姿勢を取る
- [x] `Calibrate()` を実行する
- [x] `deltaYaw` が基準姿勢でほぼ0になる
- [x] `deltaPitch` が基準姿勢でほぼ0になる

### 左右方向確認

- [x] 腕を右に振る
- [x] URDFロボットのJ1が動く
- [x] 腕を左に振る
- [x] URDFロボットのJ1が逆方向に動く
- [x] 方向が逆なら `invertJ1` を変更する

### 上下方向確認

- [x] 腕を上げる
- [x] URDFロボットのJ2が動く
- [x] 腕を下げる
- [x] URDFロボットのJ2が逆方向に動く
- [x] 方向が逆なら `invertJ2` を変更する

### 安定性確認

- [ ] 腕を真上付近にする
- [ ] yawが大きく暴れないことを確認する
- [ ] J1/J2が角度制限を超えないことを確認する
- [ ] 動作が滑らかであることを確認する

---

## 15. 完了条件

- [x] mocopi Receiverで動くHumanoidモデルを入力元にできる
- [x] Humanoidの右上腕方向を取得できる
- [x] yaw / pitch を計算できる
- [x] キャリブレーション差分を使っている
- [x] yawをURDFロボットのJ1へ反映できる
- [x] pitchをURDFロボットのJ2へ反映できる
- [x] Unity上で人間の腕方向にURDFロボットが追従する
- [x] Inspectorから補正値を調整できる
- [x] 安全な角度制限が入っている
- [x] スムージングが入っている
- [x] デバッグ表示がある
- [x] UDP送信前段階としてJ1/J2角度を外部取得できる

---

## 16. Phase 2への引き継ぎメモ

Phase 1/Phase 1.5完了後、以下をUnityからmyCobot制御側へのUDP送信実装へ引き継ぐ。

- [ ] `GetRobotJoints()` で取得できるJ1〜J6角度をUDP送信対象にする
- [x] JSON形式は `main_controller_v1.py` と互換の `{ "angles": [...] }` を基本にする
- [x] `Assets/Scripts/MyCobotUdpJointSender.cs` を作成する
- [x] 送信先IP/PortをInspectorから設定できるようにする
- [x] 送信周期をInspectorから設定できるようにする
- [x] myCobotの処理速度に合わせ、送信周期の標準値を20Hz（0.05秒間隔）にする
- [x] UDP payloadをUTF-8 JSONで送信する
- [ ] 実機制御へ接続する前に送信値と受信値をログで確認する
- [ ] 実機接続時は速度・角度制限をさらに厳しくする
- [ ] グリッパーは現時点では送信しない
- [x] 将来的に `gripper` 配列の送信有無をON/OFFで切り替えられる設計にする

---

## 17. Phase 1.5：A方式によるJ3〜J6追加検証

### 方針

- [x] 方式A（人間の腕関節・手首姿勢をロボット各軸へ直接マッピング）を採用する
- [x] 既存のJ1/J2制御は維持する
- [x] J3〜J6はIKではなく近似マッピングとして実装する
- [ ] J3〜J6はUnity上のURDFロボットで動作確認してから補正値を決める

### Humanoid側の追加取得

- [x] `RightHand` を取得する
- [x] `RightUpperArm -> RightLowerArm` から上腕方向を取得する
- [x] `RightLowerArm -> RightHand` から前腕方向を取得する
- [x] 上腕方向と前腕方向から肘角度 `rawElbow` を計算する
- [x] 前腕方向から `rawForearmYaw` を計算する
- [x] `RightHand.rotation` から `rawHandPitch` / `rawHandRoll` を計算する

### J3〜J6マッピング

- [x] `joint3` / `joint4` / `joint5` / `joint6` をInspectorから指定できるようにする
- [x] `robotBaseJ3`〜`robotBaseJ6` を追加する
- [x] `j3Scale`〜`j6Scale` を追加する
- [x] `j3Offset`〜`j6Offset` を追加する
- [x] `invertJ3`〜`invertJ6` を追加する
- [x] `j3Min`〜`j6Max` を追加する
- [x] J3は肘角度差分から計算する
- [x] J4は前腕yaw差分から計算する
- [x] J5は右手pitch差分から計算する
- [x] J6は右手roll差分から計算する

### URDFロボット側の割り当て

- [ ] `joint3` に `link3` の `ArticulationBody` を割り当てる
- [ ] `joint4` に `link4` の `ArticulationBody` を割り当てる
- [ ] `joint5` に `link5` の `ArticulationBody` を割り当てる
- [ ] `joint6` に `link6` の `ArticulationBody` を割り当てる
- [ ] Consoleの `Log Joint Diagnostics` でJ3〜J6が `RevoluteJoint` になっていることを確認する

### 動作確認

- [ ] 肘を曲げたときにJ3が動く
- [ ] 前腕方向を変えたときにJ4が動く
- [ ] 手首を上下に傾けたときにJ5が動く
- [ ] 手首をひねったときにJ6が動く
- [ ] 方向が逆の場合は `invertJ3`〜`invertJ6` で補正する
- [ ] 動きが大きすぎる場合は `j3Scale`〜`j6Scale` で補正する
- [ ] 初期角度がずれる場合は `robotBaseJ3`〜`robotBaseJ6` / `j3Offset`〜`j6Offset` で補正する
- [ ] J3〜J6が角度制限を超えないことを確認する

### 外部取得API

- [x] `GetRobotJoints()` でJ1〜J6角度を取得できる
- [ ] Phase 2では `GetRobotJoints()` のJ1〜J6角度をUnity側UDP送信スクリプトから送信対象にする

---

## 18. グリッパー付きURDFモデルへの切り替え

### 方針

- [x] 現在使用しているURDFロボットモデルを確認する
- [ ] グリッパー付きモデルへ切り替える対象を決める
- [x] 既存のJ1～J6制御ロジックは維持する
- [x] グリッパー追加によって既存の `UpperArmToUrdfRobotJoints.cs` の割り当てが壊れないことを確認する

### 対象URDF

- [x] `Assets/URDF/mycobot_320_pi_2022/mycobot_320_pi_2022_adaptive_gripper.urdf` をこのUnityプロジェクト向けに修正する
- [x] `Assets/URDF/mycobot_320_pi_2022/mycobot_320_pi_2022_force_gripper.urdf` をこのUnityプロジェクト向けに修正する
- [x] 2つのURDFで共通化できる修正内容と、グリッパー種別ごとに分ける修正内容を整理する
- [x] 修正前のURDFをバックアップまたは差分で復元できる状態にする

### Unity取り込み確認

- [x] URDF内のmesh / material参照パスが `Assets/URDF/mycobot_320_pi_2022/` 配下の実ファイルと一致していることを確認する
- [ ] Unity上でURDF Importerがエラーなく読み込めることを確認する
- [x] Import後のリンク階層がJ1～J6とグリッパー部で破綻していないことを確認する
- [x] J1～J6の `ArticulationBody` が既存タスクの割り当て対象として取得できることを確認する
- [x] グリッパー側の関節名、リンク名、可動範囲を確認する

### 制御割り当て確認

- [ ] 既存の `joint1`～`joint6` Inspector割り当てをグリッパー付きモデルでも再設定する
- [ ] `Log Joint Diagnostics` でJ1～J6が想定どおり `RevoluteJoint` として認識されることを確認する
- [x] グリッパー関節は今回のUDP送信実装では未実装とする
- [ ] グリッパーを制御対象に含める場合は、`gripper` 配列、送信ON/OFF設定、開閉用の `ArticulationBody` 割り当て、角度制限を追加タスク化する

### 動作確認

- [ ] adaptive gripper版でJ1～J6が既存モデルと同じ方向に動くことを確認する
- [ ] force gripper版でJ1～J6が既存モデルと同じ方向に動くことを確認する
- [ ] 方向が逆の場合は `invertJ1`～`invertJ6` で補正できることを確認する
- [ ] 初期角度がずれる場合は `robotBaseJ1`～`robotBaseJ6` / `j1Offset`～`j6Offset` で補正できることを確認する
- [ ] グリッパー追加後もJ1～J6が角度制限を超えないことを確認する
