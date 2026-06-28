# RobotArm-joint Unity セットアップガイド

このガイドは、`Assets/` 配下の現行構成を前提に、Unity Editor でプロジェクトを開き、mocopi 入力、URDF ロボット、UDP 送信まで確認するための手順です。

## 前提環境

- Unity Editor: `6000.0.74f1`
- Unity Hub からプロジェクトルート `C:\Users\kugis\Unity_Projects\RobotArm-joint` を開く
- 初回起動時は `Packages/manifest.json` の依存解決のためインターネット接続が必要
- mocopi を使う場合、PC と mocopi 送信端末を同一ネットワークに置く
- 実機 myCobot へ送信する場合、受信側が UDP `7010` を待ち受けていること

## 主要構成

- 起動シーン: `Assets/Scenes/SampleScene.unity`
- mocopi Receiver: `Assets/MocopiReceiver/`
- URDF ロボット: `Assets/URDF/mycobot_320_pi_2022/`
- 上腕からロボット関節への変換: `Assets/Scripts/UpperArmToUrdfRobotJoints.cs`
- Unity から UDP で関節角を送る処理: `Assets/Scripts/MyCobotUdpJointSender.cs`
- ビルド対象シーン: `Assets/Scenes/SampleScene.unity`

## 初回セットアップ

1. Unity Hub で `C:\Users\kugis\Unity_Projects\RobotArm-joint` を追加する。
2. Unity `6000.0.74f1` で開く。
3. Package Manager の依存解決が終わるまで待つ。
   - `com.unity.robotics.urdf-importer` は GitHub URL 依存なので、初回はネットワークが必要。
4. Console にコンパイルエラーが出ていないことを確認する。
5. `Assets/Scenes/SampleScene.unity` を開く。
6. `Assets > Refresh` を実行し、URDF / DAE / `.meta` の取り込み状態を更新する。

## シーン確認

`SampleScene` には以下の主要オブジェクトがあります。

- `MocopiSimpleReceiver`
  - mocopi の UDP データを受信して Humanoid アバターへ反映する。
  - 現在の受信ポートは `12351`。
- `MocopiAvatar`
  - mocopi Receiver により動かされる Humanoid モデル。
  - `Animator` が Humanoid として有効である必要がある。
- `Mycobot_gripper`
  - URDF Importer で取り込まれた myCobot 320 + グリッパーのロボットモデル。
- `UDP Sender`
  - `MyCobotUdpJointSender` が付いている。
  - 現在の送信先は `192.168.1.150:7010`。
  - 送信頻度は `20 Hz`。

## mocopi 入力の設定

1. mocopi アプリ側で送信先 IP を Unity を動かす PC の IP アドレスにする。
2. mocopi アプリ側の送信ポートを `12351` にする。
3. Unity で `MocopiSimpleReceiver` の `AvatarSettings[0].Port` が `12351` であることを確認する。
4. Play を押し、`MocopiAvatar` の右腕が mocopi の動きに追従することを確認する。
5. 動きが来ない場合は、Windows ファイアウォールで Unity Editor の UDP 受信を許可する。

## URDF ロボットの確認

1. `Mycobot_gripper` がシーン上に表示されていることを確認する。
2. `UpperArmToUrdfRobotJoints` の Inspector で以下を確認する。
   - `Humanoid Animator` に mocopi 側で動く Humanoid の `Animator` が設定されている。
   - `Joint1` から `Joint6` に、対応する `ArticulationBody` が設定されている。
3. Play 後、約 `2` 秒の自動キャリブレーションを待つ。
4. 右腕を動かし、J1 から J6 の関節値とロボットモデルが追従することを確認する。
5. 必要に応じて `j1Scale` / `j1Offset` / `invertJ1` などを Inspector で調整する。

## UDP 送信の設定

`UDP Sender` の `MyCobotUdpJointSender` は、`UpperArmToUrdfRobotJoints.GetRobotJoints()` から取得した J1-J6 の角度を JSON で送信します。

現在の送信 payload は以下の形式です。

```json
{"angles":[0,0,0,0,0,0]}
```

設定項目:

- `Remote Ip`: myCobot 制御側 PC / Raspberry Pi の IP。現状は `192.168.1.150`。
- `Remote Port`: UDP 受信ポート。現状は `7010`。
- `Send Continuously`: 有効時、Play 中に連続送信する。
- `Send Rate Hz`: 最大 `20 Hz` にクランプされる。
- `Log Sent Payload`: 送信内容を Console で確認したい場合だけ有効にする。
- `Include Gripper`: グリッパー値も送る場合に有効にする。現状は無効。

実機を動かす前に、受信側で UDP JSON をログ出力し、`angles` の値と順序が期待通りか確認してください。

## URDF / メッシュ再インポート時の注意

グリッパー付き URDF は、Unity の URDF Importer が `Assets/URDF/mycobot_320_pi_2022` を基準に相対パスを解決する構成です。

- グリッパーメッシュは `Assets/URDF/mycobot_320_pi_2022/Meshes/...` 配下に置く。
- URDF 内の mesh 参照は `Meshes/...` 形式にする。
- 期待パスにファイルが存在するのに Unity が見つけられない場合、先に `Assets > Refresh` と Reimport を試す。
- 同じメッシュを別場所へコピーした場合、古い `.meta` の GUID 重複に注意する。

## 動作確認チェックリスト

- [ ] Unity `6000.0.74f1` でプロジェクトが開ける
- [ ] Console に C# コンパイルエラーがない
- [ ] `SampleScene` が開ける
- [ ] mocopi 送信ポートと Unity 受信ポートが `12351` で一致している
- [ ] `MocopiAvatar` の右腕が Play 中に動く
- [ ] `UpperArmToUrdfRobotJoints` が Humanoid `Animator` と J1-J6 を参照している
- [ ] `Mycobot_gripper` の関節が Unity 上で追従する
- [ ] UDP 送信先 `Remote Ip` / `Remote Port` が受信側と一致している
- [ ] 実機接続前に受信側ログで `{"angles":[...]}` を確認した

## トラブルシュート

### Package Manager が URDF Importer を取得できない

- インターネット接続と GitHub へのアクセスを確認する。
- `Packages/manifest.json` の `com.unity.robotics.urdf-importer` が Git URL のままになっていることを確認する。
- Unity を再起動して Package Manager の解決をやり直す。

### mocopi の動きが Unity に入らない

- mocopi アプリの送信先 IP が PC の IP になっているか確認する。
- 送信ポートが `12351` か確認する。
- Windows ファイアウォールで Unity Editor の UDP 受信を許可する。
- `MocopiSimpleReceiver` の `IsReceivingOnEnable` が有効か確認する。

### Humanoid の右腕が取得できない

- 対象モデルの `Animator` に Humanoid Avatar が設定されているか確認する。
- `Animator.isHuman` が true になるモデルを使う。
- `UpperArmToUrdfRobotJoints` の `Humanoid Animator` に、実際に mocopi で動いている Avatar の `Animator` を割り当てる。

### URDF のメッシュが見つからない

- `Assets/URDF/mycobot_320_pi_2022/Meshes/...` に対象 `.dae` があるか確認する。
- URDF の mesh 参照が `Meshes/...` になっているか確認する。
- ファイルが存在する場合はパス変更を繰り返さず、`Assets > Refresh` と Reimport を実行する。

### UDP は送信されるが実機が動かない

- `UDP Sender` の `Remote Ip` と `Remote Port` を受信側に合わせる。
- `Log Sent Payload` を有効にして、Unity Console に JSON が出るか確認する。
- 受信側で `angles` 配列の順序、単位、可動範囲を検証する。
- 実機制御側は同期実行でキューを詰めるより、最新値で上書きする制御ループにする。
