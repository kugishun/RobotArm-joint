# RobotArm-joint Unity Setup Guide

This guide describes how to open the current Unity project, verify mocopi input, verify the URDF robot, and check UDP joint-angle transmission based on the current `Assets/` structure.

## Prerequisites

- Unity Editor: `6000.0.74f1`
- Open the project root from Unity Hub: `C:\Users\kugis\Unity_Projects\RobotArm-joint`
- Internet access is required on the first launch so Unity can resolve dependencies from `Packages/manifest.json`
- When using mocopi, put the PC and the mocopi sender device on the same network
- When sending data to a real myCobot, make sure the receiver side is listening on UDP port `7010`

## Project Structure

- Startup scene: `Assets/Scenes/SampleScene.unity`
- mocopi Receiver: `Assets/MocopiReceiver/`
- URDF robot: `Assets/URDF/mycobot_320_pi_2022/`
- Upper-arm-to-robot-joint mapping: `Assets/Scripts/UpperArmToUrdfRobotJoints.cs`
- Unity-to-UDP joint sender: `Assets/Scripts/MyCobotUdpJointSender.cs`
- Build scene: `Assets/Scenes/SampleScene.unity`

## First-Time Setup

1. Add `C:\Users\kugis\Unity_Projects\RobotArm-joint` in Unity Hub.
2. Open it with Unity `6000.0.74f1`.
3. Wait until Package Manager finishes resolving dependencies.
   - `com.unity.robotics.urdf-importer` is loaded from a GitHub URL, so network access is required the first time.
4. Confirm that the Console has no C# compile errors.
5. Open `Assets/Scenes/SampleScene.unity`.
6. Run `Assets > Refresh` to refresh URDF, DAE, and `.meta` import state.

## Scene Check

`SampleScene` contains these main objects:

- `MocopiSimpleReceiver`
  - Receives mocopi UDP data and applies it to a Humanoid avatar.
  - The current receive port is `12351`.
- `MocopiAvatar`
  - The Humanoid model driven by mocopi Receiver.
  - Its `Animator` must be valid as a Humanoid.
- `Mycobot_gripper`
  - The myCobot 320 + gripper robot model imported with URDF Importer.
- `UDP Sender`
  - Has `MyCobotUdpJointSender` attached.
  - The current UDP target is `192.168.1.150:7010`.
  - The send rate is `20 Hz`.

## mocopi Input Setup

1. In the mocopi app, set the destination IP to the IP address of the PC running Unity.
2. Set the mocopi app send port to `12351`.
3. In Unity, confirm that `MocopiSimpleReceiver` has `AvatarSettings[0].Port` set to `12351`.
4. Press Play and confirm that the right arm of `MocopiAvatar` follows mocopi motion.
5. If no motion arrives, allow UDP receiving for Unity Editor in Windows Firewall.

## URDF Robot Check

1. Confirm that `Mycobot_gripper` is visible in the scene.
2. In the Inspector for `UpperArmToUrdfRobotJoints`, confirm the following:
   - `Humanoid Animator` is assigned to the `Animator` of the Humanoid driven by mocopi.
   - `Joint1` through `Joint6` are assigned to the corresponding `ArticulationBody` components.
3. After pressing Play, wait about `2` seconds for automatic calibration.
4. Move your right arm and confirm that J1 through J6 values and the robot model follow the motion.
5. Adjust `j1Scale`, `j1Offset`, `invertJ1`, and related Inspector parameters as needed.

## UDP Transmission Setup

`MyCobotUdpJointSender` on `UDP Sender` sends J1-J6 angles from `UpperArmToUrdfRobotJoints.GetRobotJoints()` as JSON.

The current payload format is:

```json
{"angles":[0,0,0,0,0,0]}
```

Settings:

- `Remote Ip`: IP address of the myCobot controller PC or Raspberry Pi. Current value: `192.168.1.150`.
- `Remote Port`: UDP receive port. Current value: `7010`.
- `Send Continuously`: Sends continuously during Play mode when enabled.
- `Send Rate Hz`: Clamped to a maximum of `20 Hz`.
- `Log Sent Payload`: Enable only when you need to inspect sent JSON in the Console.
- `Include Gripper`: Enable when sending gripper values too. It is currently disabled.

Before moving the real robot, log the received UDP JSON on the receiver side and verify that the `angles` values and order are correct.

## URDF / Mesh Reimport Notes

The gripper-equipped URDF is structured so Unity URDF Importer resolves relative paths from `Assets/URDF/mycobot_320_pi_2022`.

- Place gripper meshes under `Assets/URDF/mycobot_320_pi_2022/Meshes/...`.
- Use `Meshes/...` paths for mesh references inside URDF files.
- If the expected file exists but Unity still reports it as missing, try `Assets > Refresh` and Reimport before changing paths again.
- When copying the same mesh into another location, watch for duplicate `.meta` GUIDs.

## Verification Checklist

- [ ] The project opens with Unity `6000.0.74f1`
- [ ] The Console has no C# compile errors
- [ ] `SampleScene` opens correctly
- [ ] The mocopi send port and Unity receive port both use `12351`
- [ ] `MocopiAvatar` right arm moves during Play mode
- [ ] `UpperArmToUrdfRobotJoints` references the Humanoid `Animator` and J1-J6
- [ ] `Mycobot_gripper` joints follow motion in Unity
- [ ] UDP target `Remote Ip` / `Remote Port` matches the receiver side
- [ ] Receiver-side logs show `{"angles":[...]}` before connecting the real robot

## Troubleshooting

### Package Manager cannot fetch URDF Importer

- Check internet access and GitHub connectivity.
- Confirm that `com.unity.robotics.urdf-importer` in `Packages/manifest.json` is still a Git URL.
- Restart Unity and let Package Manager resolve dependencies again.

### mocopi motion does not reach Unity

- Confirm that the mocopi app destination IP is the PC IP.
- Confirm that the send port is `12351`.
- Allow UDP receiving for Unity Editor in Windows Firewall.
- Confirm that `IsReceivingOnEnable` is enabled on `MocopiSimpleReceiver`.

### Humanoid right-arm bones cannot be read

- Confirm that the target model's `Animator` has a Humanoid Avatar.
- Use a model where `Animator.isHuman` is true.
- Assign the `Animator` of the avatar actually driven by mocopi to `Humanoid Animator` on `UpperArmToUrdfRobotJoints`.

### URDF meshes are missing

- Confirm that the target `.dae` files exist under `Assets/URDF/mycobot_320_pi_2022/Meshes/...`.
- Confirm that URDF mesh references use `Meshes/...`.
- If the files exist, do not keep changing paths; run `Assets > Refresh` and Reimport.

### UDP is sent but the real robot does not move

- Match `Remote Ip` and `Remote Port` on `UDP Sender` to the receiver side.
- Enable `Log Sent Payload` and confirm that JSON appears in the Unity Console.
- On the receiver side, validate the `angles` array order, units, and motion limits.
- On the real-robot control side, prefer a latest-value overwrite control loop instead of queueing synchronous commands.
