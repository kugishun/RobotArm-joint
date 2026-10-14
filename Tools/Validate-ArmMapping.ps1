param(
    [string]$UnityData = 'C:/Program Files/Unity/Hub/Editor/6000.0.74f1/Editor/Data'
)
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force Logs | Out-Null
    $refs = @(Get-ChildItem "$UnityData/NetStandard/ref/2.1.0" -Filter *.dll) +
        @(Get-ChildItem "$UnityData/Managed/UnityEngine" -Filter *.dll)
    $compileArgs = @('-nologo', '-target:library', '-langversion:9.0', '-out:Logs/ArmMappingCompileCheck.dll') +
        @($refs | ForEach-Object { '-r:"' + $_.FullName + '"' }) +
        @('Assets/Scripts/UpperArmToUrdfRobotJoints.cs', 'Assets/Scripts/MyCobotUdpJointSender.cs', 'Assets/Scripts/ArmPoseSolver.cs', 'Assets/Scripts/HandPoseIkController.cs', 'Assets/Scripts/HandPoseIkController.Workspace.cs', 'Assets/Scripts/RobotControlMenu.cs', 'Assets/Editor/ArmMappingValidation.cs', 'Assets/Editor/ArmIkValidation.cs')
    $compileArgs += 'Assets/Scripts/HandPoseIkController.References.cs'
    $compileArgs | Set-Content Logs/ArmMappingCompileCheck.rsp
    & "$UnityData/NetCoreRuntime/dotnet.exe" "$UnityData/DotNetSdkRoslyn/csc.dll" '@Logs/ArmMappingCompileCheck.rsp'
    if ($LASTEXITCODE -ne 0) { throw 'Unity-reference compilation failed.' }

    # Independent numerical checks, not a substitute for executing the Unity scripts.
    $results = [System.Collections.Generic.List[string]]::new()
    foreach ($rate in @(30, 60, 120)) {
        $alpha = 1 - [Math]::Pow(0.8, 60.0 / $rate)
        $response = 80 * (1 - [Math]::Pow(1 - $alpha, $rate / 10))
        $expected = 80 * (1 - [Math]::Pow(0.8, 6))
        if ([Math]::Abs($response - $expected) -gt 0.00001) { throw 'Frame-rate invariance failed.' }
        $results.Add("Numerical step response at ${rate}Hz, t=0.1s: $response deg")
    }
    $difference = -179 - 179
    $wrapped = (($difference % 360) + 360) % 360
    if ($wrapped -gt 180) { $wrapped -= 360 }
    if ($wrapped -ne 2) { throw 'Elbow boundary check failed.' }
    $results.Add("Elbow calibration 179 -> -179: legacy=$difference deg, corrected=$wrapped deg")

    $scene = Get-Content Assets/Scenes/SampleScene.unity -Raw
    foreach ($index in 1..6) {
        $id = [regex]::Match($scene, "(?m)^  joint${index}: \{fileID: (\d+)\}").Groups[1].Value
        $body = [regex]::Match($scene, '(?ms)^--- !u!171741748 &' + $id + '\r?\n.*?(?=^---|\z)').Value
        if (-not $body -or $body -notmatch 'm_ArticulationJointType: 2') { throw "J$index not assigned to revolute body." }
        $driveMin = [double]::Parse([regex]::Match($body, 'lowerLimit: ([-\d.]+)').Groups[1].Value, [cultureinfo]::InvariantCulture)
        $driveMax = [double]::Parse([regex]::Match($body, 'upperLimit: ([-\d.]+)').Groups[1].Value, [cultureinfo]::InvariantCulture)
        $mappingMin = [double]::Parse([regex]::Match($scene, "(?m)^  j${index}Min: ([-\d.]+)").Groups[1].Value, [cultureinfo]::InvariantCulture)
        $mappingMax = [double]::Parse([regex]::Match($scene, "(?m)^  j${index}Max: ([-\d.]+)").Groups[1].Value, [cultureinfo]::InvariantCulture)
        $min = [Math]::Max($mappingMin, $driveMin)
        $max = [Math]::Min($mappingMax, $driveMax)
        if ($min -gt $max) { throw "J$index has disjoint limits." }
        $results.Add("J$index mapping=[$mappingMin, $mappingMax], drive=[$driveMin, $driveMax], intersection=[$min, $max]")
    }
    # Analytic counterexample: upper arm elevation 45 deg; bend in its vertical plane by 100 deg.
    # Forearm elevation becomes 145 deg, its horizontal projection reverses by 180 deg.
    $forearmHorizontal = [Math]::Cos(145 * [Math]::PI / 180)
    if ($forearmHorizontal -ge 0) { throw 'Expected horizontal reversal was not reproduced.' }
    $results.Add('Known mapping limitation: planar elbow bend across vertical reverses forearm yaw by 180 deg; J4 is coupled.')
    $results.Add('Compilation, numerical calculations and serialized scene audit only. Unity runtime tests not executed by this script.')
    $results | Set-Content -Encoding utf8 Logs/ArmMappingStaticValidation.txt
    $results
}
finally { Pop-Location }



