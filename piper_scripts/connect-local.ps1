param([int]$Samples = 60, [switch]$SkipUsbAttach)
$ErrorActionPreference = 'Stop'
$repoPath = Split-Path -Parent $PSScriptRoot
$profile = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'local-hardware.json') -Raw | ConvertFrom-Json
if (-not $SkipUsbAttach) {
    $devices = ((& usbipd state | Out-String) | ConvertFrom-Json).Devices
    $required = @(
        @($devices | Where-Object { $_.BusId -and $_.InstanceId.EndsWith($profile.adapter_serial) }),
        @($devices | Where-Object { $_.BusId -and $_.InstanceId -like '*VID_8086&PID_0B5C*' }),
        @($devices | Where-Object { $_.BusId -and $_.InstanceId -like '*VID_2BC5&PID_0557*' })
    )
    foreach ($group in $required) {
        if ($group.Count -ne 1) { throw 'Expected exactly one matching CAN adapter, D455 and Dabai camera. Check usbipd list.' }
        $device = $group[0]
        if (-not $device.ClientIPAddress) {
            & usbipd attach --wsl --busid $device.BusId
            if ($LASTEXITCODE -ne 0) { throw "USB/IP attach failed for $($device.Description)" }
        }
    }
}
$canName = $profile.can_name
$identity = & wsl -e udevadm info -q property -p "/sys/class/net/$canName"
if ($identity -notcontains "ID_SERIAL_SHORT=$($profile.adapter_serial)") {
    throw "$canName is not the configured CAN adapter; refusing to use the wrong arm."
}
$linkState = (& wsl -e ip -details link show $canName) -join "`n"
if ($linkState -match 'state DOWN') {
    & wsl -u root -e ip link set $canName type can bitrate $profile.bitrate
    if ($LASTEXITCODE -ne 0) { throw 'Could not configure CAN bitrate.' }
    & wsl -u root -e ip link set $canName up
    if ($LASTEXITCODE -ne 0) { throw 'Could not bring CAN up.' }
} elseif ($linkState -notmatch "bitrate $($profile.bitrate)\b") {
    throw 'The active CAN bitrate differs from the profile.'
}
$linuxRepo = (& wsl -e wslpath -a $repoPath | Out-String).Trim()
& wsl --cd $linuxRepo -e /home/heexu/.venvs/lerobot-piper/bin/python -m piper_scripts.preflight --profile piper_scripts/local-hardware.json --samples $Samples --output outputs/local-preflight-final
if ($LASTEXITCODE -ne 0) { throw 'Preflight failed. See outputs/local-preflight-final/report.json.' }
