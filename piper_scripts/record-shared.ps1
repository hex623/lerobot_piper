param(
    [string]$Task = 'Manual master-slave demonstration',
    [double]$Duration = 30,
    [int]$Fps = 10,
    [string]$Output = ''
)
$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'connect-local.ps1') -Samples 2
$repoPath = Split-Path -Parent $PSScriptRoot
$linuxRepo = (& wsl -e wslpath -a $repoPath | Out-String).Trim()
$recordArgs = @('--profile', 'piper_scripts/local-hardware.json', '--task', $Task, '--duration', "$Duration", '--fps', "$Fps")
if ($Output) { $recordArgs += @('--output', $Output) }
& wsl --cd $linuxRepo -e /home/heexu/.venvs/lerobot-piper/bin/python -m piper_scripts.record_shared_can @recordArgs
if ($LASTEXITCODE -ne 0) { throw 'Shared CAN recording failed; the run folder contains report.json.' }
