$vidPattern = "VID_04D8&PID_00DF|VID_FAED&PID_4870|VID_0483&PID_3748|VID_1366"

$devices = Get-PnpDevice -PresentOnly | Where-Object {
    ($_.InstanceId -match $vidPattern) -and ($_.Class -in @("USB", "USBDevice"))
}

if (-not $devices) {
    Write-Host "[debug] No matching debug probes are currently connected."
    exit 0
}

$rows = foreach ($device in $devices) {
    $candidateSerial = ($device.InstanceId -split "\\")[-1]

    $probeType = switch -Regex ($device.InstanceId) {
        "VID_04D8&PID_00DF" { "CMSIS-DAP" ; break }
        "VID_FAED&PID_4870" { "CMSIS-DAP" ; break }
        "VID_0483&PID_3748" { "ST-Link" ; break }
        "VID_1366"          { "J-Link" ; break }
        default             { "Unknown" }
    }

    [pscustomobject]@{
        ProbeType       = $probeType
        Status          = $device.Status
        Class           = $device.Class
        Name            = if ($device.FriendlyName) { $device.FriendlyName } else { $device.InstanceId }
        CandidateSerial = $candidateSerial
        InstanceId      = $device.InstanceId
    }
}

$rows |
    Sort-Object ProbeType, Name, CandidateSerial |
    Format-Table -AutoSize

Write-Host "[debug] For OpenOCD-based probe selection, try the CandidateSerial value first."
