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

$hasCmsisDap = $rows | Where-Object { $_.ProbeType -eq "CMSIS-DAP" } | Select-Object -First 1
$hasSerialSelectableProbe = $rows | Where-Object { $_.ProbeType -in @("J-Link", "ST-Link") } | Select-Object -First 1

if ($hasCmsisDap) {
    Write-Host "[debug] CMSIS-DAP note: the bundled OpenOCD now uses the HID backend by default."
    Write-Host "[debug] For ATK-HS-V3-style probes, start the default CMSIS-DAP launch and leave adapter serial blank."
    Write-Host "[debug] Do not enter the Windows composite ID (for example ATK_20190528) into the CMSIS-DAP serial prompt."
}

if ($hasSerialSelectableProbe) {
    Write-Host "[debug] For J-Link or ST-Link, use the CandidateSerial value only when more than one probe of that type is connected."
}
