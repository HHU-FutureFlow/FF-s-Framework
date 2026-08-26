[CmdletBinding()]
param(
    [switch]$Quiet
)

$ErrorActionPreference = "Stop"

$names = @(
    "openocd",
    "arm-none-eabi-gdb",
    "JLinkGDBServerCL",
    "JLinkGDBServer",
    "ST-LINK_gdbserver"
)

$debugPorts = @(3333, 4444, 6666)

function Write-Info {
    param([Parameter(Mandatory)][string]$Message)

    if (-not $Quiet) {
        Write-Host "[debug] $Message"
    }
}

$processById = @{}

foreach ($process in (Get-Process -ErrorAction SilentlyContinue | Where-Object { $names -contains $_.ProcessName })) {
    $processById[$process.Id] = $process
}

$getNetTcpConnection = Get-Command Get-NetTCPConnection -ErrorAction SilentlyContinue
if ($getNetTcpConnection) {
    $listeningConnections = Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue |
        Where-Object { $debugPorts -contains $_.LocalPort }

    foreach ($connection in $listeningConnections) {
        if ($connection.OwningProcess -le 0) {
            continue
        }

        $process = Get-Process -Id $connection.OwningProcess -ErrorAction SilentlyContinue
        if (-not $process) {
            continue
        }

        if ($names -contains $process.ProcessName) {
            $processById[$process.Id] = $process
        }
    }
}

if ($processById.Count -eq 0) {
    Write-Info "No stale debug server processes found."
    return
}

$staleProcesses = $processById.Values | Sort-Object ProcessName, Id

if (-not $Quiet) {
    $staleProcesses |
        Select-Object ProcessName, Id |
        Format-Table -AutoSize |
        Out-String |
        Write-Host
}

$staleProcesses | Stop-Process -Force
Write-Info "Cleared stale debug server processes."
