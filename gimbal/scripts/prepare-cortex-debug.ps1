[CmdletBinding()]
param(
    [switch]$Quiet,
    [switch]$SkipToolValidation
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$BuildDirectory = Join-Path $ProjectRoot "build"
$VsCodeDirectory = Join-Path $ProjectRoot ".vscode"
$WorkspaceMutex = $null

function Write-Info {
    param([Parameter(Mandatory)][string]$Message)

    if (-not $Quiet) {
        Write-Host "[debug] $Message"
    }
}

function Enter-WorkspaceMutex {
    $sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        $projectBytes = [Text.Encoding]::UTF8.GetBytes($ProjectRoot.ToLowerInvariant())
        $projectHash = ([BitConverter]::ToString($sha256.ComputeHash($projectBytes))).Replace("-", "")
    }
    finally {
        $sha256.Dispose()
    }

    $mutexName = "Global\FFFramework-CortexDebug-$projectHash"
    $script:WorkspaceMutex = New-Object System.Threading.Mutex($false, $mutexName)

    if (-not $script:WorkspaceMutex.WaitOne(30000)) {
        throw "Timed out while waiting for the Cortex-Debug workspace recovery lock."
    }
}

function Exit-WorkspaceMutex {
    if (-not $script:WorkspaceMutex) {
        return
    }

    try {
        $script:WorkspaceMutex.ReleaseMutex() | Out-Null
    }
    catch {
        # Ignore release failures when the lock was not acquired by this instance.
    }
    finally {
        $script:WorkspaceMutex.Dispose()
        $script:WorkspaceMutex = $null
    }
}

function Read-CMakeCacheValue {
    param(
        [Parameter(Mandatory)][string]$CachePath,
        [Parameter(Mandatory)][string]$Key
    )

    $match = Select-String -LiteralPath $CachePath -Pattern "^$([Regex]::Escape($Key))=(.*)$" | Select-Object -First 1
    if (-not $match) {
        return ""
    }

    return $match.Matches[0].Groups[1].Value.Trim()
}

function Validate-WorkspaceTools {
    $requiredTools = @(
        @{
            Name = "OpenOCD"
            Path = Join-Path $ProjectRoot "tools\OpenOCD-20231002-0.12.0\bin\openocd.exe"
        },
        @{
            Name = "Arm GNU GDB"
            Path = Join-Path $ProjectRoot "tools\gcc-arm-none-eabi-10.3-2021.10\bin\arm-none-eabi-gdb.exe"
        },
        @{
            Name = "MinGW make"
            Path = Join-Path $ProjectRoot "tools\mingw64\bin\mingw32-make.exe"
        }
    )

    $missingTools = @()

    foreach ($tool in $requiredTools) {
        if (-not (Test-Path -LiteralPath $tool.Path -PathType Leaf)) {
            $missingTools += "$($tool.Name): $($tool.Path)"
        }
    }

    if ($missingTools) {
        throw "Missing required debug tools. Restore the bundled tools directory before launching Cortex-Debug:`n$($missingTools -join "`n")"
    }
}

function Clear-InvalidCortexDebugState {
    $stateFiles = @(
        (Join-Path $VsCodeDirectory ".cortex-debug.peripherals.state.json"),
        (Join-Path $VsCodeDirectory ".cortex-debug.registers.state.json")
    )

    $removedFiles = @()

    foreach ($stateFile in $stateFiles) {
        if (-not (Test-Path -LiteralPath $stateFile -PathType Leaf)) {
            continue
        }

        $shouldRemove = $false
        $item = Get-Item -LiteralPath $stateFile

        if ($item.Length -eq 0) {
            $shouldRemove = $true
        }
        else {
            try {
                $rawContent = Get-Content -LiteralPath $stateFile -Raw
                if ([string]::IsNullOrWhiteSpace($rawContent)) {
                    $shouldRemove = $true
                }
                else {
                    $null = $rawContent | ConvertFrom-Json -ErrorAction Stop
                }
            }
            catch {
                $shouldRemove = $true
            }
        }

        if ($shouldRemove) {
            try {
                Remove-Item -LiteralPath $stateFile -Force -ErrorAction Stop
                $removedFiles += (Split-Path -Leaf $stateFile)
            }
            catch [System.Management.Automation.ItemNotFoundException] {
                continue
            }
        }
    }

    if ($removedFiles) {
        Write-Info "Removed invalid Cortex-Debug state files: $($removedFiles -join ', ')."
    }
}

function Repair-StaleCMakeCache {
    $cachePath = Join-Path $BuildDirectory "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cachePath -PathType Leaf)) {
        return
    }

    $reasons = @()
    $expectedRoot = [IO.Path]::GetFullPath($ProjectRoot).TrimEnd("\")
    $cachedRoot = Read-CMakeCacheValue -CachePath $cachePath -Key "CMAKE_HOME_DIRECTORY:INTERNAL"

    if (-not [string]::IsNullOrWhiteSpace($cachedRoot)) {
        try {
            $resolvedCachedRoot = [IO.Path]::GetFullPath($cachedRoot).TrimEnd("\")
            if ($resolvedCachedRoot -ne $expectedRoot) {
                $reasons += "cached project root points at $resolvedCachedRoot"
            }
        }
        catch {
            $reasons += "cached project root is invalid"
        }
    }

    foreach ($entry in @(
        @{ Label = "make program"; Key = "CMAKE_MAKE_PROGRAM:FILEPATH" },
        @{ Label = "C compiler"; Key = "CMAKE_C_COMPILER:FILEPATH" },
        @{ Label = "ASM compiler"; Key = "CMAKE_ASM_COMPILER:FILEPATH" }
    )) {
        $value = Read-CMakeCacheValue -CachePath $cachePath -Key $entry.Key
        if ([string]::IsNullOrWhiteSpace($value)) {
            continue
        }

        $looksAbsolute = $value -match '^[A-Za-z]:[\\/]' -or $value -match '^/'
        if ($looksAbsolute -and -not (Test-Path -LiteralPath $value)) {
            $reasons += "cached $($entry.Label) no longer exists: $value"
        }
    }

    if (-not $reasons) {
        return
    }

    $pathsToRemove = @(
        $cachePath,
        (Join-Path $BuildDirectory "CMakeFiles"),
        (Join-Path $BuildDirectory "cmake_install.cmake"),
        (Join-Path $BuildDirectory "Makefile"),
        (Join-Path $BuildDirectory "compile_commands.json")
    )

    foreach ($path in $pathsToRemove) {
        if (-not (Test-Path -LiteralPath $path)) {
            continue
        }

        $item = Get-Item -LiteralPath $path
        if ($item.PSIsContainer) {
            Remove-Item -LiteralPath $path -Recurse -Force
        }
        else {
            Remove-Item -LiteralPath $path -Force
        }
    }

    Write-Info "Removed stale CMake cache metadata: $($reasons -join '; ')."
}

Enter-WorkspaceMutex

try {
    if (-not $SkipToolValidation) {
        Validate-WorkspaceTools
    }

    Clear-InvalidCortexDebugState
    Repair-StaleCMakeCache

    $cleanupScript = Join-Path $PSScriptRoot "cleanup-debug-servers.ps1"
    if (-not (Test-Path -LiteralPath $cleanupScript -PathType Leaf)) {
        throw "Missing cleanup script: $cleanupScript"
    }

    if ($Quiet) {
        & $cleanupScript -Quiet
    }
    else {
        & $cleanupScript
    }
}
finally {
    Exit-WorkspaceMutex
}
