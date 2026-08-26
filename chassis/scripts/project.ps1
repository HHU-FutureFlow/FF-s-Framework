[CmdletBinding()]
param(
    [ValidateSet("configure", "build", "clean", "download-dap", "download-jlink")]
    [string]$Action = "build",
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Debug"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$BuildDirectory = Join-Path $ProjectRoot "build"
$TargetName = "basic_framework"

function Find-Executable {
    param(
        [Parameter(Mandatory)]
        [string]$Name,
        [string[]]$EnvironmentVariables = @(),
        [string[]]$BundlePatterns = @()
    )

    foreach ($variableName in $EnvironmentVariables) {
        $configuredPath = [Environment]::GetEnvironmentVariable($variableName)
        if ([string]::IsNullOrWhiteSpace($configuredPath)) {
            continue
        }

        if (Test-Path -LiteralPath $configuredPath -PathType Leaf) {
            return (Resolve-Path -LiteralPath $configuredPath).Path
        }

        $candidate = Join-Path $configuredPath $Name
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    $command = Get-Command $Name -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) {
        return $command.Source
    }

    $bundleRoot = Join-Path $env:LOCALAPPDATA "stm32cube\bundles"
    foreach ($pattern in $BundlePatterns) {
        $matches = Get-ChildItem -Path (Join-Path $bundleRoot $pattern) -File -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending
        if ($matches) {
            return $matches[0].FullName
        }
    }

    throw "未找到 $Name。请将它加入 PATH，或设置对应的环境变量：$($EnvironmentVariables -join ', ')。"
}

function Add-ToolDirectoryToPath {
    param([Parameter(Mandatory)][string]$Executable)

    $toolDirectory = Split-Path -Parent $Executable
    $pathEntries = $env:PATH -split [IO.Path]::PathSeparator
    if ($pathEntries -notcontains $toolDirectory) {
        $env:PATH = "$toolDirectory$([IO.Path]::PathSeparator)$env:PATH"
    }
}

function Get-CMake {
    Find-Executable -Name "cmake.exe" -EnvironmentVariables @("CMAKE_PATH") -BundlePatterns @("cmake\*\bin\cmake.exe")
}

function Get-Ninja {
    Find-Executable -Name "ninja.exe" -EnvironmentVariables @("NINJA_PATH") -BundlePatterns @("ninja\*\bin\ninja.exe")
}

function Get-ArmGcc {
    Find-Executable -Name "arm-none-eabi-gcc.exe" -EnvironmentVariables @("ARM_GCC_PATH") -BundlePatterns @("gnu-tools-for-stm32\*\bin\arm-none-eabi-gcc.exe")
}

function Get-BundledTool {
    param([Parameter(Mandatory)][string]$RelativePath)

    $candidate = Join-Path $ProjectRoot $RelativePath
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        return (Resolve-Path -LiteralPath $candidate).Path
    }

    return $null
}

function Reset-StaleCMakeCache {
    $cachePath = Join-Path $BuildDirectory "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cachePath)) {
        return
    }

    $expectedRoot = [IO.Path]::GetFullPath($ProjectRoot).Replace("\", "/").TrimEnd("/")
    $cachedRootLine = Select-String -LiteralPath $cachePath -Pattern "^CMAKE_HOME_DIRECTORY:INTERNAL=" | Select-Object -First 1
    $cachedGeneratorLine = Select-String -LiteralPath $cachePath -Pattern "^CMAKE_GENERATOR:INTERNAL=" | Select-Object -First 1
    $cachedRoot = if ($cachedRootLine) { ($cachedRootLine.Line -split "=", 2)[1].Replace("\", "/").TrimEnd("/") } else { "" }
    $cachedGenerator = if ($cachedGeneratorLine) { ($cachedGeneratorLine.Line -split "=", 2)[1] } else { "" }

    if ($cachedRoot -eq $expectedRoot -and $cachedGenerator -eq "Ninja") {
        return
    }

    Write-Host "检测到工程目录或生成器已变化，正在清理旧 CMake 缓存。"
    $resolvedBuild = [IO.Path]::GetFullPath($BuildDirectory).TrimEnd("\")
    $expectedBuild = [IO.Path]::GetFullPath((Join-Path $ProjectRoot "build")).TrimEnd("\")
    if ($resolvedBuild -ne $expectedBuild -or -not $resolvedBuild.StartsWith([IO.Path]::GetFullPath($ProjectRoot), [StringComparison]::OrdinalIgnoreCase)) {
        throw "拒绝清理非工程 build 目录：$resolvedBuild"
    }

    Remove-Item -LiteralPath $cachePath -Force
    $cmakeFiles = Join-Path $BuildDirectory "CMakeFiles"
    if (Test-Path -LiteralPath $cmakeFiles) {
        Remove-Item -LiteralPath $cmakeFiles -Recurse -Force
    }
    foreach ($generatedFile in @("build.ninja", "rules.ninja", "Makefile", "cmake_install.cmake", "compile_commands.json")) {
        $generatedPath = Join-Path $BuildDirectory $generatedFile
        if (Test-Path -LiteralPath $generatedPath) {
            Remove-Item -LiteralPath $generatedPath -Force
        }
    }
}

function Configure-Project {
    $cmake = Get-CMake
    $ninja = Get-Ninja
    $armGcc = Get-ArmGcc
    Add-ToolDirectoryToPath -Executable $ninja
    Add-ToolDirectoryToPath -Executable $armGcc

    New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
    Reset-StaleCMakeCache

    & $cmake -S $ProjectRoot -B $BuildDirectory -G Ninja "-DCMAKE_BUILD_TYPE=$BuildType" "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0) {
        throw "CMake 配置失败，退出码：$LASTEXITCODE"
    }
}

function Build-Project {
    Configure-Project
    $cmake = Get-CMake
    & $cmake --build $BuildDirectory --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "编译失败，退出码：$LASTEXITCODE"
    }
}

function Clean-Project {
    if (-not (Test-Path -LiteralPath $BuildDirectory)) {
        return
    }

    $resolvedBuild = [IO.Path]::GetFullPath($BuildDirectory).TrimEnd("\")
    $expectedBuild = [IO.Path]::GetFullPath((Join-Path $ProjectRoot "build")).TrimEnd("\")
    if ($resolvedBuild -ne $expectedBuild) {
        throw "拒绝删除非工程 build 目录：$resolvedBuild"
    }
    Remove-Item -LiteralPath $BuildDirectory -Recurse -Force
}

function Download-WithOpenOcd {
    Build-Project
    $openOcd = Get-BundledTool -RelativePath "tools\OpenOCD-20231002-0.12.0\bin\openocd.exe"
    if (-not $openOcd) {
        $openOcd = Find-Executable -Name "openocd.exe" -EnvironmentVariables @("OPENOCD_PATH")
    }
    $config = Join-Path $ProjectRoot "openocd_dap.cfg"
    $binary = Join-Path $BuildDirectory "$TargetName.bin"
    & $openOcd -f $config -c init -c "reset halt" -c "flash write_image erase `"$binary`" 0x08000000" -c reset -c shutdown
    if ($LASTEXITCODE -ne 0) {
        throw "OpenOCD 下载失败，退出码：$LASTEXITCODE"
    }
}

function Download-WithJLink {
    Build-Project
    $jFlash = Find-Executable -Name "JFlash.exe" -EnvironmentVariables @("JFLASH_PATH", "JLINK_PATH")
    $projectFile = Join-Path $ProjectRoot "stm32.jflash"
    $hexFile = Join-Path $BuildDirectory "$TargetName.hex"
    & $jFlash "-openprj$projectFile" "-open$hexFile,0x8000000" -auto -startapp -exit
    if ($LASTEXITCODE -ne 0) {
        throw "J-Flash 下载失败，退出码：$LASTEXITCODE"
    }
}

Push-Location $ProjectRoot
try {
    switch ($Action) {
        "configure" { Configure-Project }
        "build" { Build-Project }
        "clean" { Clean-Project }
        "download-dap" { Download-WithOpenOcd }
        "download-jlink" { Download-WithJLink }
    }
}
finally {
    Pop-Location
}
