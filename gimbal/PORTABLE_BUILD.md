# 可移植构建与下载

工程内不保存任何用户目录、工具版本目录或盘符。源码、链接脚本、调试配置和构建输出都以工程根目录为基准。

## VS Code 使用方式

- `Ctrl+Shift+B`：自动配置并编译。
- 运行任务 `download dap`：编译后使用 OpenOCD 下载。
- 运行任务 `download jlink`：编译后使用 J-Flash 下载。
- 第一次打开工程或移动工程目录后，先运行一次 `configure task` 或直接编译。

`scripts/project.ps1` 会自动检测旧 `build/CMakeCache.txt` 中的工程路径。如果工程被移动或复制，它会删除失效的 CMake 缓存并在当前位置重新生成 `compile_commands.json`。

## 工具查找顺序

构建脚本按以下顺序查找工具：

1. 可选环境变量指定的文件或目录。
2. 系统 `PATH`。
3. STM32 VS Code 扩展安装在 `%LOCALAPPDATA%\stm32cube\bundles` 下的 CMake、Ninja 和 ARM GCC。

支持的可选环境变量：

| 环境变量 | 用途 |
| --- | --- |
| `ARM_GCC_PATH` | `arm-none-eabi-gcc.exe` 文件或其 `bin` 目录 |
| `CMAKE_PATH` | `cmake.exe` 文件或其目录 |
| `NINJA_PATH` | `ninja.exe` 文件或其目录 |
| `OPENOCD_PATH` | `openocd.exe` 文件或其目录 |
| `JFLASH_PATH` | `JFlash.exe` 文件或其目录 |
| `JLINK_PATH` | SEGGER J-Link 工具目录 |

通常只需要安装 STM32 VS Code 扩展；脚本可以直接发现扩展下载的 CMake、Ninja 和 ARM GCC。OpenOCD 或 J-Link 下载工具需要加入 `PATH`，也可以设置上表中的环境变量。

## 命令行使用方式

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action configure
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action clean
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action download-dap
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action download-jlink
```

`compile_commands.json` 本身按照标准会包含当前电脑上的绝对路径，因此它必须由每台电脑本地生成，不能提交到 Git。项目的 `build` 目录已被 `.gitignore` 忽略。
