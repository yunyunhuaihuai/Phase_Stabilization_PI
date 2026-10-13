# Phase Stabilization with Discrete PI

本项目包含 **Red Pitaya 上的离散 PI 闭环补偿实现**，以及 **86100C 独立环外测量的实验使用方式**。Red Pitaya 读取相位鉴别器反馈电压，完成 ODL 归零、工作点建立与极性标定，再通过 VSMD 串口命令调整光学延迟线。86100C 独立观察输出端信号；本人当前实验是在仪器上手动查看测量结果。

用于 PI 的鉴相电压属于**环内反馈**，86100C 对独立输出信号的观测属于**环外测量**。示波器读数不输入 PI，也不参与控制量计算。正式实验可以关闭板端数据记录，直接运行 C++ 程序。

```mermaid
flowchart LR
    subgraph Feedback[反馈控制路径：Red Pitaya + ODL]
        A[相位鉴别器：环内反馈电压] --> B[Red Pitaya ADC 缓冲区均值]
        B --> C[极性修正与离散 PI]
        C --> D[绝对目标 / 位置与步数限制 / 防飞车检查]
        D --> E[VSMD 串口控制]
        E --> F[光学延迟线 ODL]
        F --> G[输出端信号]
        G --> A
    end
    G --> H[86100C：独立环外测量分支]
    H --> I[本人在仪器上手动查看结果]
```

图示表达控制与观测关系，具体实验接线须以实际系统为准。相位与电压的换算需要实验标定；本仓库没有给出环外实测稳定度、机械到位精度或硬实时指标。

## 真实硬件运行：板端控制，86100C 手动观测

在 Red Pitaya 的 Linux 环境中，需要 SDK 的 `rp.h` / `librp` 以及适当调度权限。先按实际设备确认配置与串口路径；`config.txt` 是完整示例，默认参数用于离线演示，不代表具体硬件已验证的设置。

```sh
# 在公开项目根目录构建 Red Pitaya 版本
cmake -S . -B build-rp -DUSE_REAL_HARDWARE=ON -DREDPITAYA_ROOT=/opt/redpitaya
cmake --build build-rp --parallel

# /dev/ttyUSB0 仅为路径示例，替换为实际连接 ODL 驱动器的串口
export PI_SERIAL_DEVICE=/dev/ttyUSB0
# 板端 PI 控制，不保存板端数据；86100C 在仪器上独立手动查看
./build-rp/PhaseStabilization ./config.txt --log-mode off
```

需要保存板端记录时运行：

```sh
export PI_SERIAL_DEVICE=/dev/ttyUSB0
./build-rp/PhaseStabilization ./config.txt --log-mode full
# 省略 --log-mode 也使用 full
```

硬件运行时长由现有 `TOTAL_DURATION_MIN` 决定，可用 Ctrl+C 正常请求退出。`--simulation-seconds` 仅用于离线构建，硬件版本会在初始化设备前拒绝该选项。

环外实验应记录实际使用的 **86100C 测量项、参考方式和测量窗口**，并注明手动查看结果的实验条件。当前仅确认独立观察输出端信号和手动查看这一方式；未提供具体测量设置、接线或实测数值，因此不预设测量指标。已有 Python 仪器工具是可选工具，不是此手动实验的必需步骤。

## 个人贡献范围与控制实现

本人的工作范围聚焦于离散 PI 闭环补偿、ODL 启动工作点建立与极性标定、串口控制集成，以及独立环外验证的实验使用方式整理。公开项目提供相应控制代码、离线仿真、针对性测试和可选 Python 工具。

PI 输出是绝对逻辑目标位置；控制保留死区冻结、积分反算、整数物理目标转换、位置边界、每拍步数限制与原有故障停机。控制频率仍为 20 Hz，积分使用名义 0.05 s。详细配置、保护机制和测量限制见 [设计说明](docs/DESIGN.md)。

## 数据记录与测量语义

| 模式 | 板端行为 | 适用方式 |
|---|---|---|
| `--log-mode full`（默认） | 创建周期 CSV、diagnostics、安全事件 CSV 和 metadata；启动周期日志线程 | 需要保存、下载和分析板端数据的运行及现有 Python runner |
| `--log-mode off` | 不创建上述数据文件，不启动周期日志线程；关键状态、错误与停机信息仍输出终端 | 直接启动 C++，手动使用 86100C 的正式实验 |

`off` 只关闭板端文件记录；ADC 读取、归零与工作点建立、极性标定、PI、积分反算、限幅、串口发送和故障处理保持执行。`full` 的队列满、文件写入与关闭错误处理保持有效。配置文件、`--simulation-seconds` 与 `--log-mode` 可以组合使用，选项顺序不限；没有新增控制配置键。

- 板端 `V_PHS_mV` 是环内反馈电压，`LOCKED` 只表示落入环内死区。
- 板端位置列分别描述 PI 提案、软件接受目标和驱动内部报告，不是独立机械位置测量。
- 离线仿真输出标记为 `simulation`；仿真图和板端电压统计均不代表环外实测结果。
- 86100C 独立观测才属于环外测量，其结果须结合实际测量项、参考方式与窗口解释。

## 可选 Python 工具

`pi_experiment.py` 和已有实验 runner 继续使用默认 `full`，因为它们需要读取、下载和分析板端日志。手动实验可以直接使用上面的 C++ 命令。

`out_of_loop.py` 提供 PI 重复运行、刺激命令账本、板端日志下载与失败清理。可先离线预览：

```sh
python -B tools/out_of_loop.py --dry-run --output-dir ../pi-preview --batch-id preview
```

其 `--execute` 需要明确提供源地址、串口控制台或 SSH 连接、刺激换算系数 `--hz-per-mv`，以及仪器本地记录就绪确认或明确选择已有 LAN 采集。连接默认值为空；可选连接后端可用 `python -m pip install -r requirements.txt` 安装。该工具流程不代表当前手动实验已采用这些设置。

已有 `scope_86100c.py` 可选择进行 LAN 时延查询，`import_scope_native.py` 可归档仪器本地导出，`analyze_out_of_loop.py` 可分析合格 PI 运行。查询时间包络不能证明采集时刻或样本新鲜度；未核验时钟与原始格式时不输出事件对齐指标。这些工具不向 PI 提供反馈。

## 离线验证与仿真演示

不连接硬件的构建和测试（需要 CMake、C++14 编译器及 Python 3）：

```sh
cmake -S . -B build -DUSE_REAL_HARDWARE=OFF -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
python -B -m unittest discover -s tests -p 'test_*.py'

# 完整启动后运行短时 PI；关闭板端数据文件
./build/PhaseStabilization ./config.txt --simulation-seconds 2 --log-mode off
# 同样的仿真，保存板端格式的数据文件
./build/PhaseStabilization ./config.txt --simulation-seconds 2 --log-mode full

# 默认 full 的现有 runner；输出目录应为新目录
python -B tools/pi_experiment.py --binary ./build/PhaseStabilization --output-dir ../pi-demo --seconds 5
# 可选仿真图（需 matplotlib；图仅为仿真）
python -B tools/pi_experiment.py --binary ./build/PhaseStabilization --output-dir ../pi-demo-plot --seconds 5 --plot
# 既有离线步数上限扫描
python -B tools/step_limit_scan.py --binary ./build/PhaseStabilization --output-dir ../pi-scan --limits 5000 5500 --seconds 2
```

Windows 下使用实际生成的 `.exe` 路径；多配置构建通常位于 `build/Release/`。CTest 包含既有 C++ 测试、启动与闭环仿真，以及 `full/off` 参数组合、零文件生成和现有 Python runner 兼容性检查。

本次在 Windows / MinGW 上完成离线构建；7 项既有 C++ 测试、原有启动与闭环仿真、新增 5 项 CLI 检查和 64 项既有 Python 测试全部通过。`off` 检查完成启动与 2 秒闭环后正常退出，工作目录除输入配置外没有生成文件；默认 `full` 的现有 Python runner 能完成日志读取与分析。没有连接真实硬件。

新增的 `.github/workflows/ci.yml` 用于 Linux 离线构建、CTest 和 Python 测试；本次远程 CI 尚未运行。真实硬件通信、SDK ABI、Linux 调度及机械性能不属于本次验证结果。

## 项目结构与使用权限

`control/` 实现 PI；`startup/` 建立工作点；`hal/` 提供硬件与仿真接口；`comm/` 解析驱动帧；`utils/` 提供保护和日志；`tools/` 提供可选 PI 自动化与分析；`tests/` 使用合成数据和 mock 验证。

没有为整个项目指定开源许可证。公开源码的使用、修改和分发权限需由权利人另行确定；第三方依赖遵循各自声明，见 [依赖说明](THIRD_PARTY_NOTICES.md)。
