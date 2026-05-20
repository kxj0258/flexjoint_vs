# flexjoint_vs

柔性机械臂视觉伺服控制系统的跨平台 C++17 / CMake 版本。原始代码为 Windows/Visual Studio 单文件工程（`chap5.cpp`，约 2000 行），本项目将其重构为模块化工程，并同时提供 Ubuntu/Linux 与 Windows 的串口实现。

---

## 目录结构

```
flexjoint_vs/
├── CMakeLists.txt              # CMake 构建脚本
├── config/
│   └── robot_config.yaml       # 所有可调参数（串口、相机、控制增益等）
├── include/
│   ├── serial_port.hpp         # 串口类声明
│   ├── modbus_crc.hpp          # Modbus CRC 工具函数声明
│   ├── app_config.hpp          # YAML 配置加载
│   ├── feature_detection.hpp   # 圆形特征检测公共函数
│   ├── motor_client.hpp        # 电机命令/反馈封装
│   ├── kinematics.hpp          # 雅可比矩阵计算函数声明
│   ├── controller.hpp          # 控制器函数及参数结构体声明
│   └── vision.hpp              # 视觉特征提取类声明
├── src/
│   ├── main.cpp                # 主程序：初始化、控制主循环
│   ├── camera_feature_test.cpp # 摄像头实时检测与交互调参
│   ├── camera_calibration.cpp  # 棋盘格相机内参/外参标定
│   ├── motor_test.cpp          # 电机交互测试程序
│   ├── app_config.cpp          # 配置加载实现
│   ├── feature_detection.cpp   # Hough 圆检测实现
│   ├── motor_client.cpp        # 电机帧构造与反馈解码
│   ├── serial_port_posix.cpp   # Ubuntu/Linux 串口实现（termios）
│   ├── serial_port_win32.cpp   # Windows 串口实现（CreateFile/DCB）
│   ├── modbus_crc.cpp          # CRC-16/MODBUS 校验及速度编码
│   ├── kinematics.cpp          # 快动态/慢动态雅可比矩阵计算
│   ├── controller.cpp          # 双层视觉伺服控制器 + PD 备用控制器
│   └── vision.cpp              # 摄像头采集与 Hough 圆检测
└── data/
    └── frames/
        └── .gitkeep            # 运行时保存图像帧的目录
```

---

## 依赖环境

### 操作系统

- Ubuntu 20.04 / 22.04（或其他支持 POSIX termios 的 Linux 发行版）
- Windows 10 / 11（Visual Studio 2019/2022 或其他支持 C++17 的 MSVC 工具链）

### 依赖库

通用依赖：

| 库 | 版本要求 | 用途 |
|----|---------|------|
| Eigen3 | ≥ 3.3 | 矩阵运算（雅可比、观测器） |
| OpenCV | ≥ 4.0 | 摄像头采集、Hough 圆检测 |
| yaml-cpp | ≥ 0.6 | 读取配置文件 |
| Threads | 系统/工具链自带 | 线程支持 |

Ubuntu 安装示例：

```bash
sudo apt update
sudo apt install \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev
```

Windows 推荐使用 vcpkg：

```powershell
vcpkg install eigen3 opencv4 yaml-cpp
```

使用 CMake 配置 Windows 工程时，需要传入 vcpkg toolchain 文件，例如：

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake
```

---

## 编译

### Ubuntu / Linux

```bash
cd flexjoint_vs
mkdir build && cd build
cmake ..
make -j4
```

编译成功后生成：

- `build/flexjoint_vs`：视觉伺服主程序
- `build/camera_feature_test`：摄像头实时检测/调参程序
- `build/camera_calibration`：相机内参/外参标定程序
- `build/motor_test`：电机交互测试程序

### Windows

在 “x64 Native Tools Command Prompt for VS” 或 PowerShell 中执行：

```powershell
cd flexjoint_vs
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake
cmake --build build --config Release
```

编译成功后会生成 `flexjoint_vs.exe`、`camera_feature_test.exe`、`camera_calibration.exe`、`motor_test.exe`。Visual Studio 多配置生成器通常位于 `build\Release\`；Ninja 等单配置生成器通常位于 `build\`。

---

## 运行

### Ubuntu / Linux

```bash
# 在 build 目录下运行，传入配置文件路径
./flexjoint_vs ../config/robot_config.yaml
```

### Windows

```powershell
# Visual Studio 多配置生成器
.\build\Release\flexjoint_vs.exe .\config\robot_config.yaml

# 或在 build 目录下运行
.\Release\flexjoint_vs.exe ..\config\robot_config.yaml
```

按 `Ctrl+C` 可安全退出，程序会在退出前刷新日志文件。

### 论文 Section V-B 实验模式

主程序通过 `experiment.controller_mode` 选择控制器模式。默认值为
`proposed`，即现有双层视觉伺服控制器；不配置 `experiment` 段时也按
`proposed` 运行。

```yaml
experiment:
  controller_mode: "proposed"
```

可选模式：

- `proposed`：完整自适应视觉伺服 + 快子系统振动抑制。
- `proposed_no_fast`：消融试验；仍使用 `cal_joint_vel()`，但在分发层将快子系统增益 `K4` 临时置 0。
- `baseline_pd`：工程基线对比；使用固定内参的 `cal_control()` PD 视觉伺服控制器。
- `baseline_pd_no_fast`：PD 基线并禁用快子系统振动抑制。

`config/` 下提供了可直接运行的完整配置文件。它们与主配置保持相同的
硬件、视觉、目标点和安全参数，只改变 `experiment.controller_mode`：

```bash
cd build

# 完整 proposed 控制器（默认）
./flexjoint_vs ../config/robot_config.yaml

# Section V-B 消融：去掉快子系统振动抑制
./flexjoint_vs ../config/robot_config_proposed_no_fast.yaml

# 工程基线：固定内参 PD 视觉伺服
./flexjoint_vs ../config/robot_config_baseline_pd.yaml

# 工程基线消融：PD 视觉伺服且去掉快子系统
./flexjoint_vs ../config/robot_config_baseline_pd_no_fast.yaml
```

如果要临时调整相机曝光、目标点或安全阈值，建议从对应的
`config/robot_config_*.yaml` 复制一份新文件，只改需要变化的字段，避免
不同实验之间的硬件参数不一致。

做对比或消融试验时，如果担心某个控制器无法收敛，可以设置
`task.max_control_cycles`。该值为 `0` 时不限制循环次数；设为正数时，
主循环处理到指定次数会自动以 `max_control_cycles` 原因退出，并继续执行
发送零速度、回零、关闭视频和写入 `run_summary.md` 的收尾流程。

完成多组实验后，把各次 `data/log/<timestamp>` 填入
`data/experiments/example_manifest.yaml`，再生成对比图和汇总表：

```bash
python3 scripts/analyze_run.py --run data/log/<timestamp> --paper-style
python3 scripts/compare_runs.py data/experiments/example_manifest.yaml --paper-style
```

`compare_runs.py` 会比较图像误差、特征点轨迹、控制输入，并输出快状态
RMS/峰值指标，便于量化消融试验中的振动抑制差异。`baseline_pd` 是项目
已有备用控制器形成的工程基线，不是论文参考文献 [18] 的严格复现。

### 摄像头实时检测与交互调参

```bash
cd build
./camera_feature_test ../config/robot_config.yaml
```

主窗口提供 `dp x10`、`min_dist`、`param1`、`param2`、`min_radius`、`max_radius`、`blur`、`sigma x10`、`equalize` 等 trackbar，可实时优化 Hough 圆检测。Linux 下还会打开 `camera_controls` 窗口，自动列出 V4L2 暴露的可写控制项，例如亮度、对比度、饱和度、白平衡、gamma、gain、sharpness、曝光等。终端会周期性打印三枚特征圆的图像坐标和半径，输出顺序与主控制器一致：中等半径、最大半径、最小半径。

可用 `--video` 指定录制输出文件；不指定时，按 `v` 开始录制会自动生成 `data/videos/camera_feature_test_YYYYmmdd_HHMMSS.mp4`。录制内容为当前带圆检测标注的画面。

```bash
./camera_feature_test ../config/robot_config.yaml --video ../data/test_run.mp4
```

可用 `--show-gray` 启动时直接显示送入 Hough 圆检测的灰度预处理图，也可运行中按 `g` 开关该窗口。

有些控制项会被自动模式锁住。例如 `white_balance_temperature` 需要先把 `white_balance_automatic` 设为 `0`；`exposure_time_absolute` 需要先把 `auto_exposure` 切到手动模式，常见 UVC 摄像头中 `auto_exposure=1` 表示手动、`3` 表示自动/光圈优先。具体含义可用 `v4l2-ctl -d /dev/videoX --list-ctrls-menus` 确认。

快捷键：

- `p`：暂停/继续
- `g`：显示/隐藏送入 Hough 圆检测的灰度预处理图
- `s`：保存当前带标注图像到 `data/frames/`
- `v`：开始/停止录制 MP4 视频
- `w`：保存当前检测参数和相机控制项到 `data/vision_tuned.yaml`
- `r`：在终端打印当前参数
- `c`：在终端打印当前相机控制项、范围和 inactive 状态
- `q` 或 `Esc`：退出

### 相机内参/外参标定

标定程序使用棋盘格角点。`--cols` 和 `--rows` 是棋盘格内角点数量，不是方格数量。

```bash
cd build
./camera_calibration ../config/robot_config.yaml \
  --cols 8 --rows 11 --square 0.015 \
  --samples 20 \
  --output ../data/camera_calibration.yaml
```

快捷键：

- `Space` 或 `c`：采集一帧棋盘格样本
- `k`：根据已采集样本计算内参
- `e`：用当前可见棋盘格求外参
- `s`：保存标定结果
- `q` 或 `Esc`：退出

输出 YAML 包含 `camera_intrinsics: [fx, fy, cx, cy]`、`distortion_coeffs` 和 `camera_extrinsics`。外参是“棋盘格/世界坐标系到相机坐标系”的 3×4 矩阵；若要直接写回 `robot.camera_extrinsics`，请确保棋盘格坐标系与机器人基坐标系定义一致。

只做外参时可以复用已有内参：

```bash
./camera_calibration ../config/robot_config.yaml \
  --extrinsic-only \
  --intrinsics ../data/camera_calibration.yaml
```

### 电机交互测试

```bash
cd build
./motor_test ../config/robot_config.yaml
```

启动后进入 `motor>` 命令行。常用命令：

- `vel <rad_s>`：发送速度指令
- `stop`：发送零速度
- `read [timeout_ms]` / `readsys [timeout_ms]`：发送 `0x0B` 并读取系统实时数据
- `read2f [timeout_ms]`：发送 `0x2F` 并读取旧编码器反馈
- `monitor on|off`：开关后台反馈显示
- `pos <rad>`：发送位置指令（需要先配置位置命令字）
- `pid <kp> <ki> <kd>`：发送 PID 参数（需要先配置 PID 命令字）
- `frame <cmd_hex> [bytes...]`：按项目的 `0x3E ... CRC` 帧格式构造并发送
- `raw <bytes...>`：发送原始字节
- `set poscmd|pidcmd|velcmd|feedback <hex|-1>`：运行时设置命令字
- `set address|sequence <hex>`、`set strictaddr|consume|debug on|off`：运行时调整协议调试参数
- `set zerodeg|counts|pidscale <value>`：运行时设置编码器零点、位置每圈计数、PID 缩放

反馈显示包含电机绝对位置、按 `encoder_zero_offset_deg` 换算后的关节位置和速度。`read`/`monitor` 使用 `0x0B`，`read2f` 保留用于旧 `0x2F` 反馈调试；启用 `set debug on` 后会打印被跳过帧的命令码、payload 长度和 CRC/地址统计。

### 串口权限与端口名

Ubuntu 首次运行前需将当前用户加入 `dialout` 组，否则无法访问串口：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

Windows 下使用设备管理器中的串口名，例如 `COM8`。程序会自动把 `COM8` 转换为 Win32 API 需要的 `\\.\COM8` 形式。

---

## 配置文件说明

所有参数集中在 [`config/robot_config.yaml`](config/robot_config.yaml)，无需重新编译即可调整。

### serial — 串口配置

```yaml
serial:
  motor_port: "/dev/ttyUSB0"   # 兼容旧配置的默认串口
  linux_port: "/dev/ttyUSB0"   # Ubuntu/Linux 串口设备节点
  windows_port: "COM8"         # Windows 串口名
  baud_rate: 115200            # 波特率，与电机控制器一致
```

如果 `linux_port` 或 `windows_port` 存在，程序会按当前操作系统自动选择；否则使用 `motor_port`。

### camera — 摄像头配置

```yaml
camera:
  index: 0      # /dev/video0，若有多个摄像头可改为 1、2 等
  fps: 110      # 目标帧率
  width: 640    # 分辨率宽
  height: 480   # 分辨率高
```

### camera_controls — 相机驱动控制项

```yaml
camera_controls:
  auto_exposure: 1
  exposure_time_absolute: 157
  white_balance_automatic: 0
  white_balance_temperature: 4600
  gain: 70
  brightness: 16
```

- 仅 Linux 下通过 V4L2 自动应用；Windows 会忽略这段并打印提示。
- 程序在打开相机后会按 YAML 中的书写顺序依次设置，因此像 `auto_exposure`、`white_balance_automatic` 这种模式开关建议写在依赖它们的具体数值前面。
- `camera_feature_test` 按 `w` 保存的 `camera_controls` 可直接复制回 `config/robot_config.yaml`。

### control — 控制增益

```yaml
control:
  K1: 1.42          # 慢动态层阻尼增益
  B: 0.000049       # 慢动态层图像误差增益
  K2: 0.001         # 相机参数自适应增益
  Gamma: 0.0001     # 自适应律学习率
  K4: 2.7           # 快动态层控制增益
  eps: 0.1          # 快动态层奇异值保护参数
  Kq: 50.0          # 电机刚度系数
  cmd_c1: 1.0       # 速度指令混合系数1
  cmd_c2: 0.325     # 速度指令混合系数2（快/慢层权重）
  K_dtk_I: 0.8      # 积分器极点
  omega_dtk: 0.6    # 积分器前馈系数
  h: 0.04           # RK4 积分步长（秒）
  Mr: 1.0           # 连杆等效质量
  Jm: 1.0           # 电机等效转动惯量
  Kp: 5.0           # PD 备用控制器比例增益
  Kd: 0.35          # PD 备用控制器微分增益
  Ps: [10.0, 15.0, 10.0, 6.0]              # 观测器增益向量
  mu_rho: [0.01, 0.0001, 0.01, 0.001, 0.001]  # 鲁棒项自适应学习率
```

### vision — 视觉参数

```yaml
vision:
  desired_coords: [264.5, 96.5, 298.5, 166.5, 174.5, 144.5]
  # 三个特征点的期望图像坐标 [u1,v1, u2,v2, u3,v3]（像素）
  save_path: "data/frames/"   # 图像帧保存路径（相对于运行目录）
  hough_dp: 1.0               # Hough 累加器分辨率比例
  hough_min_dist: 30.0        # 圆心之间的最小距离（像素）
  hough_param1: 100           # HoughCircles Canny 高阈值
  hough_param2: 38            # HoughCircles 圆心累加器阈值（越小检测越多）
  hough_min_radius: 3         # 检测圆的最小半径（像素）
  hough_max_radius: 80        # 检测圆的最大半径（像素）
  blur_kernel: 7              # 高斯滤波核大小，偶数会自动提升为奇数
  blur_sigma: 2.0             # 高斯滤波 sigma
  equalize_hist: false        # 是否对灰度图做直方图均衡
```

### motor_protocol — 电机测试协议配置

```yaml
motor_protocol:
  address: 0x01                 # 电机设备地址，出厂默认 0x01
  sequence: 0x00                # 包序号；电机应答会回显该值
  feedback_command: 0x0B        # 主程序默认读取系统实时数据反馈
  strict_address: false         # true 时丢弃地址不匹配的合法帧
  consume_command_response: true # 发送 0x54/0x55 后短超时消费命令应答
  debug_frames: false           # true 时打印跳过帧、CRC 错误等调试信息
  velocity_command: 0x54       # 已知速度指令命令字
  position_command: 0x55       # 绝对位置指令命令字，-1 表示禁用
  pid_command: -1              # PID 参数指令命令字，-1 表示禁用
  position_counts_per_rev: 16384
  pid_scale: 1000
```

RS485 V2.3 帧格式为 `header, seq, address, command, payload_len, payload, crc16_low, crc16_high`。主机发送 header 为 `0x3E`，电机应答 header 为 `0x3C`；`0x0B` 系统实时数据反馈包含单圈绝对值、多圈绝对值、机械速度、电压、电流、温度、故障码和运行状态。主程序默认只使用 `0x0B`；`0x2F` 仍保留给 `motor_test read2f` 或显式配置。

### task — 任务停止条件与安全参数

```yaml
task:
  enable_completion_check: true
  image_error_tolerance_px: 5.0
  velocity_tolerance_rad_s: 0.03
  stable_frames_required: 30
  max_runtime_s: 120.0
  max_control_cycles: 0
  velocity_saturation_rad_s: 1.5
  min_safe_angle_rad: -1.0
  max_safe_angle_rad: 1.0
  vision_boundary_margin_px: 25.0
```

`max_runtime_s` 按运行时间限制实验，`max_control_cycles` 按主控制循环次数
限制实验。两者都大于 0 时任一条件先满足都会触发自动退出。建议做不收敛
风险较高的消融或基线实验时设置 `max_control_cycles`，例如 `600` 或
`1000`；保留为 `0` 则只依赖收敛判断、运行时间和安全停止条件。

### experiment — 实验控制器模式

```yaml
experiment:
  controller_mode: "proposed"
```

`controller_mode` 用于复现实验中的完整控制、快子系统消融和基线对比。
支持 `proposed`、`proposed_no_fast`、`baseline_pd`、
`baseline_pd_no_fast`。其中 `*_no_fast` 不会改动 YAML 中的 `control.K4`，
而是在控制器分发层复制一份 `ControlParams` 并临时置零，方便同一份配置
在多组实验之间保持可比性。

### robot — 机器人物理参数

```yaml
robot:
  L: 0.3                          # 连杆长度（米）
  rt_e1: [0.06, -0.055, 0.0]      # 特征点1相对末端偏移（米）
  rt_e2: [0.075, 0.075, 0.0]      # 特征点2相对末端偏移（米）
  initial_angle_rad: -0.185       # 初始关节角（弧度）
  encoder_zero_offset_deg: 134.539  # 编码器零点偏移（度）
  camera_intrinsics: [487.05, 487.05, 338.23, 231.89]  # fx, fy, cx, cy
  camera_extrinsics: [...]        # 相机外参矩阵（行主序 3×4）
```

零点换算公式：

```text
joint_position_rad = (single_turn_deg - encoder_zero_offset_deg) * pi / 180
encoder_zero_offset_deg = single_turn_deg - q0 * 180 / pi
```

第二行用于把当前姿态标定为期望关节角 `q0`。例如希望当前姿态为 `initial_angle_rad`，先用 `motor_test read 1000` 读取 `single_turn_deg`，再按公式更新 `encoder_zero_offset_deg`。主程序首次读到真实反馈后，如果它与 `initial_angle_rad` 差异过大或已超出安全限位，会打印明确提示并在越限时发送 0 速度。

---

## 各模块说明

### `serial_port` — 串口通信

封装跨平台 8N1 串口通信，主程序只依赖统一的 `SerialPort` 接口：

- Ubuntu/Linux：`serial_port_posix.cpp` 使用 POSIX `termios`、`select`、`read`、`write`
- Windows：`serial_port_win32.cpp` 参考原始 `chap5.cpp` 的 `CreateFile` / `DCB` / `ReadFile` / `WriteFile` 方案

原始 Windows 代码中存在两个串口句柄（`RobotCOM` 类 + `hCom`），实际控制循环只用 `hCom`，本模块将两者统一为一个 `SerialPort` 类。同时删除了原 `ThreadComm` 线程（仅轮询缓冲区大小，无实际作用）。

### `modbus_crc` — CRC 校验与速度编码

- `crc16_modbus()`：CRC-16/MODBUS 校验，用于电机控制器 RTU 帧校验
- `velocity_to_bytes()`：将浮点 RPM 值编码为两字节补码格式，供 Modbus 帧使用
- 删除了原代码中的 `hexstr[256]` 查找表（该表是恒等映射，无实际作用）

### `kinematics` — 运动学雅可比

- `cal_jacobian_fast()`：快动态雅可比矩阵，考虑关节速度对末端位置的影响
- `cal_jacobian_slow()`：慢动态（准静态）雅可比矩阵

两个函数均为纯函数，无副作用，输出 41 元素数组（前 36 元素为雅可比行，后 5 元素为预测图像坐标）。

### `controller` — 控制算法

- `cal_joint_vel()`：主控制器，双层视觉伺服 + 在线参数自适应 + RK4 数值积分
  - 慢动态层：基于图像误差的力矩计算
  - 快动态层：基于观测器状态的补偿力矩
  - 自适应律：相机内参（theta）和鲁棒项（rho）在线更新
  - 观测器：四阶状态观测器，RK4 积分
- `cal_control()`：备用 PD 控制器（Luca 方法），结构更简单，用于对比实验

所有增益通过 `ControlParams` 结构体传入，不再硬编码。

### `vision` — 视觉特征提取

`FeatureExtractor` 类封装摄像头采集和圆检测；底层圆检测逻辑由 `feature_detection` 公共模块提供，主程序和 `camera_feature_test` 使用同一套检测代码：
- 修复了原代码中 `VideoCapture` 按值传递的 bug（OpenCV `VideoCapture` 不可廉价拷贝）
- 使用现代 OpenCV 常量（`cv::COLOR_BGR2GRAY`、`cv::HOUGH_GRADIENT` 等，替代已废弃的 `CV_` 前缀常量）
- 每帧检测到的圆按半径排序，输出中圆、最大圆、最小圆的坐标和半径
- 自动保存带标注的图像帧到 `data/frames/`

### `motor_client` — 电机命令与反馈

封装 RS485 V2.3 帧构造、CRC-16/MODBUS 校验、动态长度应答读取、速度命令和反馈解析。读取期望命令时会在超时时间内跳过 CRC 正确但命令码不匹配的帧，例如速度命令 `0x54` 的应答或迟到的旧反馈帧；`flexjoint_vs` 和 `motor_test` 共用该模块，避免主程序和测试程序使用不同的速度换算或反馈解码逻辑。

### `main` — 主程序

1. 解析命令行参数，加载 YAML 配置
2. 注册 `SIGINT`/`SIGTERM` 信号处理，确保 `Ctrl+C` 时日志正常刷新
3. 初始化串口和摄像头
4. 暖机：丢弃前 5 帧图像
5. 进入控制主循环：
   - 记录当前状态到 `dataFile.txt`
   - 视觉特征提取
   - 调用控制器计算速度指令
   - 速度饱和限幅（±1.5 rad/s）
   - 发送 Modbus RTU 帧到电机
   - 读取 `0x0B` 系统实时数据反馈，解码关节角和角速度
   - 更新状态向量

---

## 输出文件

### `dataFile.txt`

每个控制周期写入一行 CSV，包含原始反馈、图像点、26 维控制状态、发送到电机的速度指令和有效性标志。主要控制状态列含义如下：

| 列索引 | 含义 |
|--------|------|
| `state_joint_angle_rad` | 控制状态中的关节角（rad） |
| `state_joint_velocity_rad_s` | 控制状态中的关节角速度（rad/s） |
| `state_img_u1`–`state_img_v3` | 三个特征点图像坐标（像素） |
| `state_theta_0`–`state_theta_3` | 自适应相机参数 theta[4] |
| `state_rho_0`–`state_rho_4` | 鲁棒项参数 rho[5] |
| `state_obs_0`–`state_obs_3` | 观测器状态 obs[4] |
| `state_qc` | 积分器状态 qc |
| `state_velocity_command_rad_s` | 控制状态中的速度指令（rad/s） |
| `state_tau` | 总力矩 tau |
| `state_tau_s` | 慢动态力矩 tau_s |
| `state_tau_f_c` | 快动态力矩 tau_f_c |

### `data/frames/`

每个控制周期保存一张带标注的图像（蓝色圆为检测结果，红色圆为期望位置）。

---

## 常见问题

**Q: 编译时找不到 Eigen**
```bash
# 部分系统 Eigen 安装在 /usr/include/eigen3，需要软链接
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

**Q: 摄像头打开失败**
```bash
# 检查设备节点
ls /dev/video*
# 修改 config/robot_config.yaml 中的 camera.index
```

**Q: 串口打开失败（Permission denied）**
```bash
sudo usermod -aG dialout $USER
# 重新登录后再运行
```

**Q: 检测不到 3 个圆**

调整 `config/robot_config.yaml` 中的 `hough_param2`（减小该值可检测到更多圆），或调整 `hough_min_radius` / `hough_max_radius` 匹配实际标记尺寸。
