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

有些控制项会被自动模式锁住。例如 `white_balance_temperature` 需要先把 `white_balance_automatic` 设为 `0`；`exposure_time_absolute` 需要先把 `auto_exposure` 切到手动模式，常见 UVC 摄像头中 `auto_exposure=1` 表示手动、`3` 表示自动/光圈优先。具体含义可用 `v4l2-ctl -d /dev/videoX --list-ctrls-menus` 确认。

快捷键：

- `p`：暂停/继续
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
  --cols 9 --rows 6 --square 0.025 \
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
- `read [timeout_ms]`：读取一次反馈
- `monitor on|off`：开关后台反馈显示
- `pos <rad>`：发送位置指令（需要先配置位置命令字）
- `pid <kp> <ki> <kd>`：发送 PID 参数（需要先配置 PID 命令字）
- `frame <cmd_hex> [bytes...]`：按项目的 `0x3E ... CRC` 帧格式构造并发送
- `raw <bytes...>`：发送原始字节
- `set poscmd|pidcmd|velcmd <hex|-1>`：运行时设置命令字
- `set zerodeg|counts|pidscale <value>`：运行时设置编码器零点、位置每圈计数、PID 缩放

反馈显示包含电机绝对位置、按 `encoder_zero_offset_deg` 换算后的关节位置和速度。当前仓库原始代码只明确给出了速度命令 `0x54` 和 15 字节反馈解析；位置/PID 命令字默认禁用，需要按实际电机控制器协议在 `motor_protocol` 中确认后开启。

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
  velocity_command: 0x54       # 已知速度指令命令字
  position_command: -1         # 位置指令命令字，-1 表示禁用
  pid_command: -1              # PID 参数指令命令字，-1 表示禁用
  position_counts_per_rev: 16384
  pid_scale: 1000
```

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

封装速度命令帧、通用命令帧、原始字节发送和 15 字节编码器反馈解析。`flexjoint_vs` 和 `motor_test` 共用该模块，避免主程序和测试程序使用不同的速度换算或反馈解码逻辑。

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
   - 读取编码器反馈，解码关节角和角速度
   - 更新状态向量

---

## 输出文件

### `dataFile.txt`

每个控制周期写入一行，共 26 列，格式为空格分隔的浮点数：

| 列索引 | 含义 |
|--------|------|
| 0 | 关节角（rad） |
| 1 | 关节角速度（rad/s） |
| 2–7 | 三个特征点图像坐标 u1,v1,u2,v2,u3,v3（像素） |
| 8–11 | 自适应相机参数 theta[4] |
| 12–16 | 鲁棒项参数 rho[5] |
| 17–20 | 观测器状态 obs[4] |
| 21 | 积分器状态 qc |
| 22 | 速度控制指令（rad/s） |
| 23 | 总力矩 tau |
| 24 | 慢动态力矩 tau_s |
| 25 | 快动态力矩 tau_f_c |

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
