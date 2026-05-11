# flexjoint_vs

柔性机械臂视觉伺服控制系统的 Ubuntu 移植版本。原始代码为 Windows/Visual Studio 单文件工程（`chap5.cpp`，约 2000 行），本项目将其重构为模块化 C++17 工程，使用 CMake 构建，支持 Linux 串口（termios）和现代 OpenCV API。

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
│   ├── kinematics.hpp          # 雅可比矩阵计算函数声明
│   ├── controller.hpp          # 控制器函数及参数结构体声明
│   └── vision.hpp              # 视觉特征提取类声明
├── src/
│   ├── main.cpp                # 主程序：初始化、控制主循环
│   ├── serial_port.cpp         # POSIX 串口实现（替代 Windows CreateFile/DCB）
│   ├── modbus_crc.cpp          # CRC-16/MODBUS 校验及速度编码
│   ├── kinematics.cpp          # 快动态/慢动态雅可比矩阵计算
│   ├── controller.cpp          # 双层视觉伺服控制器 + PD 备用控制器
│   └── vision.cpp              # 摄像头采集与 Hough 圆检测
└── data/
    └── .gitkeep                # 运行时保存图像帧的目录
```

---

## 依赖环境

### 操作系统
Ubuntu 20.04 / 22.04（或其他支持 POSIX termios 的 Linux 发行版）

### 依赖库

```bash
sudo apt update
sudo apt install \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev
```

| 库 | 版本要求 | 用途 |
|----|---------|------|
| Eigen3 | ≥ 3.3 | 矩阵运算（雅可比、观测器） |
| OpenCV | ≥ 4.0 | 摄像头采集、Hough 圆检测 |
| yaml-cpp | ≥ 0.6 | 读取配置文件 |
| pthreads | 系统自带 | 线程支持 |

---

## 编译

```bash
cd flexjoint_vs
mkdir build && cd build
cmake ..
make -j4
```

编译成功后生成可执行文件 `build/flexjoint_vs`。

---

## 运行

```bash
# 在 build 目录下运行，传入配置文件路径
./flexjoint_vs ../config/robot_config.yaml
```

按 `Ctrl+C` 可安全退出，程序会在退出前刷新日志文件。

### 串口权限

首次运行前需将当前用户加入 `dialout` 组，否则无法访问串口：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

---

## 配置文件说明

所有参数集中在 [`config/robot_config.yaml`](config/robot_config.yaml)，无需重新编译即可调整。

### serial — 串口配置

```yaml
serial:
  motor_port: "/dev/ttyUSB0"   # 电机控制器串口设备节点
  baud_rate: 115200             # 波特率，与电机控制器一致
```

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
  hough_param1: 100           # HoughCircles Canny 高阈值
  hough_param2: 38            # HoughCircles 圆心累加器阈值（越小检测越多）
  hough_min_radius: 3         # 检测圆的最小半径（像素）
  hough_max_radius: 80        # 检测圆的最大半径（像素）
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

替代原 Windows `CreateFile` / `DCB` / `ReadFile` / `WriteFile` 方案，使用 POSIX `termios` 实现 8N1 串口通信。支持带超时的阻塞读（`select`）。

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

`FeatureExtractor` 类封装摄像头采集和圆检测：
- 修复了原代码中 `VideoCapture` 按值传递的 bug（OpenCV `VideoCapture` 不可廉价拷贝）
- 使用现代 OpenCV 常量（`cv::COLOR_BGR2GRAY`、`cv::HOUGH_GRADIENT` 等，替代已废弃的 `CV_` 前缀常量）
- 每帧检测到的圆按半径排序，输出中圆、最大圆、最小圆的坐标和半径
- 自动保存带标注的图像帧到 `data/frames/`

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
