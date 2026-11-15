# weijing-quwu-hackathon
危境取物黑客松项目

## 本地底盘仿真

`sim/chassis.py` 提供理想平面全向运动：机体坐标 vx/vy/wz 转换到世界坐标，
加速 0.3、减速 0.6（线速度为 m/s²，角速度为 rad/s²），超过 1 秒无新命令后减速停车。
仿真额外限制平移合速度 0.2 m/s、角速度 0.4 rad/s；这些不是厂商实机最大速度。
反向命令先减速至零，非法长度或 NaN/Inf 被拒绝。

可视化程序将官方 MuJoCo 模型的根部设为 mocap，由理想底盘位姿驱动。
双臂保留原有 14 关节与 home 控制目标。没有 ROS 或真机通信，也没有运动指令转发。
底盘不会因墙壁接触自动停止，不模拟轮胎、打滑、载荷或碰撞避障。
模型轴方向尚未与实机标定；仿真返回的位置不代表机器人实测里程计。

### 安装及启动

从 https://www.dexteleop.com/docs/teleavatar-2/resources/mujoco 下载官方模型并解压。
素材保留在仓库外，通过 `--assets` 指向含 `robot.xml` 和 `scene.xml` 的目录。

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-sim.txt
# macOS 使用 mjpython，Linux 可使用 .venv/bin/python
.venv/bin/mjpython sim/play_chassis.py --assets /absolute/path/to/mujoco
# 固定朝向：前进、左移、后退、右移，位置反馈闭环走四边
.venv/bin/mjpython sim/play_chassis.py --assets /absolute/path/to/mujoco --square --side 0.2
```

W/S 前后，A/D 左右，Q/E 转向；按键发送一次速度命令，需重复按键维持运动。
空格取消自动路线并减速停车；R 取消路线并复位。鼠标调整视角。
正方形控制器使用仿真位置反馈，完成后停车；不能作为真机定时运动程序使用。

```bash
python3 -m unittest discover -s tests -v
.venv/bin/python sim/play_chassis.py --assets /absolute/path/to/mujoco --square --headless --duration 60
```

后续接 ROS 时应保持仿真域 90 与真机域 29 隔离，单独实现底盘桥接；
当前未实现 `/api/chassis/velocity` 订阅、轮编码器或导航 TF 输出。

## 取物路线

全程保持初始朝向：左下出发 → 左上 → 右上停车等待抓取确认 → 右下 → 中下停车满 15 秒 → 左下。
中下暂定义为下边中点；宽、长必须由调用者明确提供，仓库没有真机场地尺寸。
图中向上为初始机体 +x，向右为初始机体 -y；实机轴向需现场小幅验证。

```bash
# 以下尺寸仅用于仿真演示，不能当作实机场地测量值
.venv/bin/mjpython sim/play_chassis.py --assets /absolute/path/to/mujoco --pick-route --width 0.4 --length 0.2
```

右上到点后按 G 确认抓取完成；提前按 G 无效。没有实际抓取模型或策略服务调用。
空格取消路线并停车，其他运动键也会切回手动仿真。
停留时间由仿真时钟计量，渲染慢时对应的墙钟时间可能更长。

### 官方代码核对 2026-11-15

- https://github.com/dexteleop/openpi/blob/main/examples/teleavatar_v2/main.py
  使用 ROS 2 客户端收集观测，通过 WebSocket 调用策略，执行动作分块。
- https://github.com/dexteleop/openpi/blob/main/examples/teleavatar_v2/ros2_interface.py
  当前公开 `publish_action` 接受 16 维（左右各 7 关节 + 1 夹爪），发布双臂、夹爪及 FSM 心跳；
  未提供定点底盘导航。比赛部署镜像的 chassis/76 维版本可能不同，不能直接混用。
- https://github.com/dexteleop/openpi/blob/main/examples/teleavatar_v2/README.md
  `zero.py` 会实际移动双臂；不能把它当作只读连通性测试。

底盘文档接口为 `/api/chassis/velocity` 的 `[vx, vy, wz]`，单位 m/s、rad/s。
使用该接口不需自行将速度换成轮速，但可靠到点必须有实际定位反馈。
轮编码器里程计需要轮径、布局、减速比与符号；仅测轮距或累计命令速度不能保证距离准确。
真机仍需检查模式、控制发布者、可用里程计/视觉定位、方向、制动距离、载物后的变化及后方净空。
本项目仅运行本地仿真，不会自动启动官方实机控制示例。

## 实机控制脚本

`robot/zero_arms.py` 是刚才在 B70 真机验证过的双臂预设姿态控制：读取当前
7 关节状态，在 5 秒内线性插值到项目起始姿态，双臂稳定收敛后退出并发送
FSM 停止跟随。运行前必须确认机器人处于 API + 双臂关节模式，双臂活动范围
清空，现场人员能够立即按物理急停。

`robot/chassis_velocity.py` 提供低速、有人监护的底盘短测。它默认只打印命令，
必须显式添加 `--execute` 才会驱动真机；执行前检查 API 模式、底盘使能、状态
新鲜度、命令订阅者和竞争发布者，结束或异常时持续发送 2 秒零速度。

```bash
# 只预览，不移动
python3 robot/chassis_velocity.py --direction right --duration 2

# 真机短测：当前唯一已标定方向是 right
python3 robot/chassis_velocity.py --direction right --duration 2 --execute

# 双臂移动到项目起始姿态（会真实移动）
python3 robot/zero_arms.py
```

2026-11-15 实测：`[vx=+0.05, vy=0, wz=0]` 持续 2 秒，机器人向自身右侧
移动约 3–5 cm 并停下。这个单点结果只用于轴向确认，不能作为长距离比例标定；
`vy`、`wz` 的物理正方向和不同载荷下的距离误差仍需逐项实测。
