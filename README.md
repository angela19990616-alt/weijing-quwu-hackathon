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
