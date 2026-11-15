# B70 底盘导航

`b70/nav/` 来自 B70 的 `/root/adam-data/deploy/outputs/nav/`：

- `odom_daemon.py`：订阅 `/chassis/joint_states`，用实测轮矩阵积分位姿；不会发布运动命令。
- `goto.py`：按 `waypoints.json` 的 P0–P5 路点规划“后退到安全线 → 横移 → 前进”，只有加 `--act` 才发布 `/api/chassis/velocity`。
- `waypoints.json`：真机标定路点和手工矢量地图。
- `sim_cruise.py`：离线路线演算。

本地版本新增 `nav_filter.py`，使用三维位姿 EKF：轮编码器作为预测，机器人停在已测量地面路标并按手柄 X 时作为绝对观测。它同时处理两个原实现中的安全缺口：

1. 机器人或 ROS 桥重启时，编码器角度可能归零。不可实现的单帧位移或超过 0.5 秒的数据间隔会被视为流中断，位姿标记为未知，禁止继续自动导航。
2. 赛场路线要求机器人始终朝墙。保存的航向偏差超过 15° 时，启动即把定位标记为未知；`goto.py` 也会拒绝实机行驶，必须先停在路点重新标定。

EKF 不能凭轮速自己消除系统漂移。没有路标、视觉位姿、IMU 或其他绝对观测时，它只能传播不确定度。后续若加入 AprilTag、地面线或双目视觉定位，可调用 `PoseEKF.update_waypoint` 同样的观测更新路径。

`b70/nav_pose.json` 和 `b70/nav_marks.json` 是拉取时的诊断快照，不应部署或提交；部署时由 B70 运行环境生成。
