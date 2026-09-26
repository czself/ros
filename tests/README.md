# 验证入口

## 离线检查

这些检查针对语法、`navigation/inner_route.yaml` 的路线几何与轨迹证据验收，不需要启动 ROS、Docker 或 Gazebo。

在仓库根目录、使用 Python 3.12 执行：

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-test.txt
git ls-files -z -- 'scripts/*.sh' | xargs -0 -r -n 1 bash -n
python -m compileall -q scripts tests
python -m unittest discover -s tests -p test_calibrated_route.py -v
```

[Offline checks](../.github/workflows/offline-checks.yml) 会在 PR 和推送到 `main` 时运行同样的检查。

路线测试核对当前配置中的 13 段路线、完整车体与白线的碰撞、传感器证据时效、位姿跳变和终点静止状态，并验证缺失完成段记录会被拒绝。它验证离线几何与验收程序，不代表实际机器人已完成比赛任务。

## ROS 红绿灯与白线测试

`test_traffic_light_gate.py` 依赖 ROS1 的 `rospy`、`geometry_msgs`、`gazebo_msgs`、`std_msgs`、`tf`，以及 NumPy 和 OpenCV。

在已安装这些依赖的 ROS1 Noetic 环境中，切换到仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
python3 -m unittest discover -s tests -p test_traffic_light_gate.py -v
```

此测试直接检查速度门控决策，不启动 Gazebo。仿真画面、定位、导航与识别结果仍需按 [工程演示步骤](../README.md) 联调验收。
