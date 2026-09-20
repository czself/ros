# task005 — 运行时验收：多点寻检 + 动态障碍规避

## 目标
在已验证地图 `competition_slam_verified4`（纯 SLAM 图）上，于导航会话运行时验证两项能力：
1. **寻检验收 gate**：随机种子在存活全局代价图可达域内采样 5 个分散点，全部到达（10/10）。
2. **动态障碍规避**：移动障碍进入机器人前进车道时，激光/深度观测在局部代价图标记（cost=100）、机器人制动、障碍移开后恢复并到达目标。

## 方法
- 寻检：`start_patrol.sh competition_slam_verified4` → patrol_controller.py 用固定种子在出生连通分量 BFS 采样（真正 free、离图 0.4m、间距 >1m），5 目标串行 `move_base`。
- 动态障碍：`dynamic_obstacle_test.py` 以 AMCL 位姿/航向为基准，在正前方 2.2m 投放 r=0.25m 圆柱，命令 8m 外目标；采集 /scan 前锥、/move_base/local_costmap/costmap 障碍点（5x5 窗 max）、机器人制动与恢复；随后垂直航向横扫 2 轮（动态），删除后确认目标 SUCCEEDED。

## 证据
- 寻检：`evidence/patrol/`（2 份 controller log、汇总、脚本、RViz 截图）。
- 动态障碍：`evidence/dynamic_obstacle_evidence.json`、`evidence/dynamic_obstacle_test.py`。

## 通过条件（本任务）
- 寻检：两个种子均 5/5 SUCCEEDED。
- 动态障碍：机器人前进 ≥0.8m；在障碍 1.6m 内制动（stopped）；costmap 障碍点 max=100；横扫后恢复；删除后目标 SUCCEEDED。

## 结果
两项均达成，见 review。