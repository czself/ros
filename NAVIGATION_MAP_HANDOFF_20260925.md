# 导航地图对接说明

更新时间：2026-09-25

## 1. 当前确定使用的导航地图

后续导航统一使用“之前完整 SLAM 扫图结果 + 白线障碍叠加”的地图，不重新扫图，不使用当前实时 `/map` 局部图。

项目内固定文件：

- `maps/current_slam_preview_white_lines.yaml`
- `maps/current_slam_preview_white_lines.pgm`
- `maps/current_slam_preview_white_lines.manifest.json`

根目录仍保留来源文件：

- `current_slam_preview.pgm`：之前保存的完整 SLAM 图
- `current_slam_preview_local.yaml`：来源图元数据
- `current_slam_preview_white_lines.png`：人工查看用预览图

## 2. 地图参数

```yaml
image: current_slam_preview_white_lines.pgm
resolution: 0.050000
origin: [-6.000000, -6.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

栅格尺寸为 `224 x 224`，分辨率 `0.05 m/cell`，ROS 栅格画布为 `11.2 x 11.2 m`，原点为 `(-6, -6)`。实际赛场的 `4.2 x 4.2 m` 区域位于该画布内部；地图外灰色未知区域不作为导航目标区域。

## 3. 来源及白线校验

`maps/current_slam_preview_white_lines.manifest.json` 已登记为：

```json
{
  "kind": "navigation_real_white_line_overlay",
  "source_map": "/home/sz/game/current_slam_preview.pgm",
  "source_yaml": "/home/sz/game/current_slam_preview_local.yaml",
  "permanent_white_line_cells": 1711
}
```

来源 SLAM 图 SHA256：

```text
bdcd8feb0e64905ccbcd157ea0392570244801e5e698852b0ea462a7293a77cf
```

启动脚本会校验来源图哈希以及白线占用栅格数量。白线叠加图不是重新建图结果，而是对之前那张完整 SLAM 图增加永久白线占用单元。

## 4. 当前 ROS 状态

容器：`ros1_modeling`

Gazebo 正在运行。当前固定地图已复制到容器：

```text
/root/ros1_ws/maps/current_slam_preview_white_lines.yaml
/root/ros1_ws/maps/current_slam_preview_white_lines.pgm
/root/ros1_ws/maps/current_slam_preview_white_lines.manifest.json
```

预览用的 `/combined_map_server` 已切换到该 YAML；当前 `/combined_map/info` 应为：

```text
resolution: 0.05
width: 224
height: 224
origin: (-6.0, -6.0, 0.0)
```

注意：当前系统里仍可能有旧的 `/slam_gmapping` 进程。正式启动 AMCL + move_base 导航前，应停止旧 GMapping，避免动态 `/map` 与固定 map_server 的 TF/话题干扰。导航栈应由 `scripts/start_navigation.sh` 启动自己的固定 `map_server`、AMCL 和 move_base。

## 5. 导航启动入口

`scripts/start_standee_photo_route.sh` 的默认地图已改为：

```text
/root/ros1_ws/maps/current_slam_preview_white_lines.yaml
```

直接启动前，建议显式传参，避免窗口或环境变量带入其他地图：

```bash
cd /home/sz/game
ENFORCE_WHITE_LINES=true \
./scripts/start_standee_photo_route.sh \
  /root/ros1_ws/maps/current_slam_preview_white_lines.yaml
```

脚本内部会将主机的 `maps/` 文件复制到容器的 `/root/ros1_ws/maps/`，并在启动前验证 manifest。

如果只需要先启动导航栈而不跑拍照路线：

```bash
cd /home/sz/game
ENFORCE_WHITE_LINES=true \
./scripts/start_navigation.sh \
  maps/current_slam_preview_white_lines.yaml
```

## 6. 10 点拍照路线

路线配置：

```text
navigation/standee_photo_route.json
navigation/route_contract.yaml
```

照片验收按用户确认的画面内容逐点核对：POINT_1 红绿灯；POINT_2、POINT_3、POINT_4 分别 3 个人；POINT_5 4 个人；POINT_6 5 个人；POINT_7 红绿灯；POINT_8、POINT_9、POINT_10 为车牌。人物须全身入镜，红绿灯灯体和车牌须完整、清晰入镜。照片必须在记录的拍照位和拍摄航向到位后拍摄，不能用较宽的导航容差替代指定取景角度。

固定拍照点为 `POINT_1` 到 `POINT_10`，拍照路线脚本会：

1. 启动固定地图导航栈；
2. 依次前往 10 个点；
3. 每个点等待并拍摄照片；
4. 执行 `HOME` 返回出生点；
5. 做精确停车检查并写入 `run_summary.json`。

出生点来自路线契约：

```yaml
x: 1.714860
y: -1.599947
yaw: 1.606236
```

照片默认写入容器：

```text
/root/ros1_ws/photo_stops/standee_route_runs/YYYYMMDD_HHMMSS/
```

## 7. 下一窗口开始导航优化时的检查顺序

先确认没有旧路线执行器和旧 GMapping：

```bash
docker exec ros1_modeling bash -lc \
  'source /opt/ros/noetic/setup.bash; rosnode list | grep -E "slam_gmapping|route_executor|map_server|amcl|move_base"'
```

确认固定地图内容：

```bash
docker exec ros1_modeling bash -lc \
  'source /opt/ros/noetic/setup.bash; rostopic echo -n 1 /map/info'
```

期望：`resolution=0.05`、`width=224`、`height=224`、`origin=(-6,-6,0)`。

确认 RViz：

- Fixed Frame 使用 `map`；
- Map 显示 `/map`；
- 不要把旧 `/combined_map` 和新 `/map` 叠加成两个不同来源；
- 激光、AMCL 粒子和机器人位姿应在同一固定地图坐标系内。

导航优化时优先调整 AMCL、局部/全局代价地图、footprint 和白线安全距离；不要替换底图，也不要重新运行 SLAM。

## 8. 地图验收标准

本地图可作为导航底图的判断条件：

- 使用 `current_slam_preview_white_lines.yaml`；
- 来源图哈希通过 manifest 校验；
- `/map/info` 为 `224 x 224`、`0.05 m/cell`、origin `(-6,-6,0)`；
- 白线占用单元仍存在（manifest 记录 `1711`）；
- 导航目标全部位于赛场内部；
- 地图外未知区域不参与本次路线判断；
- AMCL、move_base、RViz 使用同一个 `/map`。
