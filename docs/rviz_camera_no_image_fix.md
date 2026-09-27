# RViz 摄像头画面显示 "No Image" 排查记录

日期：2026-09-23

## 现象

`./scripts/view_rviz.sh` 启动 RViz 后，Camera 与 DepthImage 面板一直显示
"No Image"，Displays 列表中对应项带橙色感叹号；Gazebo 与图像话题本身正常。

## 根因

`scripts/competition.rviz` 中 Image display 的话题键名写错：

```yaml
# 错误（键名无效，加载时被忽略，话题为空 → 不订阅）
- Class: rviz/Image
  Name: Camera
  Topic: /camera/image_raw

# 正确（RViz Image 面板的标准键名是 Image Topic）
- Class: rviz/Image
  Name: Camera
  Image Topic: /camera/image_raw
```

RViz 加载配置时忽略未知键 `Topic`，Image display 的话题属性保持为空，
因此**从未向 ROS master 建立图像订阅**——面板只渲染 "No Image" 占位符。
注意 `rviz/PointCloud2`、`rviz/Map` 等 display 的键名确实是 `Topic`，
只有 `rviz/Image`（及 `rviz/Camera`）使用 `Image Topic`。

## 排查过程（关键证据）

1. 话题侧一直正常：

   ```
   rostopic hz /camera/image_raw        # ~15 Hz, 640x480 rgb8
   rostopic hz /camera/depth/image_raw  # ~15 Hz, 32FC1
   ```

2. 但话题**没有任何订阅者**：

   ```
   rostopic info /camera/image_raw
   # Subscribers: None
   rosnode info /rviz_...
   # Subscriptions 中只有 /map /tf /clock 等，无任何图像话题
   ```

   这证明问题在 RViz 侧未发起订阅，而不是 Gazebo 相机没发数据。

3. 对照实验：用最小 rviz 配置，仅一个 `rviz/Image` 且键名为
   `Image Topic`，RViz 启动后立即出现订阅 → 确认是配置键名问题。

4. 附带干扰项（非根因）：容器内 RViz 启动日志有
   `libGL error ... nvidia-drm`，属缺少 GPU 直通时的软件渲染回退警告，
   与本问题无关；`/camera/camera_info` 偶发无数据也不影响
   `rviz/Image` 面板（它只依赖图像话题）。

## 修复

`scripts/competition.rviz`：两处 `Topic:` → `Image Topic:`（Camera、
DepthImage）。同步到容器后重启 RViz：

```bash
docker cp scripts/competition.rviz ros1_modeling:/root/competition.rviz
# 或直接重新执行 view_rviz.sh（脚本会重新 docker cp）
./scripts/view_rviz.sh
```

验证：

```bash
docker exec ros1_modeling bash -lc \
  'source /opt/ros/noetic/setup.bash; rostopic info /camera/image_raw'
# Subscribers 中应出现 /rviz_...
```

## 经验

- RViz 面板显示 "No Image" 且带感叹号时，先用
  `rostopic info <image_topic>` 看有无 RViz 订阅者；无订阅者 =
  配置键名/话题名错误，有订阅者才需要查 TF、时间戳、编码。
- `rosnode info` 的 Subscriptions 列表是判断 RViz 是否真正接入话题的
  最直接证据；`rostopic echo` 能收到数据只能证明发布端正常。
