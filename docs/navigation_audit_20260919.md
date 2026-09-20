# 导航与 SLAM 全项目审计（2026-09-19）

## 范围与结论

本轮是只读诊断，没有重启仿真、发送运动目标或修改实现/参数。本文件是审计产物。
检查了项目内全部 80 个非缓存文件：12 个 Python、12 个 shell、3 个 world、5 个 SDF、5 个 model.config、3 个 DAE、2 个 launch、10 个 YAML、2 个 RViz、8 个 Markdown、1 个 JSON、1 个 material、3 个纹理说明、6 个 PGM 和7 个 PNG。文本代码逐项核对，世界与模型/网格按 XML 解析，图像解码和地图逐像素比较。Python AST、shell bash -n、XML 解析均通过；这不代表行为正确。

当前不是一个参数导致的单点故障：地图来源被混淆、SLAM/导航进程没有隔离、TF 多发布者冲突、里程计速度坐标语义错误、底盘基准点不符合差速模型、避障被关闭，以及验收不充分同时存在。不能把未通过测试的地图/窗口称为导航成品。

“所有可能的问题”无法靠静态阅读穷尽。以下区分 **已证实**、**条件性缺陷** 和 **待定量验证的风险**，不把推测包装成根因。

## A. 地图真实性和场景一致性

### A1. 所谓重新建图结果与旧真值图逐像素完全相同【已证实，最高优先级】

| 文件 | 图像尺寸 | 与 competition_ground_truth.pgm 不同像素 |
| --- | --- | --- |
| competition_slam_demo.pgm | 200×200 | 0 |
| rebuilt_map.pgm | 200×200 | 0 |
| rebuilt_map_clean.pgm | 200×200 | 0 |
| competition_slam_complete.pgm | 200×200 | 0 |

上述文件与 ground_truth 解码后的像素 SHA-256 均为：
`9889c1f5f5ea62b62f5afdf767e1d06bec93e0b39ab14ec3f7361b139699b378`。
像素分布均为：占用 6171、未知 1584、空闲 32245。PGM 文件头不同不影响这一结论。

`scripts/generate_ground_truth_map.py:50` 是从 SDF 碰撞盒直接栅格化地图的代码，非激光 SLAM；`maps/competition_slam_demo.yaml:1` 更直接引用 `competition_ground_truth.pgm`。
现存“重建/完整”地图没有独立 SLAM 结果的证据，内容就是原真值图。之前保证“确定是雷达扫出来的”错误，必须撤回。

机制：`map_saver` 订阅 `/map`，并不校验发布者是 GMapping，静态 map_server 重发的地图也能保存成功。`scripts/save_slam_map.sh:11` 只检查节点名字；直接运行 map_saver 更连这个检查都没有。当前 `/map` 唯一发布者是 `/map_server`，没有存活的 GMapping。无法只凭当前状态还原每次历史保存的发布者，但像素同一性是确定事实。

### A2. 旧图含已移除的三个障碍【已证实】

在内存中按当前 world 重新执行栅格化并与旧图比较，共 402 个像素不同，全部是“旧图障碍 → 当前场景空闲”。差异分成三组，世界范围：

- x [-1.475,-0.825]、y [-4.175,-3.725]：131 格；
- x [-2.925,-2.275]、y [-4.225,-3.775]：132 格；
- x [-4.275,-3.625]、y [-4.275,-3.825]：139 格。

位置对应 6 米旧场景中的三块 car_plate_area 放大后的区域。当前 world 已删除这些模型，旧地图未更新。导航仍会绕开不存在的障碍；不能用改名另存消除它们。

### A3. 真正旧 SLAM 文件远未完成；地图白色不等于小车可达【已证实/几何分析】

`competition_slam.pgm` 为 4000×4000，其中未知 15,991,745 格，仅空闲 7,989 格、占用 266 格。大画布并不代表完整覆盖。
生成器把场地内大片区域直接填成空闲，包括没有扫描、墙内或无法驶入的区域。因此“完整轮廓”无法证明扫描覆盖或导航可达。
对当前 world 的二维栅格做四邻接分析：点机器人空闲区已有两个连通分量；用 0.16 米安全半径腐蚀后有三个分量，面积约 46.065、8.715、5.355 平方米。这里是辅助几何分析，不是实际车辆可达性证明；实际还要考虑轮子外廓和姿态。
验收应覆盖出生点可达区域，不应承诺车辆穿过封闭墙去到任意白格。

### A4. 地图生成器本身有适用限制【条件性缺陷】

`generate_ground_truth_map.py` 只处理顶层静态模型的 box，忽略 roll/pitch、三维高度区间、mesh/cylinder 和 include/nested model；按盒子厚度而非是否与车体/雷达高度相交筛选。person_area_2 有约 0.10 rad roll，二维近似有误差。真值图只能作为辅助参考，不能充当 SLAM 验收。
保存后的 YAML 使用容器绝对路径，复制到宿主机后直接本地打开不可移植；启动脚本只按 basename 复制 YAML/PGM，不通用解析 YAML 的 image 路径。`save_slam_map.sh` 对地图名也未做路径/命令字符校验。

## B. 生命周期、TF 与定位

### B1. 同一雷达 TF 三个活发布者，且数值不一致【已证实，最高优先级】

8 秒订阅 `/tf` 与 `/tf_static` 实际观察到：

| 发布者 | 通道 | chassis → lidar_link 的平移 |
| --- | --- | --- |
| /chassis_to_lidar_mapping | /tf | (-0.05, 0, 0.25) |
| /chassis_to_lidar_slam | /tf_static | (-0.083333, 0, 0.416667) |
| /chassis_to_lidar | /tf_static | (-0.083333, 0, 0.416667) |

旧 dynamic TF 进程 PID 72117 仍存活。当前文件虽然已删掉该节点定义，运行进程不会自动退出。Gazebo 查询真实雷达相对 chassis 为 (-0.083299,约0,0.416654)，所以旧发布者确实错误。
冲突会令各消费者使用错误/不一致变换；不同订阅顺序和 TF 缓存行为也影响结果。不能只把 z 高度差当作此前扇形地图的唯一原因：二维 XY 差约 3.3 cm，地图来源、时间和定位也有问题，必须分别验证。

### B2. 建图、探索、导航、测试没有互斥状态管理【已证实】

`start_slam_mapping.sh:16` 没停旧 `/map_to_odom_sim`、`/chassis_to_lidar_mapping`、`/chassis_to_lidar`、`/car_teleop`、waypoint/patrol/test 等节点。
`start_navigation.sh:39` 未停 `/explore`、`/autonomous_mapper`、`/waypoint_mapper`、`/patrol_controller`、旧 mapping roslaunch 和旧 lidar_mapping。
`start_autonomous_mapping.sh:18` 也未清理遥控、静态 map→odom、final_map_server 等来源。
当前存在两个旧 mapping roslaunch 和导航 roslaunch；`/explore` ping 成功，仍与 `/validate_navigation` 同时作为 `/move_base/goal` 发布者连接。连接本身不证明它每秒都发目标，但架构允许目标相互抢占，测试不隔离。
历史对话已采到遥控高频零速度与自动控制非零速度交错。car_teleop.py:52 在不按键时持续发零，完全可以压制其他速度源。当前 `/cmd_vel` 只剩 move_base，不能把历史竞争误报为当前仍在竞争。

### B3. AMCL 的定位修正没有进入导航【已证实】

`navigation.launch:13` 固定 map→odom 为零，`:31` 设置 `tf_broadcast=false`。AMCL 只是输出诊断位姿，2D Pose Estimate 无法通过 AMCL 改变导航 TF。
这仅能在“世界坐标真值 odom + 与世界严格对齐的真值地图”模式下有意使用，不是正常 SLAM 地图定位架构。
真实 SLAM 保存的 map→odom 可能有修正，换成零变换会丢失校正。反过来，建图时遗留的零变换又与 GMapping 的 map→odom 发布冲突。日志确有 `TF_REPEATED_DATA` 和 TF 树不连通警告。
注意：不能一概断言 GMapping 必定以出生点为地图零点。源码会使用初始 odom 位姿；本问题是未验证地图对齐就强制零变换。

### B4. 自动建图依赖上一次启动留下的参数和 TF【已证实，冷启动缺陷】

`mapping_exploration.launch:3` 没有指定 `base_frame=chassis`；GMapping 默认是 `base_link`。项目没有发布 chassis 与 base_link 的连接。
当前参数服务器残留 `/slam_gmapping/base_frame: chassis`，会掩盖冷启动错误；不能以当前参数正确证明 launch 完整。
该 launch 删除雷达 TF 后也没有 odom/传感器 TF 节点，自动启动脚本并不启动 model_state_odom。`start_waypoint_mapping.sh:14` 同样依赖缺省 base_frame。直接启动和先手动建图再启动的行为因此不同。

### B5. 出生点存在三个定义【已证实】

- world: (4.103,-4.1136667,yaw=1.621)，见 competition_classic.world:83；
- navigation/validator: (4.0833,-4.0833,yaw=1.5708)；
- autonomous/waypoint mapping: (2.30,-2.30,yaw≈1.5708)，见各脚本 set_model_state。

后者是明显的大幅位移，不满足出生点不变。重置同时存在控制器时，重置后会立即被继续驱动；历史验证的“出生点改变”可由这个机制解释，不能直接认定一定是 TF 错位。脚本未验证服务 success 和稳定静止后的实际位姿；START_X/Y/YAW 变量也未用于后面的硬编码命令。

### B6. 时间回跳处理错误、旧 ROS 状态不断累积【已证实的代码缺陷】

`model_state_odom.py:28` 在时间不递增时强行用 last_stamp+1 微秒；Gazebo 若重启/仿真时间回跳，TF 会保持旧未来时间，直到很久以后真实时钟追上。
`ModelStates` 没有 header，回调用当前时间给队列里的状态盖章，存在时间错配风险；高频发布还增加负载。
存活检测混用节点注册名、pgrep 和进程状态；ROS master 有大量失联旧节点，`rosnode list` 出现名字不等于活着。

### B7. 备用 AMCL relay 的变换公式错误【已证实，当前未启用】

`amcl_tf_relay.py:24` 直接把 AMCL 的 map→base 平移当成 map→odom。正确应求整个变换：T_map_odom = T_map_base × inverse(T_odom_base)，平移也必须扣除旋转后的 odom→base 平移。该脚本还用最新 odom 匹配不同时间的 AMCL 消息；不能作为修复方案直接启用。
`sim_tf_relay.py` 若与 model_state_odom 一起启动则重复发布 odom→chassis。

## C. 底盘、里程计与规划器

### C1. /odom 的 twist 坐标语义错误【已证实，直接影响 DWA】

`model_state_odom.py:33` 标记 child_frame_id=chassis，`:45` 却直接复制 Gazebo 世界坐标系 twist。ROS Odometry 要求 twist 在 child_frame_id 中表达。DWA 的 OdometryHelperRos 直接读取 vx/vy/wz，不会替代码转换。
应至少对世界平面速度做 v_body=R(yaw)^T v_world；若更换到轮轴基准点，还必须变换速度作用点。
实测例子：yaw=-1.37335，消息写入 vx=0.05649、vy=-0.02418；旋转到车体应为 vx=0.03479、vy=0.05065。DWA 使用了错误方向和分量，动态窗口与实际运动不一致。

### C2. 规划基准点不在差速轮轴中心【已证实的几何问题】

world 的左右轮中心在 chassis 的 x=0.125，而 move_base 使用 chassis、vy 限定为0。差速车实际转动中心在前方轮轴，原地转动时 chassis 会绕轮轴平移，并产生车体侧向速度 vy≈-0.125×wz。
实测 wz 约 -0.4 时，旋转到 chassis 后 vy 约 +0.05 m/s，符合这一关系。这会造成“只发转向但车身中心在绕圈”，并破坏规划器对非完整运动的预测。不是轮径和轮距不匹配：当前 world 中轮径0.11、轮距0.31666667与插件一致。
后续应把导航基准统一到轮轴投影，并重新计算 TF、footprint、速度及出生点语义，不能直接把 yaw 参数调大了事。

### C3. 局部避障事实上全部关闭，深度相机没参与导航【已证实】

`local_costmap.yaml:12` 是 `plugins: []`；实测局部 costmap 为100×100，全部10000格都是0。
全局 costmap 只有 static+inflation；`common_costmap.yaml:17` 的 laser marking=false，而且其 obstacle_layer 根本未加载。
深度点云的订阅者列表只有 RViz 类显示，没有 costmap 订阅。RGB/depth/points 有数据只是显示成功，不是深度辅助导航成功。
这使 DWA 缺少墙体/新增障碍碰撞约束，完整静态图也无法替代局部避障。

### C4. footprint 没包住车轮，膨胀距离不足【已证实】

footprint 半宽0.16；实际轮子外侧约0.158333+0.0366667/2=0.176667，完整宽度约0.353333。每侧漏掉约1.67 cm。
inflation_radius=0.08 小于当前 footprint 的内切半径0.16，不能提供应有的整圈车体净空。全局规划是栅格点路径，局部又没有障碍层，容易产生视觉上有路、实体车却擦碰的行为。

### C5. DWA 配置中有无效参数和不协调阈值【已证实/调参问题】

- `move_base.yaml:32` 的 `vtheta_samples:32` 拼错，真实参数是 `vth_samples`；运行值仍是20。
- `min_vel_theta:-0.45` 错把最小角速度绝对值当负方向下限，动态配置运行值被限制到0。
- 实际最大线速仍0.12，遥控仍0.10；前面提速建议没有落实。
- `acc_lim_trans` 未设置，运行值0.1；`trans_stopped_vel` 默认0.1，与0.12巡航速度很接近。
- `prune_plan` 当前false，恢复行为关闭、不允许倒车，窄道/贴墙时缺乏脱困手段。这些不是全部独立必现错误，要在前面模型/避障正确之后调参。
- 1.2秒仿真窗口在0.12m/s下只预测约0.144米，属于短视低速配置，不应先靠它解决基础坐标错误。

### C6. diff_drive 参数名和缩放物理属性不一致【已证实/次要风险】

`<torque>10</torque>` 不是当前 Noetic 插件读取的 `<wheelTorque>`；运行日志明确 missing wheelTorque default is 5。`publishOdometry` 也不是该版本的主要开关；目前 `publishTf=false` 阻止其 publishOdometry 调用，所以本次确实没有发现第二个 /odom 来源，不能误报重复 odom。
`scale_competition_world.py` 只缩几何/位置/轮径轮距，未同步质量、惯量、控制 TF、传感器和脚本坐标。活动 world 已放大，但独立 models/my_car/model.sdf 仍旧尺寸和普通相机。改独立模型不会自动修改 world 内嵌机器人。

## D. 传感器与 RViz

### D1. 雷达扫描面擦着矮墙顶部【几何已证实，漏扫程度待测】

地面顶面 z≈0.083333、雷达相对底盘 z=0.416667，合计0.5；矮墙中心0.25、高0.5，顶面也是0.5。
Gazebo get_link_state 实测雷达世界z=0.4999908，且物理姿态有微小俯仰/滚转。射线可能有的打中墙，有的从墙顶越过，存在严重不稳定漏扫风险。
将 TF 的雷达高度改低并不会移动真实传感器；应修改真实传感器安装位置并同步外参，在稳定高度观察所有方向的墙命中率。

### D2. 相机模型和显示配置分叉【已证实】

活动 world 使用 depth 插件，独立 model.sdf:208 仍是普通 camera。导航中的 camera_link 被用作 optical frame，但 view_rviz.sh:12 发布零旋转相机 TF，两个入口不一致。
view_rviz.sh 的 pgrep 匹配字符串也出现在自身 shell 参数里，有自匹配而跳过启动的风险；它还把相机 TF 的启动错误地绑在“雷达TF不存在”的条件里。
`autonomous_mapping.rviz` 没有任何深度相机/图像显示，且 Frontiers、Global Plan、Local Plan 全是 disabled；此前说这个配置包含全部相机画面不实。
`competition.rviz` 的 PointCloud2 选 Intensity，但实测字段为 x,y,z,rgb，没有 intensity；应核对显示状态或选择RGB8/平面着色。导航配置也未显示全局/局部路径，排查信息不足。

### D3. 当前传感器与物理仿真活着，不能继续笼统归咎 Gazebo 退出【实测排除项】

8秒采样：scan79帧、RGB118帧、深度118帧、点云118帧，约10Hz/15Hz；/clock持续前进，实时率约0.99，physics.pause=false。
当前深度图32FC1、640×480，RGB rgb8，点云存在。日志有显卡加载警告，但本次没有证据说明它是导航转圈的原因。
地面贴图11942×11942（约1.43亿像素）偏大，是显示/资源风险，不是本轮已证实控制故障。

## E. 自动建图、测试与启动可靠性

### E1. wall follower 不具备完整探索能力【已证实】

`autonomous_mapper.py` 只有前方/右侧激光阈值控制，不读取地图、不找 frontier、不记访问、不判断闭环或覆盖；右侧空旷时固定0.08m/s、-0.30rad/s，会沿半径约0.267米转弯，可能空地绕圈。
没有 scan 时间新鲜度检查；无有效射线时当作远处无障碍；无卡住检测和可信完成判定。等待30秒/60秒或900秒再保存，均不能证明完整覆盖。
`waypoint_mapper.py` 是旧6米坐标的直线巡点，遇障碍停止，亦非全场自动探索器。持续操作和传感器断流要有停机与状态管理。

### E2. 五点验收脚本不足以支持原要求【已证实】

`validate_navigation.py`：

- :63 BFS 把 unknown=-1 当可通行，只排除>=50；空闲连通性结论可穿未知区。
- BFS按点机器人计算，只有目标做7格净空检查，没有验证全路径车体通行。
- :75 目标限定距出生点≤2.2米，不能代表全场五点。
- :102 只检查出生x/y，未检查yaw/静止/真实Gazebo位姿。
- :95 成功只看action SUCCEEDED，打印终点但未比较到达误差、朝向、碰撞或是否被瞬移。
- 目标是map坐标，但位置读取odom，没有TF转换。
- :93 超时未取消并等待停止，退出/异常也未可靠取消最后活动目标。
- 90秒是ROS仿真时间；仿真暂停可让等待长时间不退出，缺少墙钟看门狗。
- 没有单实例锁、控制源检查、逐目标持久结果、录制和自动汇总；shell启动未用python -u，终端可能长时间不显示缓冲输出。

`random_navigation_test.py:94` 根本没有调用free_points，实际上发送固定五点；count默认10，但只有五个目标，统计分母也可能错误。`patrol_controller.py` 全部失败仍会打印 Patrol complete，不能作成功证明。
本轮没有重新运行会驱动车辆的五点测试，当前失败诊断不等于修复后验证。

### E3. 启动成功信息是假阳性；重复启动进一步污染环境【已证实】

多数宿主shell虽然set -e，容器内bash -lc没有，内部服务调用失败后仍启动并回报成功；`grep -E` 只需匹配一个节点就通过，并非全部节点都ready。
只sleep固定秒数，没有检查clock、scan、TF、/map来源、action server、Gazebo服务success和RViz实际状态。
start_sim只拷贝world，若gzserver已经存在则不重载；修改文件后重复执行不等于运行世界更新。容器首次创建分支不安装项目额外依赖/模型资源，依赖现有持久环境；进程式roscore检测亦不是健康检查。
此前操作中的 `pkill -f "python3 /root/autonomous_mapper.py"` 可以匹配包含该字符串的外层bash -lc，先杀掉启动shell，后续nohup不会执行。历史第一次启动后ps为空与此风险一致。
此前多次重跑同名validate_navigation会踢掉前一个ROS节点；不能把没有终端输出当成需要再启动测试。
动态/静态TF、控制器、地图发布者须由同一套launch生命周期管理，仅杀同名节点不够。

### E4. 文档/工作流状态和实际交付不一致【已证实】

README/technical_solution描述AMCL定位、局部激光避障，但实际已关闭；技术文档footprint0.30×0.22/膨胀0.30与配置也不同。
architecture_analysis声称深度参与costmap，实际没有；summary/review仍明确五点失败，state却只是awaiting_review，未形成通过证据。
README说RViz录制/map、/scan、TF，但脚本没有rosbag录制功能。单凭窗口打开、地图保存和节点存在都不属于验收通过。
当前工程也没有.git用于回溯先前修改。

## 修复依赖顺序与验收门槛（本轮未实施）

1. 隔离运行模式：每条TF一个发布者、/map一个来源、/cmd_vel一个控制源、导航目标一个测试驱动；清理旧launch并确认无残留，而不是盲重启所有东西。
2. 校正真实机器人：统一出生定义、轮轴导航基准、真实雷达高度、传感器TF、全外廓footprint；核对轮驱动参数。
3. 校正odom：速度坐标/作用点、仿真时间回跳；先用受控直行和旋转实验验证实际位移及TF，记录误差。
4. 独立SLAM会话：明确chassis/轮轴base_frame和唯一map→odom权威，停静态地图，录制scan/odom/TF；覆盖所有物理可达区域，保存时验证/map发布者及本次会话来源。
5. 导航模式使用AMCL或明确标识的仿真真值定位方案，两者不要混用；恢复局部障碍层，把深度点云按正确光学TF和高度过滤接入，并确认真实障碍在costmap出现。
6. 最后调DWA和速度；修验收脚本，在全场可达区域选择五个分散目标，固定出生只重置一次，逐点检验action状态、map坐标终点误差、朝向、碰撞/瞬移和墙钟超时。保留JSON/日志/轨迹/RViz证据再交付。

## 外部行为核验来源

find-docs/Context7 检索只返回ROS2条目，不适用于本项目，已改为直接核对ROS1上游源码和本机Noetic运行参数：

- https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/nav_msgs/msg/Odometry.msg （twist坐标约定）
- https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/base_local_planner/src/odometry_helper_ros.cpp （DWA直接取车体速度）
- https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/dwa_local_planner/cfg/DWAPlanner.cfg （vth_samples）
- https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/base_local_planner/src/local_planner_limits/__init__.py （角速度绝对值/默认阈值）
- https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp （wheelTorque、publishTf、世界速度转换）
- https://raw.githubusercontent.com/ros-perception/slam_gmapping/melodic-devel/gmapping/src/slam_gmapping.cpp （ROS1 GMapping默认base_link、初始位姿及map→odom；不是声称与已安装二进制commit逐字一致）
- https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/map_server/src/map_saver.cpp （只订阅/map，不验证SLAM来源）
