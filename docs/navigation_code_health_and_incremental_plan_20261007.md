# 导航代码体检与分步改进计划

日期：2026-10-07
范围：当前 ROS/Gazebo 导航代码及本地 Git/GitHub 状态
首要问题：巡检结束后车身不能稳定回到 HOME 朝向

## 约束

- 本计划只处理代码、代码测试和导航验证流程。技术报告与答辩 PPT 中的 20 轮数据是后续测试数据，本轮不改这些数字，也不顺手改动其他文案。后续只有完成对应测试后，才更新相应数据及由它计算出的图表。
- 保留当前工作区的已有改动和未跟踪文件。不得用 `git reset`、`git clean` 或覆盖操作清理现场。
- GitHub `main` 当前为 `43ab387`，导航归档分支 `codex/navigation-before-rebuild` 为 `52dc590`。这两个远端分支作为保留的好版本；新工作放到新的任务分支，不强推、不直接改写这两个分支。
- 每个代码任务执行以下顺序：检查工作区和目标文件 → 将修改前基线推到独立 GitHub 检查点分支 → 只做一个聚焦改动 → 运行该任务规定的检查 → 将通过检查的提交推送到同一任务分支 → 确认远端提交与本地一致后再开始下一项。检查失败时停止推进并修复当前项。
- 当前启动脚本会停止共享容器中的导航节点并重置运行状态。完成隔离 ROS/Gazebo 会话前，不在共享仿真上启动完整导航回归。

## 当前基线

- 修复工作的代码基线采用 GitHub `main` 的 `43ab387`。本地开发分支 `codex/home-return-fix-20261007` 从该提交建立，现有 `main` 和导航归档分支均保留。
- 另一个检查点分支 `codex/navigation-health-20261007` 保存了原本地归档快照及实验性 `navigation_rebuild/`；它不是当前正式导航入口。本地初始代码位于 `52dc590`，不得将其实验栈当成已验证的生产实现。
- 工作区中已有点位、地图截图、视频及提交材料等未跟踪文件。它们保留在本地，不纳入代码提交，也不打开或改写提交材料。
- 对 `origin/main` 当前路线测试的基线运行结果为 7 通过、2 失败。两项失败源自路线契约已更新而测试仍断言旧 7 段路线，以及合成审计记录只列出 7 段。更新为当前 13 段路线后，路线与人物统计两组非 ROS 测试合计 18 通过。
- 全量 `pytest tests` 在当前主机无法收集 ROS 测试，因为缺少 ROS Noetic 的 `geometry_msgs`。当前没有运行中的 Docker/ROS 容器，因此真实导航回归测试尚未运行。
- 主机侧 `git diff --check`、`python3 -m compileall -q navigation_rebuild scripts` 与 `bash -n scripts/start_navigation_rebuild.sh` 在旧归档快照上通过；这不代表 GitHub `main` 的实时导航已运行验证。
- GitHub `main` 已先推送计划与测试基线检查点 `9e59364`，随后推送路线测试修正 `f674e38`。这两次提交均未改写 `main`。

## 已确认的回位风险

1. GitHub `main` 的正式路径是 `scripts/route_executor.py`。基线 `return_home()` 发送一次 HOME 目标；`navigate_goal()` 使用地图位姿检查到点，未达目标容差时记录 `ARRIVAL_MISMATCH` 并失败。
2. `precise_park()` 要求地图位姿误差不超过 3 cm、航向误差不超过 0.04 rad，并要求里程计线速度和角速度均低于 0.01、持续静止 2 秒。基线在位姿不达标时发布零速度并失败，没有发起末端校正。
3. 当前代码改动只对 HOME 使用 3 cm / 0.04 rad；仅当动作状态为成功、结果为 `ARRIVAL_MISMATCH` 时，追加一次最多 12 秒的校正目标。超时、走廊越界、门控拒绝和其他失败不重试。
4. 单轮bag显示：最后一条 `/amcl_pose` 时间戳为 667.849 s，仿真终点为 674.45 s；同期 `map → odom` TF仍更新至674.378 s。导航结束时AMCL话题约6.6秒没有新估计，而HOME验收只看TF新鲜度。这个时间差能解释“TF看起来新、定位估计实际旧”的风险，尚需单独验证。
5. 同一bag中 `/odom` 首末航向净变化约0.00032 rad，Gazebo车身真值变化0.11671 rad。活动world的轮心位置是 `y=±0.0665 m`，几何间距0.133 m，插件参数为0.13572 m；历史试验曾认为0.13572 m改善了里程计漂移，所以不能只按几何值直接回改。该差异是待验证假设。
6. 在线控制不能读取Gazebo真值。后续仿真验收须分别记录fresh AMCL、map TF、轮式里程计与车身真值；只有此轮对应的消息时间和坐标变换校准后，才判断哪一层造成分叉。
7. 旧检查点中的 `navigation_rebuild/` 使用 8 cm / 0.14 rad 地图位姿验收，且管理器会按该结果写 `HOME_VERIFIED`。这条实验路径保留在独立检查点分支，不用于本次正式 HOME 修复。

## 分步任务与验收

### 阶段 0：保存正式代码基线并修正过期测试

1. 正式修复分支从 GitHub `main` 建立，现有 `main`、归档分支和本地材料保持原样。
2. 已修正两项过时的路线测试，使断言跟当前路线契约和审计器清单一致；碰撞、轨迹连续性、静止验收及错误轨迹拒绝均保留。路线与人物统计测试为 18 通过。
3. 完整 `pytest tests` 在本机因缺少 ROS Noetic 的 `geometry_msgs` 无法收集；需在适配环境完成完整测试。

通过条件：现有非 ROS 测试全部通过；ROS 测试在正确环境完成收集并报告结果；测试修正没有放宽路线或安全验收。

### 阶段 1：收紧并补足 HOME 末端校正

1. 统一 HOME 的导航容差和最终验收阈值为位置 0.03 m、航向 0.04 rad；其他巡检点保持原配置。
2. 只在 HOME 动作确实返回成功、但最终地图位姿回报 `ARRIVAL_MISMATCH` 时，允许一次有记录的 HOME 校正目标。其他导航、安全或传感器失败立即停止，不自动重试。
3. HOME动作成功后调用AMCL的 `/request_nomotion_update`（`std_srvs/Empty`），等到时间戳晚于请求的新 `/amcl_pose`，再与map TF一起验收。服务缺失或没有新扫描时应失败关闭。该服务在[ROS Noetic AMCL源码](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp)中将强制下一帧激光参与更新。
4. 校正后继续使用现有 2 秒静止验收；将AMCL样本时间、AMCL与TF各自的误差、速度和最终停稳情况写入运行总结。
5. 同时检查 `+π/-π` 角度误差边界，避免把角度环绕误判为大偏差。不得使用固定 yaw 偏置。

进展：HOME容差与一次有界重试已实现；AMCL fresh-pose 检查已加入，并要求HOME验收使用请求之后的AMCL样本且与TF误差同时过阈值。第一次全路线集成验证暴露出“单次无运动更新不一定发布新 `/amcl_pose`”；根因和限时重复请求修正见下方记录。修正代码通过24项相关非ROS测试，也已在静止仿真中直接调用生产方法验证；完整路线复验仍待隔离仿真环境。

通过条件：HOME只使用请求之后的AMCL样本；AMCL与map TF均满足阈值；一次校正仅由成功动作后的位姿误差触发；缺样本时失败关闭；其他waypoint不变。

### 阶段 2：在隔离仿真中验证回位

1. 先跑单目标 HOME 回归，确认目标角度、odom→map变换、末端速度和校正分支。
2. 再跑完整路线，比较地图估计与 Gazebo 车身真值误差；记录碰撞、越线、红灯和回位失败。
3. 覆盖目标失败、取消、旧 TF、旧里程计、无速度反馈和中途 shutdown，确保门控仍失败即停车。

通过条件：机器人仅在fresh AMCL与TF位置/航向均达标，且持续静止2秒后才报告泊车通过；运行真值检查与AMCL结果分列。

### 阶段 3：形成新的 20 轮结果

代码和验收条件固定后，再执行计划中的 20 轮任务。按轮保存配置版本、运行摘要和失败原因；统计全部完成率、HOME 位置/航向误差、任务时长及安全事件。只有新一轮测试完成后，才更新报告和 PPT 对应的 20 轮数字与图表。

通过条件：20 轮使用同一代码提交和同一验收口径；失败轮次不删除、不混入旧版本数据。

## 工作顺序

本计划以 GitHub `main` 为正式代码基线。HOME 校正改动已在独立分支完成并通过非 ROS 检查，提交后推送，再在隔离 ROS/Gazebo 环境验证。隔离仿真通过后再跑新的 20 轮；仅在新测试完成后更新报告与 PPT 对应数据。每项记录文件、提交号、检查结果和未解决问题，确保随时能回退。

## 单轮仿真记录

日期：2026-10-08
代码：`20853ee`
运行：`20261008_home_refine_trial_01`，地图 `current_slam_preview_white_lines`，`PADDLE_OCR=false`

- 十点导航与拍照流程结束，摘要状态为 `COMPLETE_PARKED`；HOME 动作耗时 17.97 秒，move_base 返回成功。AMCL 地图误差为 1.60 cm、0.03475 rad，线速度和角速度低于 0.01，连续停稳 2.026 秒。
- HOME 位姿在AMCL容差内，所以本轮没有触发新增的末端重试。
- 从同一bag的Gazebo车身真值取起点和终点：车身位置相对起点漂移 3.53 cm，航向漂移 0.11671 rad（6.69°）。这超过最终泊车的3 cm、0.04 rad阈值。
- 结论：新增逻辑能在AMCL位姿不匹配时进行一次有界重校正；本轮没有触发该分支。更关键的是，AMCL验收通过时，Gazebo车身真值仍未回到阈值内，单靠HOME重发不能解决这个偏差。下一步先核对 `map → odom → base_footprint`、轮式里程计与车身参考点/驱动轴偏移，再决定控制或标定修改。
- 本次关闭了OCR，属于HOME导航与停稳检查，不代表完整提交验收。该单轮结果只记入本计划，不纳入后续20轮统计；技术报告和答辩PPT均未改动。
- 仿真容器在测试前已停止。本次运行结束后已再次停止容器，运行文件保存在 `/home/sz/ros1_ws/photo_stops/standee_route_runs/20261008_home_refine_trial_01/`。

## AMCL fresh-pose 集成失败与根因

日期：2026-10-08
代码：`dd7d759`（失败复现）；修正提交 `a1aec2e`，已推送至 `codex/home-return-fix-20261007`
运行：`/home/sz/ros1_ws/navigation_diagnostics/home_amcl_refresh_20261008/pose_route_01.bag`

- 全路线到达 HOME 后，`/route/status` 于仿真时刻 755.119 s 变为 `FAILED:MISSION`。bag 中 `/scan` 仍持续约 10 Hz；`/amcl_pose` 在 752.730 s 后直到 806.266 s 才再出现样本，间隔约 53.5 s。后一个样本对应之后手动调用无运动更新服务。
- 当时 AMCL 参数 `resample_interval=2`。ROS Noetic 源码显示 `/request_nomotion_update` 只把 `m_force_update` 设为 true；下一个激光回调据此处理一次激光，但 `/amcl_pose` 只在 `resampled` 或首次强制发布时输出。重采样间隔为2时，单次服务请求可能只处理一帧而没有发布 pose；是否碰巧发布取决于重采样计数所处相位。[源码](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp)
- 因此失败不是扫描中断，也不是已证明的时间戳偏移；现有实现只调用服务一次，却要求随后一定有新 `/amcl_pose`。这项假设不成立。HOME-only 的一次成功只说明该次重采样相位恰好允许发布。
- 修正是在原2秒总等待时间内每隔0.25秒再次请求无运动更新，直到收到时间戳晚于首次请求且年龄有效的 AMCL pose；仍不接受旧样本，超时仍失败关闭。失败信息增加请求次数、请求时间和最后样本时间。
- 相关 HOME/路线/人物统计测试为24通过，编译与 `git diff --check` 通过。之后把 `a1aec2e` 中的生产脚本复制到运行容器，在机器人静止、`/my_car/cmd_vel=0` 时连续两次直接执行 `request_fresh_amcl_pose()`；两次都在第2次服务请求后返回新样本，时间戳分别为1225.984 s、1226.285 s。测试不发导航目标或速度指令，没有移动机器人。
- 这项直接仿真检查验证了AMCL重复请求机制，但还没有覆盖全路线后的完整调用链。下一步是在隔离仿真中重跑单路线，检查 `COMPLETE_PARKED`、AMCL/TF/轮式里程计与Gazebo真值；20轮报告和PPT数据未改。

## HOME 末端 TF 内容过旧的证据

来源：`20261008_home_refine_trial_01/motion.bag`，首次完整路线

- 最后一条 `/amcl_pose` 的样本时间为 667.849 s，航向 1.708115 rad。该时刻附近 `odom→base_footprint` 航向约 0.0656 rad，`map→odom` 航向约 1.64095 rad；合成 map 航向约 1.7066 rad，与 AMCL pose 一致。
- 车身随后停稳：`odom→base_footprint` 航向降到约 0.00004 rad，Gazebo `my_car::chassis` 真值停在约 1.72331 rad。但 AMCL 没有新 pose，`map→odom` 的数值仍为约 1.64095 rad；AMCL 继续给这个旧变换发带有新时间戳的 TF。于是 `map→odom→base_footprint` 合成航向约 1.641 rad，看起来新鲜、内容却没有跟上编码器末端变化。
- 旧验收在HOME成功后只读取 TF，记录航向误差约 0.03475 rad并报告 `COMPLETE_PARKED`；同一时段 Gazebo 车身相对目标航向差约 0.1171 rad。这里的关键失效是用 TF 时间戳代替定位估计的新鲜度。
- Noetic AMCL 在没有重采样/强制发布时会复用 `latest_tf_` 重新广播变换；该行为与 bag 中 TF 时间戳持续更新、变换值不变相符。[AMCL 源码](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp)
- 当前代码先请求无运动更新并等待请求之后的 `/amcl_pose`，再分别测量 AMCL pose 和 TF 误差、按较大误差验收。因此两者分歧会失败关闭，不会再由单独的 TF 误差误报泊车成功。全路线验证此修正仍待完成。
- 轮距参数 `0.13572 m` 与轮心几何间距 `0.133 m` 的2%差异仍是待测因素，不能单凭这次 bag 确认为主因，也没有改动。后续用隔离的受控原地转向测量轮编码器角度与Gazebo车身角度，再判断是否需要校准轮距或转速/加速度。

## AMCL 新样本仍与车身真值分离；轮编码器转角比例复核

日期：2026-10-08
代码：`a1aec2e`（生产代码；本轮未修改）

- 运行 `/tmp/ros1_ws_home_validation/photo_stops/standee_route_runs/20261008_home_tf_regression_01/motion.bag` 全路线状态为 `COMPLETE_PARKED`。HOME 新鲜 AMCL 样本在 656.223 s，航向 1.57144 rad；TF 合成航向约 1.5714 rad，均在 0.04 rad验收限内。但停稳后的 Gazebo chassis 真值航向约 1.47533 rad，目标为1.606236 rad，真实误差约−0.1309 rad；AMCL/TF与真值仍差约0.096 rad。新鲜度修正阻止不了地图定位本身的偏差。
- 从 POINT_10 结果到 HOME 停稳，真值航向变化约2.9605 rad，轮编码器里程计变化约3.1685 rad，相差约0.208 rad（约7%）。
- 在旧 `wheelSeparation=0.13572 m` 下做四次受控原地转向，正向0.3/0.9 rad/s的编码器/真值转角比分别为1.046/1.042，反向分别为1.060/1.033；输出速度命令与请求值一致。正反向、两种速度都显示编码器转角偏大，支持“有效轮距/滑移标定不匹配”的判断。
- 临时 world 将轮距改为0.1418 m后，正向0.9 rad/s的受控转角比降至1.005；但临时全路线在 POINT_10 因 `NO_ROUTE_PROGRESS` 失败，进度保持0且门控为 `CLEAR`。这说明单独改大轮距会引入路线风险，因此候选值没有合入正式 world 或 `models/my_car/model.sdf`。
- 后续先定位 POINT_10 临时失败的局部规划原因，再用中间轮距做同一受控转向与全路线对照；只有在路线安全验收通过且 HOME 真值误差改善后，才提交校准值。报告和PPT的20轮数据仍未改。
