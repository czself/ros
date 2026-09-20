# Root coder foundation draft (runtime pending)

Implemented axle odometry with omega-cross-offset body velocity; capped 50Hz and reset-safe time. Legacy /my_car/odom remains chassis pose for old tools, while /odom is axle-frame. Split camera physical/optical frames. Lowered actual lidar to0.30 and visuals. World chassis birth normalized to user fixed coordinates; standalone car model synchronized to embedded active model. Corrected wheelTorque and explicit publishOdomTF=false.

Common robot/planner launch shared by SLAM/navigation. AMCL owns map->odom and uses axle initial pose. All GMapping parameters explicit with12m bounded initial map, truth odom noise0.001. Laser and depth obstacle layers restored, full axle-offset footprint, inflation0.4, DWA parameter names/thresholds corrected, max0.30m/s. RViz RGB8 point cloud and paths/footprint.

Static verification: Python AST, all navigation XML/YAML, modified world/model XML and RViz YAML parse PASS. check_foundation.py added for live TF/sensor and controlled movement verification; not executed yet. No map/five-point acceptance claim.

Reviewer should examine math/config while runtime coder finishes; final summary/evidence follows.
