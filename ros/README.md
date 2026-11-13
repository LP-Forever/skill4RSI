# ROS action-chunk 接入——初版实验版本

**此文件夹是 ROS 链路初版实验快照，不是当前使用版本。当前实际使用的是兄弟目录 `../v2/` 的 V2 STANDARD skill 链路。** 本打包仅去掉本机路径/无关旧模块依赖，没有把 ROS 初版升级为经过完整搭建验收的新版本。

## 打包边界

包含模型 chunk 服务、执行适配器、反馈订阅、相机 HTTP 接入、纯几何和记录逻辑。**不包含原已有 ROS master、FrankaPy、原 franka_interface driver、ROS 消息/Action 定义、夹爪节点、catkin 工作空间、原 CartesianImpedanceFeedbackController，或旧 TrajectoryExecutor 的 IK/FK/控制代码。** 外部安装必须已经兼容真机。

旧轨迹执行器中的五段必要纯数学（正交化 + 最小 jerk 标量 + SLERP + delta 到 waypoint + trajectory）抽为 `ros_chunk_geometry.py`，方法体保持一致；不是把整套原控制链路搬进本文件夹。

## 数据流与完整循环

```text
操作者任务 + 两组本地参考 + memory（发布包为空）
                  │
先启动 halo_ros_action_service.py：只监听 /health、/predict、/abort
                  │
操作者 run_task.py → 订阅外部 ROS 实测状态、检查接口空闲
                  ├─ camera HTTP /snapshot → 四路当前 JPEG + metadata
                  └─ 实测 O_T_EE → state=[x,y,z,qx,qy,qz,qw,width]
                  │ POST /predict，带 exp_id、执行反馈及上一夹爪命令
                  ▼
模型桥 /v1/responses → {status,stage,message,act}
                  │ 20×7 chunk，或 done/error + act=[]
                  ▼
核验 exp_id / chunk_id / 模型 / 数据合同
                  │
ros_chunk_geometry：基坐标平移累加、局部欧拉旋转右乘
                  │ 平移 minimum jerk + 旋转 SLERP，生成位姿参考
                  ▼
trajectory_executor_ros：按夹爪变化把 chunk 分组
                  │ 15 Hz：PosePosition + CartesianImpedance sensor messages
──────────────────┼───────── 本仓库交付边界 ─────────────────────────
外部 ROS / FrankaPy 动态技能 → 原笛卡尔阻抗 C++ 控制器 → Torques → 真机
                  │ 对应行的位姿流结束后，独立 gripper move/grasp Action
                  ▼
订阅实测反馈 → 保存结果 → 全部 20 行结束后重新看四路图
                  └───────────────── 下一次 /predict
```

`halo_ros_runtime.py` 订阅：
- `/robot_state_publisher_node_1/robot_state`：`franka_interface_msgs/RobotState`；
- `/franka_interface_status_publisher_node_1/franka_interface_status`：`FrankaInterfaceStatus`；
- `/run_loop_process_info_state_publisher_node_1/run_loop_process_info_state`：`RunLoopProcessInfoState`；
- `/franka_gripper_1/joint_states`：`sensor_msgs/JointState`，两个手指位置之和得到开口 m。

这些是本次适配所用外部主题名称，不是对所有 Franka ROS 安装的通用承诺。改变命名空间需匹配外部部署，不得删掉新鲜度/错误检查。

## 数据合同

```json
{"status":"continue","stage":"阶段","message":"视觉证据和不确定性","act":["必须是20行，每行7个有限数值"]}
```

上例只展示格式，不是可执行 action。每行实际是：

```text
[dx, dy, dz, droll, dpitch, dyaw, g]
  └基坐标系 m┘  └局部 XYZ 欧拉增量 rad┘  └1=open，0=close┘
```

- 起点是当前实测 ROS `O_T_EE`，**不是 V2 TCP，也没有 fr3_hand IK/FK 重映射**。
- `p_next=p_current+[dx,dy,dz]`；`R_next=R_current @ R.from_euler('xyz',[droll,dpitch,dyaw])`。这是**右乘局部旋转**，与 V2 `rotate` 的左乘基坐标 yaw 不同。
- state 是 `number[8]`：位置 m、四元数 **xyzw**、实测开口 m。不是 V2 API 的 wxyz 参数顺序。
- action 夹爪值是离散指令，不是宽度或连续训练数据；本 ROS 初版不使用十维 action。
- `PosePositionSensorMessage` 包含 id/timestamp、位置三数、SDK/FrankaPy要求的 **wxyz** 四元数；序列化成 `SensorDataGroup`，不是把 state 的 xyzw 原样传入。
- `CartesianImpedanceSensorMessage` 是平移刚度 `[900,900,600] N/m` 和旋转刚度 `[75,75,75] Nm/rad`。它们不是 V2 七关节的 Nm/rad 刚度数组。
- `ShouldTerminateSensorMessage` 请求结束自己持有的流；原控制器再计算七关节力矩并交付 SDK，这个原控制器不在包里。

默认 15 Hz、每动作行 40 个插值 interval，名义约 2.67 s/行。包含端点样本、组终止与夹爪等待会增加实际时间；**15 Hz 不等于模型每秒执行15行动作**。

夹爪变化在对应行终点后执行；连续相同指令不反复抓取。初始 previous_gripper 为 None，第一行会显式应用其夹爪命令。需要原地夹爪动作时使用零位移行表达该阶段，而不是随意补零凑数。**整个 chunk 内没有视觉纠偏**。

模型必须输出恰好 20 行；done/error 的 act 必须为空。每轴位移 ≤0.05 m、每轴旋转 ≤0.1 rad、chunk 平移路径长度 ≤0.20 m；累积 waypoint x=[0.20,0.65]、y=[-0.35,0.35]、z≤0.60 m。没有现场通用的桌面下限，不能据此规划穿桌动作；这些边界不是碰撞证明。

## 外部安装与双环境

- 模型/相机服务：Python **≥3.11**；依赖 `requirements-model.txt`。
- ROS 执行：已有 ROS Noetic/FrankaPy 兼容环境；原实验为 Python 3.8。必须使用安装原 ROS 消息、FrankaPy、autolab_core 的解释器，不能假设新 Python 3.11 venv 就能加载原 ROS 栈。
- 已有模型桥在 8411 提供 Responses；接口/认证说明同 V2，未打包 bridge 或任何密钥。
- 已有 V2 网页提供**仅相机预览**。本初版相机 HTTP 桥读取现有预览，不自己开物理设备；切到 ROS 时必须停掉 V2 arm writer，保留 camera-only 功能。
- 外部 ROS master、原 driver（原实验 `with_gripper=0`、`stop_on_error=1`）、原 arm bridge、独立 gripper node 必须先按原部署启动。本包不擅自启动这些组件。

准备本地配置：
```bash
cd skill4RSI/ros
cp cameras.example.json cameras.local.json
cp halo_ros_config.json halo_ros_config.local.json
cp task.example.txt task.local.txt
mkdir -p runs
```
填写相机身份、明确任务、参考 manifest 和端点。配置中的文件路径相对本 `ros/` 解析，可使用绝对路径。参考 01/02 必须用自己当前部署的身份导入，示例导入方法见 `../v2/README.md`；**在 ros/ 运行它自己的 prepare_references.py**，manifest 位于 `ros/halo_skill_memory/task_references/`，不能直接引用兄弟 v2/ 路径（路径约束会拒绝）。memory 的读取是 legacy 和最近3个 episode；发布包只有框架，新 ROS 实验不会自动接入 memory。

## 操作者运行一次完整任务

以下服务与 V2 模型监听共用 7777，切换前确认没有已有监听/执行器，且不存在两个机械臂 owner。

终端 A（模型 Python 3.11 环境）：
```bash
cd skill4RSI/ros
python halo_ros_action_service.py --config halo_ros_config.local.json --port 7777
```

终端 B（相机 HTTP 服务；不会启动原相机/机械臂驱动）：
```bash
cd skill4RSI/ros
python halo_ros_camera_service.py --console-url http://127.0.0.1:6657 --port 5000
```

终端 C（现有 ROS 环境；下面路径必须替换成外部安装的真实路径）：
```bash
export SKILL4RSI_ROS_SETUP=/opt/ros/noetic/setup.bash
export SKILL4RSI_CATKIN_SETUP=/path/to/existing/catkin_ws/devel/setup.bash
export SKILL4RSI_ROS_PYTHON=/path/to/existing/ros/environment/bin/python
cd skill4RSI/ros
bash run_task.sh --config halo_ros_config.local.json --check
# 现场确认、操作者明确开始后：
bash run_task.sh --config halo_ros_config.local.json
```

`--check` 查询状态和服务，不请求模型、不发机械臂运动；它不是完整安全验收。正式执行每轮观察→预测→整chunk执行→新观察，直到模型视觉判断 done、失败、预算用尽或操作者中断。没有自动初始移动、HOME、开爪、恢复或重试。

模型 done 只代表模型对当前图像的判断，不是物体成功的独立证明。每轮驱动 Action 的成功也不是抓对积木的证明。保存 model 输入/输出与实际轨迹/反馈分别用于研究。

Ctrl+C 会尝试取消本实例持有的臂/夹爪 goal，记录确认，不 cancel_all_goals；保存视频/结果，并对失败/中断的实验发 `/abort` 丢弃未接收动作。取消失败需现场检查。模型监听在 done/error/abort 后不自动复用；结束后操作者关闭这次模型服务，再开启新尝试。相机 HTTP 服务、ROS master、原 driver、模型桥不会被任务结束自动全关。

## 已知初版限制与保护边界

初版已有真实启动/运行记录，但尚没有本打包 ROS 路径完整搭建成功的验收。本机首次实验曾因相机预览源失败终止；发布包不包含该视频，也不能把 V2 成功搭建记录当成 ROS 的成功证据。

底层是原 external-torque 笛卡尔阻抗，非 V2 native `move_pose`。不复制 V2 的接触趋势/跟踪策略。原 driver 的碰撞设置未改；其 run_loop 可能在检查 stop_on_error 前调用自己的 `automaticErrorRecovery`，所以不能声称底层完全无恢复。本应用不添加恢复、回放或自动清锁。反馈时钟、原错误、碰撞标志、接口就绪、相机新鲜度与 action/workspace 合同仍检查；没有新增 3 mm 误差拒绝或 8 N 差分阈值。

`runs/model_*` 保存任务、memory快照、参考上下文、模型输入、SSE、usage、输出；`runs/run_*` 保存chunk、实际执行轨迹 NPZ、反馈/操作日志、outcome 与四路视频。全部默认 Git 忽略。

离线模型/合同测试：
```bash
python -m unittest discover -s tests -v
```
纯几何比较不需要 ROS/GPU；若验证真实 ROS 消息、Action 交接与取消，需要外部环境和现场分级测试。打包验证不代替真机验收。
