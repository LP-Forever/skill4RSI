# V2 skill 接入链路——当前使用版本

**这是本系统当前使用的链路：V2 `STANDARD` + GPT-6-Astra + 一次完整 skill 规划。** 根目录合并图包含遥操柔顺分支，但本文件夹只提供模型到 V2 的任务侧程序；不包含手柄程序、V2 服务或 Python 柔顺有限动作改造。

## 程序边界与模块

```text
操作者：任务文本 + 可选两组参考照片 + 本地 memory
                      │
操作者先启动 start_model.py（只监听，不推理、不控制机器人）
                      │
操作者启动 run_task.py
  ├─ camera_preview.py ← 已有 V2 网页 /api/status、/api/offer → 四路 WebRTC
  └─ 外部 teleop_v2.Robot.get_state() → V2 原生实测状态
                      │ 四路 JPEG base64 + metadata + actual_state
                      ▼
             POST http://127.0.0.1:7777/predict
                      │
halo_skill_service.py：加载任务 / 参考 / memory / 可选标定证据
                      │ Responses input_text + input_image
model_io.py → 已有模型桥 /v1/responses → 模型推理
                      │ 完整 skill 序列 JSON + 上游 token usage
                      ▼
halo_pipeline_skills.py：核验协议、规划序列一致性
                      │
robot_calibration/block_skills_v2.py：每个 skill 前读实测 TCP
                      │ skill → 绝对目标位置 + wxyz 四元数 / 独立夹爪请求
                      ▼
外部 teleop_v2.Robot.move_pose() / robot.gripper.move() / grasp()
                      │ Unix Socket，UTF-8 JSON RPC
──────────────────────┼──── 本仓库的运动交付边界 ──────────────────────
外部 V2 C++ 服务 → CartesianPose / kJointImpedance → Franka 真机
                      │ 状态与 operation_status 回传
                      ▼
v2_transport.py：只在 SUCCEEDED 后接受下一动作；失败停止
                      │
序列终止 → 保存图像/轨迹/状态/结果 → POST /finish → 可选最终视觉复盘
                      │ 复盘不产生新运动
                      ▼
关闭本次模型监听；默认 hold，无自动开爪或 reset
```

| 模块 | 职责 |
|---|---|
| `start_model.py` | 从任务文件启动模型服务；默认 `--finish hold`；不连接机械臂 |
| `halo_skill_service.py` | `/health`、`/predict`、`/finish`；完整规划一次、参考/memory、结果记录 |
| `run_task.py`、`halo_pipeline_skills.py` | 操作者触发任务，录像、初态、模型输出核验、执行与收尾 |
| `robot_calibration/block_skills_v2.py` | skill schema、目标计算、时长、顺序执行及独立夹爪 |
| `v2_transport.py`、`v2_state.py` | 原生状态/控制权/新鲜度/保护检查及 operation 结果等待 |
| `camera_preview.py`、`camera_ids.py` | 读取已有 V2 四路预览，验证来源、身份、分辨率与时间戳 |
| `model_io.py`、`halo_model_usage.py` | Responses HTTP/SSE、完整 JSON 解析、上游报告的 token 用量 |
| `halo_task_references.py`、`prepare_references.py` | 配对参考场景、身份/路径/hash 校验；离线导入 JPEG |
| `robot_calibration/skill_calibration.py` | 可选现场标定证据；没有复制本站标定文件 |

## 数据合同：不要混淆维度、单位和坐标系

**模型返回**：一个 JSON 对象，例如结构如下（此处不是可直接执行的现场计划）：

```json
{"status":"planned","message":"视觉依据、假设及不确定性","skills":[
  {"op":"translate","position_m":[0.40,0.02,0.40]},
  {"op":"descend","distance_m":0.03},
  {"op":"close"},
  {"op":"lift","distance_m":0.03}
]}
```

模型必须生成 1–64 个完整必要动作，或 `{"status":"error","message":"原因","skills":[]}`。不输出代码、action chunk、关节角或力矩。

| skill | 数据 | 转成机械臂/夹爪交付 |
|---|---|---|
| `translate` | `position_m: number[3]`，基坐标系 m | 只取 XY，Z 保持执行前实测平面；保持姿态 |
| `descend` | 正 `distance_m`，m | 执行前实测 TCP 的基坐标 Z 减去距离；不自动夹取 |
| `lift` | 正 `distance_m`，m | Z 加距离；不自动开爪 |
| `rotate` | 有符号 `angle_deg`，度 | 原地基坐标 Z 旋转，四元数**左乘**：`q_next = q_baseZ × q_actual` |
| `close` | 无参数 | 独立原生 grasp；臂不移动 |
| `open` | 无参数 | 独立原生 move 开爪；臂不移动 |

臂交付 `position_m: float[3]` + `quaternion_wxyz: float[4]` + `duration_s` + operation 标识；四元数顺序 **w,x,y,z**。这七个数是位姿，不是 `[dx,dy,dz,droll,dpitch,dyaw,g]`。夹爪不属于位姿七维。

夹取目前采用 `width=0 m, speed=0.08 m/s, force=20 N, epsilon_inner=0.005 m, epsilon_outer=min(0.08, SDK max_width) m`；开爪至 SDK 最大开口（最多 0.08 m），速度 0.08 m/s。20 N 是**夹爪抓取力请求**，不是机械臂接触保护门槛。

50% 臂速时请求上限为水平 0.07 m/s、竖直 0.03 m/s、旋转 0.40 rad/s；有限动作时长会加最小持续时间并受 V2 重定时影响，不是严格保证的实际速度。夹爪参数不随臂速缩放。

四路图像键固定为 `kinect_0,kinect_1,realsense_0,realsense_1`；当前预览是 640×360 JPEG base64，metadata 包括相机身份、尺寸、帧时间与预览 generation。图像不是度量深度。实测状态由 V2 提供 `actual_pose`、关节/夹爪/估计 wrench、控制权、错误与新鲜度。

正常序列不在中途重看图重新规划；相机持续录像/健康检查不等于视觉闭环。每个动作在执行前用新实测 TCP 计算交付目标，不能由此宣称模型的整套世界坐标计划被实时视觉纠正。

## 外部环境与配置

需要 Linux、Python **3.11 或更高**、FFmpeg，以及 `requirements.txt` 中任务侧依赖。V2 Python SDK 必须来自与你的 V2 C++ 服务匹配的外部安装。

外部先就绪：
1. V2 native-pose `service` 已启动，**STANDARD、空闲 HOLD**，无故障或接触锁定，无其他 writer。
2. V2 网页四路预览可用（默认 loopback 6657）。本程序只读预览，不启动/restart V2。
3. 模型桥提供 `<model_base_url>/responses`（默认 `http://127.0.0.1:8411/v1/responses`），支持本部署指定的模型、reasoning、图像与 SSE。**模型桥不在仓库内**；当前客户端只带 Content-Type，认证如需要应由部署/受控代理配置，不要把密钥写进仓库。模板模型名是本部署使用的 `gpt-6-astra`，不是所有兼容端点都必然提供的模型；服务不会静默 fallback。
4. 端口 7777 没有其他任务服务。V2 和 ROS 不应同时写同一机器人；两种模型监听也共用该端口，不能同时启动。

```bash
cd skill4RSI/v2
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
cp cameras.example.json cameras.local.json
cp halo_skills.yaml halo_skills.local.yaml
cp task.example.txt task.local.txt
```

编辑：
- `cameras.local.json`：填当前 V2 status 中的真实身份字符串，顺序由程序固定；不要把示例占位符当成真相机。
- `halo_skills.local.yaml`：`v2_sdk_path` 填外部 `V2/python` 的绝对路径；`camera_python` 填同一已安装 aiortc 的 Python 解释器绝对路径；设置模型桥等地址。
- `task.local.txt`：写明完整任务、最终夹爪行为、必要接口边界，并要求参考 memory；历史修正应整理进有效加载的 memory，而不是不断堆入任务文本。

**本站原始标定不随包分发。** 可不配置 `calibration`，模型依据可用当前证据推理；这不是自动标定或可靠定位保证。如需保留现场原标定流程，在 YAML 添加：

```yaml
calibration:
  site_profile: "local_site/site.profile.json"
  table_reference: "local_site/table_reference.json"
  taught_trajectory: "local_site/taught_trajectory.json"
  setup_unchanged_confirmed: true
```

三个路径可以是绝对路径或相对本 `v2/` 的路径。只在自己的相机/工具/现场校验满足时设置 true。该模块仍有原安装的腕部相机约定（`realsense_1`、site camera_id `real_2`）、工具绑定和教学文件 schema 校验，不能拿别人的校准文件冒充当前实测。SDK 色彩内参只读查询需要已有 `pyrealsense2`；缺证据或校验失败时不会偷偷跳过已启用标定。将真实校准放到仓库外，或配置成被忽略的 `local_site/`。

## 配对参考与空 memory

发布包只有 `halo_skill_memory/.gitkeep` 及 `task_references/.gitkeep`，**没有 memory 经验内容**。
- 加载器只读取 `legacy_*.json` 和最近 3 个 `episode_*.json`；任意命名的 MD 或 `operator_preferences.json` **不会自动进入模型上下文**。
- `episode` 中最终 observation 的内嵌图像会从 memory 文本移出；运行时完整证据另存在 run/session 目录。
- 参考 01：物体身份/外观，**不是当前位置**。参考 02：目标最终结构。当前位置来自此次 current 四路图像与 V2 实测状态。
- 参考 manifest 被限制在本分支 `halo_skill_memory/task_references/<set-id>/`；图像必须属于同一 set，匹配相机身份、640×360 与 sha256。

自己准备两个目录，每个包含四个 `<camera_id>.jpg`。不附带本站原图。导入时提供照片的**实际拍摄时间**：

```bash
python prepare_references.py --set-id demo \
  --objects-dir /path/to/object_images --target-dir /path/to/target_images \
  --objects-captured-at '2026-11-13T10:00:00+08:00' \
  --target-captured-at '2026-11-13T10:05:00+08:00'
```

时间为格式示例，必须替换；导入工具不拍摄、不推理、不控制机器人。V2 模型服务可省略参考参数；有参考图的任务应明确传入 manifest。

## 完整任务如何运行

两个操作者终端；**以下 `run_task.py` 非 check 调用会执行机械臂/夹爪动作**。

终端 A（模型监听，只等任务）：
```bash
cd skill4RSI/v2
source .venv/bin/activate
python start_model.py --config halo_skills.local.yaml --task-file task.local.txt \
  --reference-set halo_skill_memory/task_references/demo/reference_set.json --finish hold
```

终端 B（检查，然后由操作者授权开始）：
```bash
cd skill4RSI/v2
source .venv/bin/activate
python run_task.py --config halo_skills.local.yaml --check
# 现场确认后才能运行：
python run_task.py --config halo_skills.local.yaml
```

`--check` 不请求模型预测、不发送机械臂动作，也不启动 V2。它检查模型 health 和 V2 状态；**不代表已完成相机/几何/路径安全验收**。正式启动后拉取当前四视图与实测状态，模型生成必要完整序列，执行器顺序交付并等待结果，最后保存结果、录像并请求 `/finish` 复盘。

默认 `--finish hold`：不额外开爪/reset；任务本身的 `open`/`lift`/`rotate` 仍按显式计划执行。`--finish open_reset` 是另一个可选实验模式，正常收尾会释放物体并 HOME（某些失败路径也会尝试清理），**不建议无意启用**。Ctrl+C 的中断路径不自动 HOME，但会尝试停止本执行器拥有的动作并保存记录。native 保护触发后不会自动清锁/重试。需要开启新尝试时由操作者重新启动 model 监听，使用当前画面，不重放旧序列。

## “完成”与证据

V2 原生 `SUCCEEDED` 是运动结果，不是任务成功。当前外部服务通常要求轨迹结束、TCP 误差 ≤1 mm/0.5°，实测与指令侧静止并连续满足 0.1 s；该实现不在本包，具体以安装版本为准。夹爪 `is_grasped` 也不能证明抓对物体。

`runs/run_*` 保存当前图像、四视角 MP4、初态、计划、每步目标/实际反馈、outcome 等；`runs/session_*` 保存模型输入输出、SSE、usage、最终复盘等。模型 output、native result、视觉结果必须分开解释。token 使用上游 usage，不由字符数推断；reasoning/cached token 是相应项的子集，不重复加总。

## 离线检查与当前限制

```bash
python -m unittest discover -s tests -v
python -m compileall -q .
```
测试使用 fake 模型和 fake 机器人，不打开设备。打包提取验证见根目录 `VALIDATION.md`。本站原生模型启动/任务记录不能当成移植包在另一台真机上已验收。

本分支保留现有实现：单次整序列、现有 workspace/动作幅度、独立夹爪和 V2 原生保护；三个原任务侧一致性拒绝块仍是原源码的注释状态，没有在发布时恢复或进一步删除。没有 Python 柔顺动作支持、自动恢复、自动视觉重规划或障碍规划。基坐标系和工作范围必须由现场操作者确认，不可从示例数值推导安全保证。
