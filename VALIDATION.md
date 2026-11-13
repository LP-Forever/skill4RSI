# 发布包验证记录

日期：2026-11-13。**当前使用版本是 V2 STANDARD；ROS 是初版实验版本。**

本记录只覆盖整理后的发布包。验证没有向模型桥发送推理请求，没有接触正在运行的相机/V2/ROS 服务，没有发送机械臂或夹爪动作，也没有修改原运行项目。原实验记录不等于本发布包的真机验收结果。

## 已通过的离线验证

| 项目 | 结果 | 范围 |
|---|---|---|
| V2 单元测试 | 24/24 通过 | fake 模型/机器人、skill 合同、执行目标、原生结果等待、参考图、token usage |
| ROS 单元测试 | 23/23 通过 | chunk 合同、纯几何、参考图、token usage；不加载真实 ROS 控制栈 |
| Python 编译检查 | 37 个文件通过 | 使用 `compile()`，不产生 `__pycache__` |
| ROS 执行端 Python 3.8 语法检查 | 6 个文件通过 | `ast.parse(feature_version=(3,8))`；不是 Python 3.8 运行时/ROS 依赖验收 |
| Shell 语法 | 通过 | `bash -n ros/run_task.sh` |
| 提取源码一致性 | 22 项通过 | 函数/类 AST 对比，忽略位置属性；仅对说明过的 import 与相机脚本文件名重命名做归一化 |
| ROS 纯几何数值对照 | 30 个随机 20 行 chunk 通过 | 提取原始纯数学节点，与发布 helper 的 waypoint、旋转、插值、夹爪序列和时间戳对照；`rtol=0, atol=1e-12` |
| CLI 帮助 | 7 个入口通过 | 只执行 `--help`，不启动监听、不执行任务 |
| 配对参考导入 | 两个分支均通过 | 临时副本中 8 张合成 640×360 JPEG → manifest/hash → 原 loader；重复 set-id 拒绝覆盖 |
| 发布内容清理 | 通过 | memory 与 runs 仅零字节 `.gitkeep`；无 `.git`/缓存/真实参考照片；Python 文件无个人 `/home/` 路径；常见明文密钥模式扫描未命中 |
| PNG | 原文件一致 | SHA256 与原先生成的合并图一致；没有重新绘制或替换 |

ROS 六个语法检查文件：`run_task.py`、`halo_pipeline_ros.py`、`halo_ros_runtime.py`、`halo_ros_protocol.py`、`trajectory_executor_ros.py`、`ros_chunk_geometry.py`。

CLI 帮助入口：V2 的 `start_model.py`、`run_task.py`、`prepare_references.py`；ROS 的 `halo_ros_action_service.py`、`halo_ros_camera_service.py`、`run_task.py`、`prepare_references.py`。

22 项源码对比包含：V2 两个状态/四元数函数、HTTP helper、相机 IPC 类、五个监测方法；两分支各四项模型 HTTP helper；ROS 五项纯数学。原 executor 没有被导入：数值对照只对提取的纯数学 AST 执行，避免初始化 ROS/GPU/机器人依赖。来源和每个发布文件的 SHA256 见 `SOURCE_MANIFEST.json`。

## 可以自行复查

在各自已安装离线测试依赖的 Python 3.11 环境里：

```bash
cd v2
python -B -m unittest discover -s tests -v
cd ../ros
python -B -m unittest discover -s tests -v
bash -n run_task.sh
```

测试使用临时 fixture，包含的模型名、图像和操作结果不代表实际调用或真实成功证据。

## 尚未验证／不能据此宣称的内容

- 尚未用这个移植目录启动真实模型、相机、V2 SDK 或 ROS 接口并执行任务；没有新增硬件验收。
- 未验证另一台机器的模型桥认证、模型名、SSE、SDK 与 native 服务版本是否兼容。
- 未验证真实 ROS 消息/Action、动态技能交接、取消和底层 torque 控制时序。Python 3.8 语法解析不证明这些可运行。
- 未验证现场坐标系、标定、工具/负载、物体位置、碰撞风险或最终建筑效果。
- 没有把 ROS 初版称为经过完整搭建成功验收的方案；当前实际使用仍是 V2 STANDARD。
- 没有实现 V2 Python 柔顺有限动作，没有进一步注释或删除原生保护。
- 密钥模式扫描不是完整安全审计；提交前仍需人工检查将要暂存的所有文件、归属和许可。

配置和真实 memory 在发布包中刻意留空／占位。离线测试通过代表打包逻辑的有限检查通过，不代表下载后可以不经配置与现场确认就运行真机。
