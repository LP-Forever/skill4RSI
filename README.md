# skill4RSI

**当前使用：`v2/` 的 V2 STANDARD skill 链路。`ros/` 是初版实验链路，不是当前默认方案。**

本项目是任务侧接入程序的整理快照：模型看图/状态/任务/memory 做推理，执行器转换输出并交付已经安装的机器人控制服务。它不是完整的 Franka 驱动或机器人控制栈。

```text
skill4RSI/
├── v2/  当前使用：一次生成完整 skill 序列 → V2 Python 有限动作
├── ros/ 初版实验：每轮 20×7 action chunk → ROS/FrankaPy 动态位姿参考
├── v2_standard_compliant_control.png  已生成的 V2 标准/柔顺合并图
├── README.md
├── GITHUB_UPLOAD.md                 人工提交步骤，本次未执行
├── SOURCE_MANIFEST.json             来源哈希及仅为打包做的改动
└── VALIDATION.md                    离线验证范围和结果
```

- [当前 V2 链路说明、使用方法与数据流](v2/README.md)
- [ROS 初版说明、使用方法与数据流](ros/README.md)
- [提交到 LP-Forever 的步骤](GITHUB_UPLOAD.md)

## 版本边界

| 项目 | V2（当前） | ROS（初版） |
|---|---|---|
| 模型输出 | 1–64 个显式 skill，一次规划必要完整序列 | 每轮 20×7 增量动作，或无动作的 done/error |
| 执行交付 | 绝对 TCP 位置 + wxyz 四元数，调用 V2 `move_pose()` | 15 Hz 流式位姿与六维笛卡尔刚度，交付外部 ROS 栈 |
| 最终真机执行 | 外部 V2 native-pose / 内部关节阻抗 | 外部 ROS torque / 笛卡尔阻抗 |
| 重看图 | 正常序列中不重规划；结束复盘不产生新动作 | 完成整个 chunk 后重看图并推理下一轮 |
| 状态 | 正在使用 STANDARD；Python 柔顺有限动作尚未实现 | 初版集成，不作为已验证的完整搭建方案 |

## 不包含什么

不包含 V2 C++ 服务、V2 Python SDK `teleop_v2`、libfranka、FrankaPy、ROS 工作空间、原生机械臂/夹爪驱动、模型桥接服务、模型权重、原相机驱动、原旧版轨迹执行器、GPU IK/FK。ROS 所需的纯插值数学被单独抽出，而非打包原控制链路。

memory 只有空目录框架（`.gitkeep`）；没有历史 episode、经验规则、原始任务参考照片、真实相机序列号或本机标定。配置是待填写的移植模板。**从 GitHub 下载并不代表可以直接驱动任意机械臂。**

## 发布说明

这是当前源码的可移植打包，不修改正在运行的项目/模型/控制服务。为去掉无关旧任务与机器路径，提取了必要的模型 HTTP、相机传输、V2 状态检查及 ROS 纯几何 helper；来源和具体改动见 `SOURCE_MANIFEST.json`。未关闭原生保护，未实现 Python COMPLIANT 支持。

硬件版本、SDK、坐标系、相机身份、工具/负载与现场标定必须匹配。原生动作成功不等于目标物体或建筑任务成功。故障时停止，不自动清锁、恢复、重放或回 HOME；停止命令不能替代现场急停。

当前暂不指定开源许可证：代码归属及提取 helper 的授权范围需由拥有者核实后决定。未上传上游控制栈或第三方库源码，不等于自动拥有所有提取代码的公开再许可权。
