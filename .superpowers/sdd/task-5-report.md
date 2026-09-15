# Task 5 交付报告：实物跟踪实验汇报 PPT

## 交付内容

- 生成了 `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx`。
- PPT 固定为 9 页，按任务简报的标题、方法、条件、两组实验、视频占位页和结论顺序组织。
- 生成脚本扩展为同时支持 Word 和 PPT；使用 `--skip-word --ppt-out` 可只生成 PPT。
- 没有改动条件表、指标表、图表或任何原始数据。

## 事实与边界

- 所有误差叙述保持 1.0 m 理想编队半径口径。
- 控制方法页明确区分 Follower 前向预测（状态、最近命令、时间常数 tau）和 Leader 命令速度前馈（可选 cmd_vel 增量支路）。
- HPC/LPC 页保留 LPC lambda 初值与 Leader 速度共同变化、lambda=1.5 / 0.25 m/s 发生碰撞，因而不作严格单变量优劣结论的安全边界。

## 视频占位页

第 8 页仅含原生可编辑文本和形状：一个 16:9 边框矩形、所需标题及建议实验条件副标题。PPT archive 检查结果：无视频条目、无 video 关系、无外部关系；未嵌入、链接、复制或生成视频文件。

## 验证记录

```text
/tmp/homo-report-venv/bin/python -m unittest homo_multirobot_formation_control/test/test_build_real_tracking_deliverables.py
Ran 3 tests ... OK

slide_count=9
slide8_shape_count=5
slide8_has_16_by_9_placeholder=True
slide8_has_external_relationship=False
slide8_has_video_relationship=False
embedded_or_linked_video_entries=[]
slide8_placeholder_title=True
slide8_placeholder_subtitle=True
```

## 关注项

PPT 中的图表以现有 PNG 图像展示；文字、标题、边框、占位区和布局形状均可在 PowerPoint 中编辑。第 8 页保留为空白手动插入区，需由用户自行插入最新视频。
