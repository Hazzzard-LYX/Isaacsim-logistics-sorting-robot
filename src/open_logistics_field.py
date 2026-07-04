#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 Isaac Sim GUI 中生成并打开物流比赛场地场景。

用法（在 env_isaaclab 环境中）：
    cd /home/hazzzard/机电一体化课程设计
    python open_logistics_field.py
"""

from isaacsim import SimulationApp

# 必须先启动 SimulationApp，再导入 pxr / omni 相关模块
simulation_app = SimulationApp({"headless": False})

import omni.usd

from create_logistics_field import OUTPUT_USD, main
from final_robot_model import import_final_robot_model

if __name__ == "__main__":
    saved_path = main()
    ctx = omni.usd.get_context()
    ctx.open_stage(saved_path)
    for _ in range(30):
        simulation_app.update()
    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("USD Stage 加载失败")
    import_final_robot_model(stage, apply_home_pose=True)
    print(f"[LogisticsField] 已打开场景: {saved_path}")

    while simulation_app.is_running():
        simulation_app.update()
