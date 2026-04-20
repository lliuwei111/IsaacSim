# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import math
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})
# 🌟🌟🌟 新增 1：开启 WebRTC 推流扩展 🌟🌟🌟
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")

# 空跑几帧，确保 WebRTC 服务完全启动就绪，防止黑屏
for _ in range(10):
    simulation_app.update()


from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.prims import create_prim  # 新增：用于创建光源和地面

# 🌟🌟🌟 附加优化：添加一个环境光和地面，让画面更真实 🌟🌟🌟
create_prim("/World/defaultLight", "DomeLight")
from pxr import Gf
create_prim("/World/defaultGround", "Plane", attributes={"size": 100.0})
# 或者先创建prim，后设置属性
ground_prim = create_prim("/World/defaultGround", "Plane")

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot = add_reference_to_stage(usd_path=asset_path, prim_path="/Franka")
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")
simulation_app.update()

simulation_context = SimulationContext()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize()
dof_ptr = art.get_dof_index("panda_joint2")

simulation_app.update()

simulation_context.play()
# NOTE: before interacting with physics directly you need to step physics for one step at least
# simulation_context.step(render=True) which happens inside .play()
frame = 0
while simulation_app.is_running():
    # 使用 sin 函数生成一个在 -1.5 到 0 之间来回摆动的目标位置
    # 公式解析: -0.75 为中心点，幅度为 0.75，速度为 0.02
    target_pos = -0.75 + 0.75 * math.sin(frame * 0.02)
    
    # 持续发送新的关节位置指令
    art.set_joint_positions([[target_pos]], joint_indices=[dof_ptr])
    
    # 步进物理仿真并渲染画面推流
    simulation_context.step(render=True)
    frame += 1

# print("Reached: ", np.array2string(art.get_joint_positions(), precision=3, suppress_small=True))

simulation_context.stop()
simulation_app.close()
