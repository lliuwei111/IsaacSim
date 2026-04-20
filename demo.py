# 1. 启动仿真引擎 (必须放在最前面)
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

# 2. 开启推流扩展
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")

# ==========================================
# 【关键修复 1】: 让引擎空跑几帧，等待推流插件和 UI 完全加载完毕
for _ in range(10):
    simulation_app.update()
# ==========================================

# 3. 导入核心 API
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np

# 4. 创建物理世界与地面
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 5. 添加环境光
create_prim(
    prim_path="/World/DefaultLight",
    prim_type="DomeLight",
    attributes={"inputs:intensity": 1000.0}
)

# 6. 添加动态方块
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/MyCube",
        name="my_cube",
        position=np.array([0, 0, 5.0]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0.0, 0.5, 1.0]),
    )
)

# 7. 设置相机视角 (保证能看到方块掉落)
set_camera_view(eye=np.array([8.0, 8.0, 6.0]), target=np.array([0.0, 0.0, 0.0]))

# 8. 初始化物理状态
world.reset()

# ==========================================
# 【关键修复 2】: 在进入死循环前，强制把刚才创建的所有物体同步到前端 UI 和渲染器
for _ in range(10):
    simulation_app.update()
# ==========================================

print("=======================================")
print("修复版 Demo 已启动！请在浏览器刷新 http://localhost:8211")
print("=======================================")

# 9. 仿真主循环
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()