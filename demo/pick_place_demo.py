import sys
import numpy as np
from isaacsim import SimulationApp

# 1. 启动仿真器 (开启无头模式)
simulation_app = SimulationApp({"headless": True})

# 2. 开启 WebRTC 推流扩展
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")
for _ in range(10):
    simulation_app.update()

# 导入 Isaac Sim 核心高级库
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.objects import FixedCuboid
from isaacsim.core.utils.prims import create_prim

# 3. 解决 WebRTC 黑屏：阻塞等待视口初始化
viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

# 4. 创建物理世界 (World 是对 SimulationContext 的更高级封装)
world = World(stage_units_in_meters=1.0)
world.scene.add(
    FixedCuboid(
        prim_path="/World/Ground",
        name="ground",
        # 将 Z 轴设为 -0.05（因为厚度是0.1），这样它的上表面刚好完美贴合 Z=0
        position=np.array([0.0, 0.0, -0.05]), 
        scale=np.array([10.0, 10.0, 0.1]),    # 创建一个 10米 x 10米 的宽广地面
        color=np.array([0.5, 0.5, 0.5])       # 设置为灰色
    )
)
create_prim("/World/defaultLight", "DomeLight", attributes={"inputs:intensity": 1000.0})

# 设置一个好点的观察视角
set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0.0, 0.0, 0.0]))

# 5. 加载 Franka 机械臂
franka = world.scene.add(
    Franka(prim_path="/World/Franka", name="franka_robot")
)

# 6. 加载要抓取的蓝色小方块 (5cm大小)
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.3, 0.3, 0.025]), # 初始位置 (右侧)
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0.0, 0.0, 1.0]),
        mass=0.1
    )
)

# 7. 初始化官方内置的抓取放置控制器 (内置了状态机和逆运动学)
controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)

# 必须先 reset 一次世界，底层才会编译物理和运动学引擎
print("正在编译底层运动学图 (RMPflow)，请稍候 (可能需要5-10秒)...")
world.reset()

# 定义两个目标位置，用于来回搬运
pos_A = np.array([0.3, 0.3, 0.025])   # 右侧
pos_B = np.array([0.3, -0.3, 0.025])  # 左侧
current_place_pos = pos_B             # 第一次任务：从 A 搬到 B

print("====================================================")
print("✅ 抓取环境就绪，WebRTC 已启动！")
print("👉 请在本地浏览器中打开 http://localhost:8211 查看动画")
print("====================================================")

# 8. 仿真主循环
while simulation_app.is_running():
    # 推进物理世界并渲染推流
    world.step(render=True)
    
    if world.is_playing():
        # 实时获取方块的当前坐标
        cube_pos, _ = cube.get_world_pose()
        
        # 将当前状态喂给控制器，计算出机械臂各个关节应该怎么动
        actions = controller.forward(
            picking_position=cube_pos,
            placing_position=current_place_pos,
            current_joint_positions=franka.get_joint_positions(),
            # 给夹爪加一点 Z 轴向下偏移，确保能对准方块中心抓紧
            end_effector_offset=np.array([0, 0, 0.02]), 
        )
        
        # 将计算出的动作指令发送给机械臂底层关节
        franka.apply_action(actions)
        
        # 检查控制器的内置状态机是否走完了 "松开夹爪并撤回" 的最后一步
        if controller.is_done():
            print("🎯 一次搬运完成，准备交换目标位置！")
            
            # 逻辑反转：如果刚才放到了 B，下一次就把目标改成 A
            if np.allclose(current_place_pos, pos_B):
                current_place_pos = pos_A
            else:
                current_place_pos = pos_B
            
            # 重置控制器的内部状态机，准备进行下一次全新的抓取循环
            controller.reset()

# 退出清理
world.stop()
simulation_app.close()