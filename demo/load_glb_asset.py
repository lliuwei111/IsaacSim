import os
import asyncio
import numpy as np
from isaacsim import SimulationApp

# 1. 启动仿真器并开启 WebRTC 推流 (Headless 模式)
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")
for _ in range(10):
    simulation_app.update()

import omni.kit.asset_converter
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.prims import RigidPrim
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim
from omni.physx.scripts import physicsUtils
from omni.isaac.core.materials import PhysicsMaterial
from omni.physx.scripts.utils import setCollider
from omni.isaac.core.objects import FixedCuboid
from pxr import UsdGeom, Gf

# ================= 路径配置 =================
GLB_FILE_PATH = "/workspace/my_code/IsaacSim/biscuit.glb"  # 替换为你的 GLB 路径
USD_OUTPUT_PATH = "/workspace/my_code/IsaacSim/biscuit.usd"
# ============================================

# 2. 阻塞等待视口初始化 (防止 WebRTC 黑屏)
viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

# 3. 异步函数：将 GLB 转换为底层的 USD 格式
async def convert_glb_to_usd(input_glb, output_usd):
    print(f"🔄 正在将 {input_glb} 转换为 USD...")
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = task_manager.create_converter_task(input_glb, output_usd, None, context)
    success = await task.wait_until_finished()
    if success:
        print(f"✅ 转换成功！保存在: {output_usd}")
    return success

if not os.path.exists(USD_OUTPUT_PATH):
    asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(GLB_FILE_PATH, USD_OUTPUT_PATH))

# 4. 创世：设置物理世界、地面、灯光和摄像机
world = World(stage_units_in_meters=1.0)
world.scene.add(
    FixedCuboid(
        prim_path="/World/Ground",
        name="ground",
        position=np.array([0.0, 0.0, -0.05]), 
        scale=np.array([10.0, 10.0, 0.1]),
        color=np.array([0.5, 0.5, 0.5])
    )
)
create_prim("/World/defaultLight", "DomeLight", attributes={"inputs:intensity": 1000.0})
set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0.0, 0.0, 0.0]))

# 5. 加载主角：Franka 机械臂
franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka_robot"))

# 6. 【终极工程修复】：使用“隐形代理文件夹”策略解决尺度和轴向问题
print("🧱 正在构建物理代理与资产视觉微调...")

# 6.1 创建一个不可见的 Xform Prim (文件夹) 作为机器人的物理抓取目标
# 我们不需要 DynamicCuboid 了，用最纯净的 Xform，原点设在中心
proxy_prim_path = "/World/Biscuit_Proxy"
proxy_prim = UsdGeom.Xform.Define(simulation_app.context.get_stage(), proxy_prim_path)

# 6.2 将代理文件夹包装为受物理控制的 RigidPrim (刚体)
# 注意：我们将代理放在 [0.3, 0.3, 0.1] 悬空位置，让它掉下来测试物理
target_obj = world.scene.add(
    RigidPrim(
        prim_path=proxy_prim_path,
        name="target_obj",
        position=np.array([0.3, 0.3, 0.1]), # 放在仿真世界的 0.1米 半空中
        # 🌟 此处的 scale 是物理代理的 Scale，设为 [1, 1, 1] 米制单位
        scale=np.array([0.07, 0.07, 0.07]),     
        mass=0.15                             # 设为饼干的大概重量 150g
    )
)

# 6.3 【核心】：将饼干视觉 USD 引用进这个“代理文件夹”里，作为 Child
visual_prim_path = proxy_prim_path + "/VisualModel"
add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=visual_prim_path)

# 6.4 【终极微调】：在代理文件夹内部，调整饼干的 local (局部) XYZ、旋转和大小
visual_prim = UsdGeom.Xform.Get(simulation_app.context.get_stage(), visual_prim_path)
xform_api = UsdGeom.XformCommonAPI(visual_prim)

# 💡 解决“太大”：根据图像 9 的视觉对比，TRELLIS 导出的尺度可能需要放小 100 倍
# 这个数值需要根据你的具体模型微调，直到它看起来跟正常饼干盒一样大 (例如 X方向 5cm, Y方向 10cm, Z方向 18cm)
new_scale = Gf.Vec3f(0.01, 0.01, 0.01) # 尝试缩小 100倍。如果看起来太小，就改成 0.02
xform_api.SetScale(new_scale)

# 💡 解决“放反了”：绕 X轴 旋转 90 度让饼干立起来，绕 Z轴 旋转 90度把窄面朝向机器人
# 这个旋转角度通常是 (90, 0, 0), (-90, 0, 0) 或 (90, 0, 90) 飞几下试试
xform_api.SetRotate((90, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)

# 💡 解决“原点偏移”：如果按完 'F' 键聚焦它，发现它歪在盒子的角落
# 你可以在这里用 SetTranslate(Gf.Vec3d(delta_x, delta_y, delta_z)) 
# 手动把它拉到代理文件夹的正中心 [0,0,0]。通常需要 Z轴 偏移它自身高度的一半
# xform_api.SetTranslate(Gf.Vec3d(0, 0, 0)) # 暂时不调，如果掉下来总是弹飞再微调

# 6.5 赋予极其关键的“碰撞网格”属性 (绑定给视觉子节点，而不是代理父节点)
final_visual_prim = simulation_app.context.get_stage().GetPrimAtPath(visual_prim_path)
setCollider(final_visual_prim, approximationShape="boundingCube")

# 6.6 【适配上次报错】：底层绑定高摩擦材质到视觉子节点 Prim 上
high_friction_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/HighFriction",
    dynamic_friction=3.0,  # 设得更高，彻底解决“滑”
    static_friction=3.0,
    restitution=0.0
)
stage = simulation_app.context.get_stage()
physicsUtils.add_physics_material_to_prim(
    stage, 
    final_visual_prim,                # 绑定到视觉 Mesh 节点
    high_friction_material.prim_path
)

# 7. 初始化抓取控制器
controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)

print("正在编译底层运动学图 (RMPflow)...")
world.reset()
print("👐 正在初始化夹爪，强制张开...")
franka.gripper.open() # 发送张开指令

# 强制物理引擎空跑 60 帧（大约 1 秒钟）
# 让夹爪有充足的物理时间在原地完全滑开，而不是一边冲向饼干一边开
for _ in range(60):
    world.step(render=True)

# 定义搬运的目标点
pos_A = np.array([0.3, 0.3, 0.05])   
pos_B = np.array([0.3, -0.3, 0.05])  
current_place_pos = pos_B             

print("✅ 环境就绪！请在 WebRTC 浏览器中查看机器人抓取 AI 生成的资产。")

# 8. 仿真闭环主循环
while simulation_app.is_running():
    world.step(render=True)
    
    if world.is_playing():
        # 实时获取资产的位置
        obj_pos, _ = target_obj.get_world_pose()

        grasp_target_pos = obj_pos + np.array([0.0, 0.0, 0.01])
        
        actions = controller.forward(
            picking_position=grasp_target_pos,
            placing_position=current_place_pos,
            current_joint_positions=franka.get_joint_positions(),
            # 如果模型比较矮，可以给夹爪加一点向下的 Z 轴偏移
            end_effector_offset=np.array([0, 0, 0.01]), 
        )
        
        franka.apply_action(actions)
        
        # 状态机循环
        if controller.is_done():
            if np.allclose(current_place_pos, pos_B):
                current_place_pos = pos_A
            else:
                current_place_pos = pos_B
            controller.reset()

world.stop()
simulation_app.close()