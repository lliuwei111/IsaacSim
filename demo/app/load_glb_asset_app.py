import os
import asyncio
import numpy as np

import omni.kit.app
import omni.kit.asset_converter
import omni.usd
import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.prims import RigidPrim
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.physx.scripts import physicsUtils
from omni.isaac.core.materials import PhysicsMaterial
from omni.physx.scripts.utils import setCollider
from pxr import UsdGeom, Gf

# ================= 自定义路径配置 =================
GLB_FILE_PATH = "/workspace/my_code/IsaacSim/biscuit.glb"  
USD_OUTPUT_PATH = "/workspace/my_code/IsaacSim/biscuit.usd"
# ===============================================

async def setup_and_run():
    # ====================================================================
    # 🌟 0. 强力终止环境：确保物理引擎和时间轴完全停转
    # ====================================================================
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    await omni.kit.app.get_app().next_update_async()

    print("🔄 正在由代码接管并彻底清空场景...")
    await omni.usd.get_context().new_stage_async()
    await omni.kit.app.get_app().next_update_async()
    await omni.kit.app.get_app().next_update_async()

    # ====================================================================
    # 🌟 1. 动态清理底层单例内存 (防追踪崩溃核心)
    # 兼容最新版 4.x 和旧版 2023.x 的各种混乱模块
    # ====================================================================
    try:
        from isaacsim.core.api.simulation_context import SimulationContext
        from isaacsim.core.api.physics_context import PhysicsContext
        from isaacsim.core.api import World as ApiWorld
        ApiWorld.clear_instance()
        SimulationContext.clear_instance()
        if hasattr(PhysicsContext, "clear_instance"):
            PhysicsContext.clear_instance()
    except ImportError:
        pass
    
    if World.instance() is not None:
        World.instance().clear_all_callbacks()
        World.instance().clear_instance()

    # ====================================================================
    # 🌟 2. 资产转换
    # ====================================================================
    if not os.path.exists(USD_OUTPUT_PATH):
        print(f"🔄 正在将 {GLB_FILE_PATH} 转换为 USD...")
        task_manager = omni.kit.asset_converter.get_instance()
        context = omni.kit.asset_converter.AssetConverterContext()
        task = task_manager.create_converter_task(GLB_FILE_PATH, USD_OUTPUT_PATH, None, context)
        success = await task.wait_until_finished()
        if not success:
            print("❌ 转换失败，请检查 GLB 路径！")
            return
        print(f"✅ 转换成功！保存在: {USD_OUTPUT_PATH}")

    # ====================================================================
    # 🌟 3. 创世：强力注入物理上下文 (根治 NoneType warm_start 的核心)
    # ====================================================================
    world = World(stage_units_in_meters=1.0)
    
    # 💡 终极修复点：如果 World 没拿到物理环境，手动硬塞一个进去！
    if world.get_physics_context() is None:
        print("⚠️ 检测到物理上下文游离，正在强行注入...")
        try:
            from isaacsim.core.api.physics_context import PhysicsContext
        except ImportError:
            from omni.isaac.core.physics_context import PhysicsContext
        world._physics_context = PhysicsContext()

    # 地面
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Ground",
            name="ground",
            position=np.array([0.0, 0.0, -0.05]), 
            scale=np.array([10.0, 10.0, 0.1]),
            color=np.array([0.5, 0.5, 0.5])
        )
    )
    
    # 灯光
    create_prim(
        prim_path="/World/defaultLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 2000.0}
    )
    set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0.0, 0.0, 0.0]))

    # ====================================================================
    # 🌟 4. 加载机器人资产与代理
    # ====================================================================
    print("🤖 正在加载机械臂...")
    franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka_robot"))

    print("🧱 正在构建物理代理...")
    stage = omni.usd.get_context().get_stage()
    
    proxy_prim_path = "/World/Biscuit_Proxy"
    UsdGeom.Xform.Define(stage, proxy_prim_path)
    target_obj = world.scene.add(
        RigidPrim(
            prim_path=proxy_prim_path,
            name="target_obj",
            position=np.array([0.3, 0.3, 0.1]),
            scale=np.array([0.1, 0.1, 0.1]),
            mass=0.05                            
        )
    )

    visual_prim_path = proxy_prim_path + "/VisualModel"
    add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=visual_prim_path)

    visual_prim = UsdGeom.Xform.Get(stage, visual_prim_path)
    xform_api = UsdGeom.XformCommonAPI(visual_prim)
    # xform_api.SetScale(Gf.Vec3f(0.01, 0.01, 0.01)) 
    xform_api.SetRotate((90, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    final_visual_prim = stage.GetPrimAtPath(visual_prim_path)
    setCollider(final_visual_prim, approximationShape="convexHull")

    high_friction_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/HighFriction",
        dynamic_friction=3.0,  
        static_friction=3.0,
        restitution=0.0
    )
    physicsUtils.add_physics_material_to_prim(stage, final_visual_prim, high_friction_material.prim_path)

    # ====================================================================
    # 🌟 5. 初始化控制器与物理预热
    # ====================================================================
    controller = PickPlaceController(
        name="pick_place_controller",
        gripper=franka.gripper,
        robot_articulation=franka,
    )

    print("⏳ 正在编译底层运动学图 (RMPflow)...")
    await world.reset_async() 
    
    # 💡 核心修复 1：把播放按钮提前！让物理引擎先转起来
    world.play()

    print("👐 正在初始化夹爪并等待物理掉落...")
    franka.gripper.open() 

    # 💡 核心修复 2：使用最底层的引擎帧等待，彻底抛弃有 Bug 的 step_async()
    for _ in range(60):
        await omni.kit.app.get_app().next_update_async()

    # ====================================================================
    # 🌟 6. 核心动作逻辑
    # ====================================================================
    state = {
        "pos_A": np.array([0.3, 0.3, 0.05]),
        "pos_B": np.array([0.3, -0.3, 0.05]),
        "current_place_pos": np.array([0.3, -0.3, 0.05])
    }

    def on_physics_step(step_size):
        if world.is_playing():
            obj_pos, _ = target_obj.get_world_pose()
            grasp_target_pos = obj_pos + np.array([0.0, 0.0, 0.01])
            
            actions = controller.forward(
                picking_position=grasp_target_pos,
                placing_position=state["current_place_pos"],
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0, 0, 0.01]), 
            )
            
            franka.apply_action(actions)
            
            if controller.is_done():
                if np.allclose(state["current_place_pos"], state["pos_B"]):
                    state["current_place_pos"] = state["pos_A"]
                else:
                    state["current_place_pos"] = state["pos_B"]
                controller.reset()

    # 将控制逻辑挂载到物理引擎
    world.add_physics_callback("pick_place_logic", on_physics_step)
    
    print("✅ 环境就绪！机器人开始自动抓取。")
    # 因为上面已经 play() 过了，这里就不需要再调用 world.play() 了

# 启动！
asyncio.ensure_future(setup_and_run())