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

# 🌟 新增：导入相机模块
from omni.isaac.sensor import Camera

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
    # 🌟 1. 动态清理底层单例内存
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
    # 🌟 2. 资产转换 (GLB -> USD)
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
    # 🌟 3. 创世：强力注入物理上下文与场景基础
    # ====================================================================
    world = World(stage_units_in_meters=1.0)
    
    if world.get_physics_context() is None:
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
    # 🌟 4. 加载机器人与目标物体 (已修复缩放与质量问题)
    # ====================================================================
    print("🤖 正在加载机械臂...")
    franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka_robot"))

    print("🧱 正在构建物理代理...")
    stage = omni.usd.get_context().get_stage()
    
    proxy_prim_path = "/World/Biscuit_Proxy"
    UsdGeom.Xform.Define(stage, proxy_prim_path)
    
    # 修复点：修改为更真实的质量 (50g)，并在父级统一缩放
    target_obj = world.scene.add(
        RigidPrim(
            prim_path=proxy_prim_path,
            name="target_obj",
            position=np.array([0.3, 0.3, 0.1]),
            scale=np.array([0.1, 0.1, 0.1]), # 统一使用真实大小缩放
            mass=0.05                           # 0.05 kg = 50g 
        )
    )

    visual_prim_path = proxy_prim_path + "/VisualModel"
    add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=visual_prim_path)

    visual_prim = UsdGeom.Xform.Get(stage, visual_prim_path)
    xform_api = UsdGeom.XformCommonAPI(visual_prim)
    # 修复点：删除子级的缩放，防止碰撞体生成异常，仅保留模型本身需要的旋转
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
    # 🌟 4.5 添加场景俯视相机 (模拟真实视觉传感器)
    # ====================================================================
    print("📷 正在安装视觉传感器...")
    camera = Camera(
        prim_path="/World/OverheadCamera",
        position=np.array([0.5, 0.0, 1.0]),               # 相机在桌子正上方 1米处
        orientation=np.array([0.0, 0.7071, 0.0, 0.7071]), # 镜头垂直朝下
        resolution=(640, 480),
    )
    camera.initialize()

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
    
    # 核心：提前播放，让物理引擎启动
    world.play()

    print("👐 正在初始化夹爪并等待物体稳定掉落...")
    franka.gripper.open() 

    # 等待 60 个物理帧，确保饼干掉到桌面上并静止
    for _ in range(60):
        await omni.kit.app.get_app().next_update_async()

    # ====================================================================
    # 🌟 6. 真实状态机：视觉感知 -> 动作规划 -> 执行
    # ====================================================================
    class RobotState:
        WAITING_FOR_VISION = 0  
        EXECUTING_GRASP = 1     
        DONE = 2                

    state_machine = {
        "current_state": RobotState.WAITING_FOR_VISION,
        "vision_timer": 0,
        "perceived_target_pos": None, 
        "place_pos": np.array([0.3, -0.3, 0.05])
    }

    # 🧠 模拟视觉算法处理 (YOLO + Depth)
    def simulate_vision_pipeline():
        # 获取底层真实物理坐标
        true_pos, _ = target_obj.get_world_pose()
        # 注入 5mm 的高斯噪声，模拟真实相机的标定误差和深度噪点
        noise = np.random.normal(0, 0.005, 3) 
        calculated_pos = true_pos + noise
        print(f"👁️ 视觉系统识别完毕！\n  > 真实坐标: {true_pos}\n  > 视觉计算坐标 (带噪声): {calculated_pos}")
        return calculated_pos

    # 🦾 物理引擎每一帧的回调
    def on_physics_step(step_size):
        if not world.is_playing():
            return

        # 阶段 1：视觉搜索与位姿计算
        if state_machine["current_state"] == RobotState.WAITING_FOR_VISION:
            state_machine["vision_timer"] += 1
            # 模拟相机拍照和 AI 模型推理耗时 (30 帧)
            if state_machine["vision_timer"] > 30:
                detected_pos = simulate_vision_pipeline()
                state_machine["perceived_target_pos"] = detected_pos
                state_machine["current_state"] = RobotState.EXECUTING_GRASP
                print("🤖 获取视觉反馈，开始执行抓取！")

        # 阶段 2：执行盲抓 (基于视觉反馈，不再实时查询物理真值)
        elif state_machine["current_state"] == RobotState.EXECUTING_GRASP:
            # 获取视觉认定的坐标，并抬高 Z 轴 (防止夹爪戳穿地面)
            grasp_pos = state_machine["perceived_target_pos"] + np.array([0.0, 0.0, 0.015])
            
            actions = controller.forward(
                picking_position=grasp_pos,
                placing_position=state_machine["place_pos"],
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0, 0, 0.02]), 
            )
            franka.apply_action(actions)
            
            if controller.is_done():
                print("✅ 任务完成！饼干盒已移动到指定位置。")
                state_machine["current_state"] = RobotState.DONE

    # 将状态机挂载到物理引擎
    world.add_physics_callback("real_robot_logic", on_physics_step)
    
# 启动异步任务
asyncio.ensure_future(setup_and_run())