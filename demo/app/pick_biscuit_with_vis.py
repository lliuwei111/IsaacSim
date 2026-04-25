import os
import asyncio
import struct
import json
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

from omni.isaac.sensor import Camera

GLB_FILE_PATH = "/workspace/IsaacSim/biscuit.glb"
USD_OUTPUT_PATH = "/workspace/IsaacSim/biscuit.usd"


def extract_textures_from_glb(glb_path, usd_dir):
    usd_dir = os.path.dirname(os.path.abspath(usd_dir))
    textures_dir = os.path.join(usd_dir, "textures")
    try:
        with open(glb_path, "rb") as f:
            magic, version, length = struct.unpack("<III", f.read(12))
            json_len, json_type = struct.unpack("<II", f.read(8))
            json_data = json.loads(f.read(json_len))
            bin_len, bin_type = struct.unpack("<II", f.read(8))
            bin_data = f.read(bin_len)

        if "images" not in json_data:
            return

        os.makedirs(textures_dir, exist_ok=True)
        for i, img_info in enumerate(json_data["images"]):
            bv_idx = img_info.get("bufferView")
            mime = img_info.get("mimeType", "")
            if bv_idx is None or bv_idx >= len(json_data["bufferViews"]):
                continue
            bv = json_data["bufferViews"][bv_idx]
            offset = bv.get("byteOffset", 0)
            size = bv["byteLength"]
            img_data = bin_data[offset:offset + size]

            ext = ".png" if "png" in mime else ".jpg"
            out_name = f"biscuit_texture{i}{ext}"
            out_path = os.path.join(textures_dir, out_name)
            with open(out_path, "wb") as out:
                out.write(img_data)
            print(f"提取纹理: {out_path} ({len(img_data)} bytes)")
    except Exception as e:
        print(f"提取纹理失败: {e}")


async def setup_and_run():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    await omni.kit.app.get_app().next_update_async()

    print("正在清空场景...")
    await omni.usd.get_context().new_stage_async()
    await omni.kit.app.get_app().next_update_async()
    await omni.kit.app.get_app().next_update_async()

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

    if not os.path.exists(USD_OUTPUT_PATH):
        print(f"正在将 {GLB_FILE_PATH} 转换为 USD...")
        task_manager = omni.kit.asset_converter.get_instance()
        context = omni.kit.asset_converter.AssetConverterContext()
        task = task_manager.create_converter_task(GLB_FILE_PATH, USD_OUTPUT_PATH, None, context)
        success = await task.wait_until_finished()
        if not success:
            print("转换失败，请检查 GLB 路径！")
            return
        print(f"转换成功！保存在: {USD_OUTPUT_PATH}")
        extract_textures_from_glb(GLB_FILE_PATH, USD_OUTPUT_PATH)

    world = World(stage_units_in_meters=1.0)

    if world.get_physics_context() is None:
        try:
            from isaacsim.core.api.physics_context import PhysicsContext
        except ImportError:
            from omni.isaac.core.physics_context import PhysicsContext
        world._physics_context = PhysicsContext()

    world.scene.add(
        FixedCuboid(
            prim_path="/World/Ground",
            name="ground",
            position=np.array([0.0, 0.0, -0.05]),
            scale=np.array([10.0, 10.0, 0.1]),
            color=np.array([0.5, 0.5, 0.5])
        )
    )

    create_prim(
        prim_path="/World/defaultLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 2000.0}
    )
    set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0.0, 0.0, 0.0]))

    print("正在加载机械臂...")
    franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka_robot"))

    print("正在构建物理代理...")
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

    print("正在安装视觉传感器...")
    camera = Camera(
        prim_path="/World/OverheadCamera",
        position=np.array([0.5, 0.0, 1.0]),
        orientation=np.array([0.0, 0.7071, 0.0, 0.7071]),
        resolution=(640, 480),
    )
    camera.initialize()

    controller = PickPlaceController(
        name="pick_place_controller",
        gripper=franka.gripper,
        robot_articulation=franka,
    )

    print("正在编译底层运动学图 (RMPflow)...")
    await world.reset_async()
    world.play()

    print("正在初始化夹爪并等待物体稳定掉落...")
    franka.gripper.open()

    for _ in range(120):
        await omni.kit.app.get_app().next_update_async()

    pos_A = np.array([0.3, 0.3, 0.05])
    pos_B = np.array([0.3, -0.3, 0.05])
    current_place_pos = pos_B

    grasp_state = {"phase": "waiting", "timer": 0, "fixed_pick_pos": None}

    def on_physics_step(step_size):
        nonlocal current_place_pos
        if not world.is_playing():
            return

        if grasp_state["phase"] == "waiting":
            grasp_state["timer"] += 1
            if grasp_state["timer"] > 30:
                obj_pos, _ = target_obj.get_world_pose()
                grasp_state["fixed_pick_pos"] = obj_pos.copy()
                grasp_state["phase"] = "executing"
                print(f"物体位置已锁定: {obj_pos}")
            return

        if grasp_state["phase"] == "executing":
            actions = controller.forward(
                picking_position=grasp_state["fixed_pick_pos"] + np.array([0.0, 0.0, 0.015]),
                placing_position=current_place_pos,
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0, 0, 0.02]),
            )
            franka.apply_action(actions)

            if controller.is_done():
                print("抓取放置完成，准备下一轮")
                if np.allclose(current_place_pos, pos_B):
                    current_place_pos = pos_A
                else:
                    current_place_pos = pos_B
                controller.reset()
                grasp_state["phase"] = "waiting"
                grasp_state["timer"] = 0
                grasp_state["fixed_pick_pos"] = None

    world.add_physics_callback("robot_pick_place", on_physics_step)


asyncio.ensure_future(setup_and_run())
