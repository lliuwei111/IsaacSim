import os
import asyncio
import struct
import json
import re
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

DESK_GLB_PATH = "/workspace/IsaacSim/desk.glb"
DESK_USD_PATH = "/workspace/IsaacSim/desk.usd"
CUP_GLB_PATH = "/workspace/IsaacSim/shuibei.glb"
CUP_USD_PATH = "/workspace/IsaacSim/shuibei.usd"

DESK_POS = np.array([0.0, 0.7, 0.294])
DESK_SURFACE_Z = 1.5
DESK_COLLISION_WIDTH = 1.0
DESK_COLLISION_DEPTH = 0.54

CUP_SCALE = 0.1
CUP_HALF_HEIGHT = CUP_SCALE * 0.355
CUP_MASS = 0.1

ROOM_SIZE = 5.0
WALL_HEIGHT = 3.0

FRANKA_POS = np.array([0.0, -0.3, 0.0])


def extract_textures_from_glb(glb_path, usd_path):
    usd_dir = os.path.dirname(os.path.abspath(usd_path))
    textures_dir = os.path.join(usd_dir, "textures")
    try:
        with open(glb_path, "rb") as f:
            magic, version, length = struct.unpack("<III", f.read(12))
            json_len, json_type = struct.unpack("<II", f.read(8))
            json_data = json.loads(f.read(json_len))
            bin_len, bin_type = struct.unpack("<II", f.read(8))
            bin_data = f.read(bin_len)

        if "images" not in json_data:
            print("GLB中没有嵌入纹理图片")
            return

        referenced_textures = set()
        with open(usd_path, "rb") as f:
            usd_data = f.read()
        for m in re.finditer(rb'([\w/_.\-]+\.(?:png|jpg|jpeg|hdr|exr))', usd_data):
            ref_path = m.group(1).decode("utf-8", errors="ignore")
            if len(ref_path) < 200:
                referenced_textures.add(ref_path)
        print(f"USD引用的纹理文件: {referenced_textures}")

        os.makedirs(textures_dir, exist_ok=True)
        glb_images = []
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
            glb_images.append({"data": img_data, "ext": ext, "index": i})

        for ref in referenced_textures:
            ref_full = os.path.join(usd_dir, ref) if not os.path.isabs(ref) else ref
            if os.path.exists(ref_full):
                print(f"纹理已存在: {ref_full}")
                continue
            for img in glb_images:
                out_path = os.path.join(usd_dir, ref)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                with open(out_path, "wb") as out:
                    out.write(img["data"])
                print(f"提取纹理: {out_path} ({len(img['data'])} bytes)")
                break

        if not referenced_textures:
            for img in glb_images:
                out_name = f"texture{img['index']}{img['ext']}"
                out_path = os.path.join(textures_dir, out_name)
                with open(out_path, "wb") as out:
                    out.write(img["data"])
                print(f"提取纹理(默认命名): {out_path} ({len(img['data'])} bytes)")
    except Exception as e:
        print(f"提取纹理失败: {e}")


async def convert_glb_to_usd(glb_path, usd_path):
    if os.path.exists(usd_path):
        print(f"USD已存在: {usd_path}")
        return True
    print(f"正在将 {glb_path} 转换为 USD...")
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = task_manager.create_converter_task(glb_path, usd_path, None, context)
    success = await task.wait_until_finished()
    if not success:
        print(f"转换失败: {glb_path}")
        return False
    print(f"转换成功: {usd_path}")
    extract_textures_from_glb(glb_path, usd_path)
    return True


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

    if not await convert_glb_to_usd(DESK_GLB_PATH, DESK_USD_PATH):
        return
    if not await convert_glb_to_usd(CUP_GLB_PATH, CUP_USD_PATH):
        return

    world = World(stage_units_in_meters=1.0)

    if world.get_physics_context() is None:
        try:
            from isaacsim.core.api.physics_context import PhysicsContext
        except ImportError:
            from omni.isaac.core.physics_context import PhysicsContext
        world._physics_context = PhysicsContext()

    world.scene.add(
        FixedCuboid(
            prim_path="/World/Floor",
            name="floor",
            position=np.array([0.0, 0.0, -0.05]),
            scale=np.array([ROOM_SIZE, ROOM_SIZE, 0.1]),
            color=np.array([0.15, 0.35, 0.55])
        )
    )

    wall_thickness = 0.1
    world.scene.add(
        FixedCuboid(
            prim_path="/World/BackWall",
            name="back_wall",
            position=np.array([ROOM_SIZE / 2, 0.0, WALL_HEIGHT / 2]),
            scale=np.array([wall_thickness, ROOM_SIZE, WALL_HEIGHT]),
            color=np.array([0.75, 0.25, 0.20])
        )
    )
    world.scene.add(
        FixedCuboid(
            prim_path="/World/LeftWall",
            name="left_wall",
            position=np.array([0.0, ROOM_SIZE / 2, WALL_HEIGHT / 2]),
            scale=np.array([ROOM_SIZE, wall_thickness, WALL_HEIGHT]),
            color=np.array([0.75, 0.25, 0.20])
        )
    )
    world.scene.add(
        FixedCuboid(
            prim_path="/World/RightWall",
            name="right_wall",
            position=np.array([0.0, -ROOM_SIZE / 2, WALL_HEIGHT / 2]),
            scale=np.array([ROOM_SIZE, wall_thickness, WALL_HEIGHT]),
            color=np.array([0.75, 0.25, 0.20])
        )
    )

    create_prim(
        prim_path="/World/defaultLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 3000.0}
    )
    create_prim(
        prim_path="/World/DeskLight",
        prim_type="DistantLight",
        attributes={
            "inputs:intensity": 2000.0,
            "inputs:angle": 30.0,
        },
        position=np.array([1.0, 0.0, 2.5]),
    )
    create_prim(
        prim_path="/World/FillLight",
        prim_type="SphereLight",
        attributes={
            "inputs:intensity": 50000.0,
            "inputs:radius": 0.3,
        },
        position=np.array([0.5, 0.0, 2.0]),
    )

    set_camera_view(eye=np.array([1.2, 0.5, 1.0]), target=np.array([0.0, 0.5, 0.4]))

    print("正在加载桌子模型...")
    stage = omni.usd.get_context().get_stage()

    desk_prim_path = "/World/Desk"
    add_reference_to_stage(usd_path=DESK_USD_PATH, prim_path=desk_prim_path)
    desk_prim = UsdGeom.Xform.Get(stage, desk_prim_path)
    xform_api = UsdGeom.XformCommonAPI(desk_prim)
    xform_api.SetTranslate(Gf.Vec3d(DESK_POS[0], DESK_POS[1], DESK_POS[2]))

    print("正在创建桌子碰撞体...")
    desk_collision_path = "/World/DeskCollision"
    world.scene.add(
        FixedCuboid(
            prim_path=desk_collision_path,
            name="desk_collision",
            position=np.array([DESK_POS[0], DESK_POS[1], DESK_SURFACE_Z / 2]),
            scale=np.array([DESK_COLLISION_WIDTH, DESK_COLLISION_DEPTH, DESK_SURFACE_Z]),
            color=np.array([0.0, 0.0, 0.0])
        )
    )
    UsdGeom.Imageable(stage.GetPrimAtPath(desk_collision_path)).MakeInvisible()

    print("正在加载水杯模型...")
    cup_proxy_path = "/World/Cup_Proxy"
    UsdGeom.Xform.Define(stage, cup_proxy_path)

    cup_initial_pos = np.array([
        0,0,1
        # DESK_POS[0] - 0.1,
        # DESK_POS[1] + 0.15,
        # DESK_SURFACE_Z + CUP_HALF_HEIGHT + 0.1
    ])

    target_obj = world.scene.add(
        RigidPrim(
            prim_path=cup_proxy_path,
            name="target_obj",
            position=cup_initial_pos,
            scale=np.array([CUP_SCALE, CUP_SCALE, CUP_SCALE]),
            mass=CUP_MASS
        )
    )

    cup_visual_path = cup_proxy_path + "/VisualModel"
    add_reference_to_stage(usd_path=CUP_USD_PATH, prim_path=cup_visual_path)

    cup_visual_prim = UsdGeom.Xform.Get(stage, cup_visual_path)
    xform_api = UsdGeom.XformCommonAPI(cup_visual_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))

    final_cup_prim = stage.GetPrimAtPath(cup_visual_path)
    setCollider(final_cup_prim, approximationShape="convexHull")

    high_friction_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/HighFriction",
        dynamic_friction=3.0,
        static_friction=3.0,
        restitution=0.0
    )
    physicsUtils.add_physics_material_to_prim(stage, final_cup_prim, high_friction_material.prim_path)

    print("正在加载机械臂...")
    franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka_robot", position=FRANKA_POS))

    print("正在安装视觉传感器...")
    camera = Camera(
        prim_path="/World/OverheadCamera",
        position=np.array([0.7, 0.0, 1.5]),
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

    print("正在初始化夹爪并等待物体稳定...")
    franka.gripper.open()

    for _ in range(150):
        await omni.kit.app.get_app().next_update_async()

    pos_A = np.array([DESK_POS[0] - 0.1, DESK_POS[1] + 0.15, DESK_SURFACE_Z + CUP_HALF_HEIGHT + 0.02])
    pos_B = np.array([DESK_POS[0] - 0.1, DESK_POS[1] - 0.15, DESK_SURFACE_Z + CUP_HALF_HEIGHT + 0.02])
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
