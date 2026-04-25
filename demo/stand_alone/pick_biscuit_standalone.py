import os
import asyncio
import struct
import json
import numpy as np
import cv2
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
for _ in range(10):
    simulation_app.update()

import omni.replicator.core as rep
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
from pxr import UsdGeom, Gf

# ================= 路径配置 =================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GLB_FILE_PATH = "/workspace/IsaacSim/biscuit.glb"
USD_OUTPUT_PATH = "/workspace/IsaacSim/biscuit.usd"
VIDEO_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "pick_biscuit.mp4")
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_FPS = 30.0
MAX_FRAMES = 1500
# ============================================

viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

viewport_api.set_texture_resolution((VIDEO_WIDTH, VIDEO_HEIGHT))

camera_path = "/OmniverseKit_Persp"
cam_rp = rep.create.render_product(camera_path, (VIDEO_WIDTH, VIDEO_HEIGHT))
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(cam_rp)

# 3. 异步函数：将 GLB 转换为底层的 USD 格式
async def convert_glb_to_usd(input_glb, output_usd):
    print(f"正在将 {input_glb} 转换为 USD...")
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = task_manager.create_converter_task(input_glb, output_usd, None, context)
    success = await task.wait_until_finished()
    if success:
        print(f"转换成功！保存在: {output_usd}")
    return success

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

if not os.path.exists(USD_OUTPUT_PATH):
    asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(GLB_FILE_PATH, USD_OUTPUT_PATH))
    extract_textures_from_glb(GLB_FILE_PATH, USD_OUTPUT_PATH)

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
world.play()
print("正在初始化夹爪，强制张开...")
franka.gripper.open()

for _ in range(120):
    world.step(render=True)

pos_A = np.array([0.3, 0.3, 0.05])
pos_B = np.array([0.3, -0.3, 0.05])
current_place_pos = pos_B

grasp_state = {"phase": "waiting", "timer": 0, "fixed_pick_pos": None}

def on_physics_step(step_size):
    global current_place_pos
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

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video_writer = cv2.VideoWriter(VIDEO_OUTPUT_PATH, fourcc, VIDEO_FPS, (VIDEO_WIDTH, VIDEO_HEIGHT))
if not video_writer.isOpened():
    print(f"Error: cv2.VideoWriter failed to open. Video will not be saved.")
    video_writer = None

print(f"Recording video to: {VIDEO_OUTPUT_PATH}")
print(f"Max frames: {MAX_FRAMES}, Resolution: {VIDEO_WIDTH}x{VIDEO_HEIGHT}, FPS: {VIDEO_FPS}")

frame_count = 0

while simulation_app.is_running() and frame_count < MAX_FRAMES:
    world.step(render=True)

    if video_writer is not None:
        rep.orchestrator.step()
        rgb_data = rgb_annotator.get_data()
        if rgb_data is not None and rgb_data.size > 0:
            bgr_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2BGR)
            video_writer.write(bgr_image)
            frame_count += 1
            if frame_count % 100 == 0:
                print(f"Recorded {frame_count}/{MAX_FRAMES} frames...")

if video_writer is not None:
    video_writer.release()
    print(f"Video saved to: {VIDEO_OUTPUT_PATH} ({frame_count} frames)")

world.stop()
simulation_app.close()