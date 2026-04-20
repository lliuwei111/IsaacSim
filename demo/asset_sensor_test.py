import os
import math
import asyncio
import numpy as np
import cv2  # 🌟 新增：用于视频编码
from isaacsim import SimulationApp

# 1. 启动仿真器并开启 WebRTC 推流
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")
enable_extension("omni.replicator.core")
enable_extension("isaacsim.sensors.rtx")

for _ in range(10):
    simulation_app.update()

import omni.replicator.core as rep
import omni.kit.commands
import omni.kit.asset_converter
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
import omni.usd

# ================= 配置路径 =================
GLB_FILE_PATH = "/workspace/my_code/IsaacSim/sample_tremos.glb"
USD_OUTPUT_PATH = "/workspace/my_code/IsaacSim/sample_tremos.usd"
DATASET_OUTPUT_DIR = "/workspace/my_code/IsaacSim/sensor_output_video" 
os.makedirs(DATASET_OUTPUT_DIR, exist_ok=True) # 确保文件夹存在
# ============================================

# 🌟 新增：极简的 PLY 文件写入函数
def save_ply(points, filename):
    if points is None or len(points) == 0:
        return
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        # 提取前三列 (X, Y, Z) 并格式化写入
        np.savetxt(f, points[:, :3], fmt='%f %f %f')

viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

# 2. 资产转换 
async def convert_glb_to_usd(input_glb, output_usd):
    print(f"🔄 正在转换 {input_glb}...")
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = task_manager.create_converter_task(input_glb, output_usd, None, context)
    await task.wait_until_finished()

if not os.path.exists(USD_OUTPUT_PATH):
    asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(GLB_FILE_PATH, USD_OUTPUT_PATH))

from omni.isaac.core.utils.nucleus import get_assets_root_path

world = World(stage_units_in_meters=1.0)

# 3. 加载室内房间
print("🌐 加载 Simple_Room...")
assets_root_path = get_assets_root_path()
room_asset_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
add_reference_to_stage(usd_path=room_asset_path, prim_path="/World/Room")

# 4. 加载物体与物理属性 (保留了你优秀的掉落逻辑！)
asset_prim_path = "/World/ScanTarget"
add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=asset_prim_path)
stage = omni.usd.get_context().get_stage()

visual_prim = UsdGeom.Xform.Get(simulation_app.context.get_stage(), asset_prim_path)
xform_api = UsdGeom.XformCommonAPI(visual_prim)
xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0)) 
xform_api.SetTranslate(Gf.Vec3d(0, 0, 2.0)) # 悬空 2 米
xform_api.SetRotate((90, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)

cup_prim = stage.GetPrimAtPath(asset_prim_path)
UsdPhysics.CollisionAPI.Apply(cup_prim)
mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(cup_prim)
mesh_collision_api.CreateApproximationAttr().Set("convexHull")
UsdPhysics.RigidBodyAPI.Apply(cup_prim)
mass_api = UsdPhysics.MassAPI.Apply(cup_prim)
mass_api.CreateMassAttr(0.5)

# 5. 架设传感器
robot_head_path = "/World/RobotHead"
robot_head = UsdGeom.Xform.Define(simulation_app.context.get_stage(), robot_head_path)

camera_path = robot_head_path + "/Camera"
lidar_path = robot_head_path + "/Lidar"

cam = UsdGeom.Camera.Define(simulation_app.context.get_stage(), camera_path)
cam.GetFocalLengthAttr().Set(24.0)

omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    parent=None,
    config="Example_Rotary",
    translation=(0, 0, 0.1), 
    orientation=Gf.Quatd(1, 0, 0, 0),
)

viewport_api.set_active_camera(camera_path)

# 6. 🌟🌟🌟 抛弃 BasicWriter，改用底层数据截获器 (Annotators) 🌟🌟🌟
cam_rp = rep.create.render_product(camera_path, (1024, 768))
lidar_rp = rep.create.render_product(lidar_path, [1, 1])

# 挂载 RGB 抓取器
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(cam_rp)

# 挂载激光雷达点云抓取器
pc_annotator = rep.AnnotatorRegistry.get_annotator("pointcloud")
pc_annotator.attach(lidar_rp)

# 🌟 初始化 OpenCV 视频编码器
video_out_path = os.path.join(DATASET_OUTPUT_DIR, "orbit_scan.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 使用 mp4 编码
video_writer = cv2.VideoWriter(video_out_path, fourcc, 30.0, (1024, 768))

world.reset()
world.play() 

print("✅ 录像机已开启，开始 360 度动态拍摄...")

# 7. 扫描主循环
frame = 0
radius = 4.0 
height = 0.95

# 录制 300 帧 (约 10 秒)，防止视频录得太大死循环
while simulation_app.is_running() and frame < 300:
    world.step(render=True)
    
    # --- 相机飞行控制 ---
    angle = frame * 0.01
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    
    eye = Gf.Vec3d(x, y, height)
    target = Gf.Vec3d(0, 0, 0.75) 
    up = Gf.Vec3d(0, 0, 1)
    
    view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
    transform_matrix = view_matrix.GetInverse()
    quat = transform_matrix.ExtractRotation().GetQuat()
    quat_np = np.array([quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]])
    eye_np = np.array([eye[0], eye[1], eye[2]])
    
    robot_head_controller = XFormPrim(prim_path=robot_head_path)
    robot_head_controller.set_world_pose(position=eye_np, orientation=quat_np)
    
    # 强制让 Replicator 渲染这一帧
    rep.orchestrator.step()
    
    # 🌟🌟🌟 截获数据并写入 🌟🌟🌟
    
    # 1. 录制视频流
    rgb_data = rgb_annotator.get_data()
    if rgb_data is not None and rgb_data.size > 0:
        # ⚠️ 注意：Replicator 吐出的是 RGBA，OpenCV 需要的是 BGR，必须转换颜色通道！
        bgr_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2BGR)
        video_writer.write(bgr_image)
        
    # 2. 保存 PLY 点云 (每 30 帧保存一次，防止硬盘爆炸)
    if frame % 30 == 0:
        pc_data = pc_annotator.get_data()
        if pc_data is not None and 'data' in pc_data:
            points = pc_data['data']
            if len(points) > 0:
                ply_filename = os.path.join(DATASET_OUTPUT_DIR, f"lidar_scan_{frame:04d}.ply")
                save_ply(points, ply_filename)
                print(f"   💾 保存点云: {ply_filename} (点数: {len(points)})")

    # 进度提示
    if frame % 30 == 0:
        print(f"🎬 视频已录制 {frame//30} 秒...")
        
    frame += 1

# 🌟 循环结束后，一定要释放录像机，否则视频文件会损坏！
video_writer.release()
print(f"🎉 任务完成！视频已保存至: {video_out_path}")

world.stop()
simulation_app.close()