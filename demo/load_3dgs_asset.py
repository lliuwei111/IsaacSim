import os
import math
import asyncio
import numpy as np
from isaacsim import SimulationApp

# 1. 启动仿真器并开启 WebRTC 推流
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.livestream.webrtc")
enable_extension("omni.replicator.core")
enable_extension("isaacsim.sensors.rtx")

for _ in range(10):
    simulation_app.update()

import carb
import omni.replicator.core as rep
import omni.kit.commands
import omni.kit.asset_converter
from omni.isaac.core import World
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Gf

try:
    import cv2
except ImportError:
    cv2 = None

# Replicator 在手动 step 时建议关闭「播放即采」
carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

# ================= 配置路径（相对本脚本所在仓库根目录）=================
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_ISAAC_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
GLB_FILE_PATH = os.path.join(_ISAAC_ROOT, "sample_tremos.glb")
USD_OUTPUT_PATH = os.path.join(_ISAAC_ROOT, "sample_tremos.usd")
DATASET_OUTPUT_DIR = os.path.join(_ISAAC_ROOT, "sensor_output_ground")
# 输出：MP4 与 ply 子目录
VIDEO_FILENAME = "capture_rgb.mp4"
PLY_SUBDIR = "ply_pointclouds"
# 相机分辨率（与 render_product 一致）
CAM_WIDTH, CAM_HEIGHT = 1024, 768
# 仿真帧间隔多少步触发一次采集（与原先一致）
CAPTURE_EVERY_SIM_FRAMES = 30
# 共采集多少帧写入视频/ply（避免无限循环无法 finalize 视频）
NUM_CAPTURES = 120
# 视频播放帧率（仅影响播放速度，与仿真步频无关）
VIDEO_FPS = 10.0
# ============================================

os.makedirs(DATASET_OUTPUT_DIR, exist_ok=True)
ply_dir = os.path.join(DATASET_OUTPUT_DIR, PLY_SUBDIR)
os.makedirs(ply_dir, exist_ok=True)
video_path = os.path.join(DATASET_OUTPUT_DIR, VIDEO_FILENAME)


def extract_xyz_from_pointcloud(pc):
    """从 Replicator pointcloud annotator 结果中取出 Nx3 世界坐标。"""
    if pc is None:
        return None
    if isinstance(pc, dict):
        for key in ("data", "points", "pointcloud", "xyz"):
            if key in pc:
                pc = pc[key]
                break
        else:
            return None
    arr = np.asarray(pc)
    if arr.size == 0:
        return None
    if arr.dtype.names:
        names = arr.dtype.names
        if all(n in names for n in ("x", "y", "z")):
            v = np.empty((arr.shape[0], 3), dtype=np.float64)
            v[:, 0] = arr["x"].astype(np.float64).ravel()
            v[:, 1] = arr["y"].astype(np.float64).ravel()
            v[:, 2] = arr["z"].astype(np.float64).ravel()
            return v
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim == 2 and arr.shape[1] >= 3:
        return arr[:, :3]
    if arr.ndim == 3:
        flat = arr.reshape(-1, arr.shape[-1])
        if flat.shape[1] >= 3:
            return flat[:, :3]
    return None


def write_ply_ascii(path, xyz: np.ndarray):
    """写入 ASCII PLY（仅 xyz）。"""
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    n = xyz.shape[0]
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for i in range(n):
            f.write(f"{xyz[i, 0]} {xyz[i, 1]} {xyz[i, 2]}\n")


def write_ply_binary_le(path, xyz: np.ndarray):
    """写入 binary_little_endian PLY，点数大时比 ASCII 小且快。"""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(xyz.tobytes())


viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()


async def convert_glb_to_usd(input_glb, output_usd):
    print(f"Converting {input_glb}...")
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = task_manager.create_converter_task(input_glb, output_usd, None, context)
    await task.wait_until_finished()


if not os.path.exists(USD_OUTPUT_PATH):
    asyncio.get_event_loop().run_until_complete(convert_glb_to_usd(GLB_FILE_PATH, USD_OUTPUT_PATH))

from omni.isaac.core.utils.nucleus import get_assets_root_path

world = World(stage_units_in_meters=1.0)

print("Connecting to Nucleus...")
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Cannot resolve Nucleus assets root. Check Omniverse login / network.")

room_asset_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
print(f"room_asset_path: {room_asset_path}")
add_reference_to_stage(usd_path=room_asset_path, prim_path="/World/Room")

asset_prim_path = "/World/ScanTarget"
add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=asset_prim_path)

visual_prim = UsdGeom.Xform.Get(simulation_app.context.get_stage(), asset_prim_path)
xform_api = UsdGeom.XformCommonAPI(visual_prim)
xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
xform_api.SetTranslate(Gf.Vec3d(0, 0, 0.75))
xform_api.SetRotate((90, 0, 90), UsdGeom.XformCommonAPI.RotationOrderXYZ)

print("Setting up camera + RTX LiDAR...")
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

cam_rp = rep.create.render_product(camera_path, (CAM_WIDTH, CAM_HEIGHT))
lidar_rp = rep.create.render_product(lidar_path, [1, 1])

rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach(cam_rp)
pc_annot = rep.AnnotatorRegistry.get_annotator("pointcloud")
pc_annot.attach(lidar_rp)

world.reset()
rep.orchestrator.preview()

video_writer = None
if cv2 is not None:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(video_path, fourcc, VIDEO_FPS, (CAM_WIDTH, CAM_HEIGHT))
    if not video_writer.isOpened():
        print("Warning: cv2.VideoWriter failed to open; install opencv or check codec. Video will be skipped.")
        video_writer = None
else:
    print("Warning: OpenCV (cv2) not found; MP4 will not be written. Install opencv-python in Isaac env.")

print(f"Output video: {video_path}")
print(f"Output PLY directory: {ply_dir}")
print(f"Captures: {NUM_CAPTURES} frames every {CAPTURE_EVERY_SIM_FRAMES} sim steps.")

frame = 0
radius = 0.3
height = 0.95
capture_index = 0

try:
    while simulation_app.is_running() and capture_index < NUM_CAPTURES:
        world.step(render=True)

        angle = frame * 0.01
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        eye = Gf.Vec3d(x, y, height)
        target = Gf.Vec3d(0, 0, 0.05)
        up = Gf.Vec3d(0, 0, 1)

        view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
        transform_matrix = view_matrix.GetInverse()

        quat = transform_matrix.ExtractRotation().GetQuat()
        quat_np = np.array(
            [quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]]
        )
        eye_np = np.array([eye[0], eye[1], eye[2]])

        robot_head_controller = XFormPrim(prim_path=robot_head_path)
        robot_head_controller.set_world_pose(position=eye_np, orientation=quat_np)

        if frame % CAPTURE_EVERY_SIM_FRAMES == 0:
            print(f"Capture {capture_index + 1}/{NUM_CAPTURES}...")
            rep.orchestrator.step()
            rep.orchestrator.wait_until_complete()

            rgb_data = rgb_annot.get_data()
            if rgb_data is not None and video_writer is not None:
                img = np.asarray(rgb_data)
                if img.ndim == 2:
                    frame_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                else:
                    frame_bgr = cv2.cvtColor(img[..., :3], cv2.COLOR_RGB2BGR)
                if frame_bgr.shape[0] != CAM_HEIGHT or frame_bgr.shape[1] != CAM_WIDTH:
                    frame_bgr = cv2.resize(frame_bgr, (CAM_WIDTH, CAM_HEIGHT))
                video_writer.write(frame_bgr)

            pc_raw = pc_annot.get_data()
            xyz = extract_xyz_from_pointcloud(pc_raw)
            if xyz is not None and len(xyz) > 0:
                ply_path = os.path.join(ply_dir, f"scan_{capture_index:05d}.ply")
                if len(xyz) > 500000:
                    write_ply_binary_le(ply_path, xyz)
                else:
                    write_ply_ascii(ply_path, xyz)
            else:
                print(f"  (pointcloud empty or unsupported layout for frame {capture_index})")

            capture_index += 1

        frame += 1
finally:
    if video_writer is not None:
        video_writer.release()
        print(f"Video saved: {video_path}")

world.stop()
simulation_app.close()
