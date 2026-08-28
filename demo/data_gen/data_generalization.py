import os
import sys
import math
import asyncio
import json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run.log")
_log_fh = open(LOG_FILE, "w", buffering=1)

class Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            s.write(data)
            s.flush()
    def flush(self):
        for s in self.streams:
            s.flush()

sys.stdout = Tee(sys.__stdout__, _log_fh)
sys.stderr = Tee(sys.__stderr__, _log_fh)

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
enable_extension("isaacsim.sensors.rtx")
for _ in range(10):
    simulation_app.update()

import omni.replicator.core as rep
import omni.kit.commands
from omni.isaac.core import World
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Gf, UsdPhysics
import omni.usd

from gen_config import (
    DATASET_OUTPUT_DIR, USD_ASSETS_DIR, INPUT_GLB_PATH,
    NUM_EPOCHS, FRAMES_PER_EPOCH, POINTCLOUD_SAMPLE_INTERVAL,
    IMAGE_SAVE_INTERVAL, IMAGE_WIDTH, IMAGE_HEIGHT, VIDEO_FPS,
    PHYSICS_SETTLE_FRAMES,
    ENABLE_DEPTH_VIS, ENABLE_POINTCLOUD_VIS, DEPTH_VIS_SAMPLE_COUNT,
    get_random_object_config, get_random_camera_config,
    get_random_light_config, get_random_environment,
)
from gen_utils import save_ply, ensure_usd_asset, convert_glb_if_needed, visualize_depth, visualize_pointcloud

viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

usd_path = ensure_usd_asset(INPUT_GLB_PATH, USD_ASSETS_DIR)
if usd_path is None:
    print("[错误] 输入 GLB 文件不存在，请检查路径！")
    simulation_app.close()
    exit(1)

convert_glb_if_needed(INPUT_GLB_PATH, usd_path)

if not os.path.exists(usd_path):
    print(f"[错误] USD 资产不存在: {usd_path}")
    simulation_app.close()
    exit(1)

asset_name = os.path.splitext(os.path.basename(INPUT_GLB_PATH))[0]
print(f"输入资产: {asset_name} ({INPUT_GLB_PATH})")
print(f"USD 路径: {usd_path}")

os.makedirs(DATASET_OUTPUT_DIR, exist_ok=True)

from omni.isaac.core.utils.nucleus import get_assets_root_path
assets_root_path = get_assets_root_path()


def clear_stage_objects():
    stage = omni.usd.get_context().get_stage()
    paths_to_remove = [
        "/World/Room", "/World/ScanTarget", "/World/RobotHead",
        "/World/Ground", "/World/defaultLight", "/World/FillLight",
        "/World/RobotHead/Camera",
    ]
    for path in paths_to_remove:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)


def setup_environment(env_rel_path):
    env_usd = assets_root_path + env_rel_path
    add_reference_to_stage(usd_path=env_usd, prim_path="/World/Room")


def setup_ground_and_lights(light_config):
    stage = omni.usd.get_context().get_stage()
    ground_prim = stage.GetPrimAtPath("/World/Ground")
    if not ground_prim.IsValid():
        ground_path = "/World/Ground"
        UsdGeom.Xform.Define(stage, ground_path)
        cube_path = ground_path + "/cube"
        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.GetSizeAttr().Set(1.0)
        xform_api = UsdGeom.XformCommonAPI(cube)
        xform_api.SetScale(Gf.Vec3f(10.0, 10.0, 0.1))
        xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(cube_path))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(cube_path))
    create_prim(
        prim_path="/World/defaultLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": light_config["dome_intensity"]},
    )
    create_prim(
        prim_path="/World/FillLight",
        prim_type="DistantLight",
        attributes={
            "inputs:intensity": light_config["fill_intensity"],
            "inputs:angle": light_config["fill_angle"],
        },
        position=np.array([2.0, 2.0, 3.0]),
    )


def setup_object(usd_path, obj_config):
    asset_prim_path = "/World/ScanTarget"
    add_reference_to_stage(usd_path=usd_path, prim_path=asset_prim_path)
    stage = omni.usd.get_context().get_stage()

    visual_prim = UsdGeom.Xform.Get(stage, asset_prim_path)
    xform_api = UsdGeom.XformCommonAPI(visual_prim)
    s = obj_config["scale"]
    xform_api.SetScale(Gf.Vec3f(s, s, s))
    dx, dy, dz = obj_config["drop_position"]
    xform_api.SetTranslate(Gf.Vec3d(dx, dy, dz))
    rx, ry, rz = obj_config["rotation"]
    xform_api.SetRotate((rx, ry, rz), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    prim = stage.GetPrimAtPath(asset_prim_path)
    UsdPhysics.CollisionAPI.Apply(prim)
    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mesh_collision_api.CreateApproximationAttr().Set("convexHull")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(obj_config["mass"])


def setup_sensors():
    stage = omni.usd.get_context().get_stage()
    robot_head_path = "/World/RobotHead"
    robot_head = stage.GetPrimAtPath(robot_head_path)
    if not robot_head.IsValid():
        UsdGeom.Xform.Define(stage, robot_head_path)

    camera_path = robot_head_path + "/Camera"

    cam_prim = stage.GetPrimAtPath(camera_path)
    if not cam_prim.IsValid():
        cam = UsdGeom.Camera.Define(stage, camera_path)
        cam.GetFocalLengthAttr().Set(24.0)

    viewport_api.set_active_camera(camera_path)
    return camera_path, robot_head_path


def setup_annotators(camera_path):
    cam_rp = rep.create.render_product(camera_path, (IMAGE_WIDTH, IMAGE_HEIGHT))

    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(cam_rp)

    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annotator.attach(cam_rp)

    return cam_rp, rgb_annotator, depth_annotator


def depth_to_pointcloud(depth_map, fov_horizontal=60.0):
    if depth_map is None or depth_map.size == 0:
        return None
    if depth_map.ndim == 3:
        depth_map = depth_map[:, :, 0]
    h, w = depth_map.shape
    fx = (w / 2.0) / math.tan(math.radians(fov_horizontal / 2.0))
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    u = np.arange(w)
    v = np.arange(h)
    uu, vv = np.meshgrid(u, v)
    z = depth_map.astype(np.float64)
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy
    valid = z > 0
    points = np.stack([x[valid], y[valid], z[valid]], axis=-1)
    if len(points) == 0:
        return None
    return points


def compute_camera_pose(frame, cam_config):
    angle = frame * cam_config["angular_speed"]
    r = cam_config["radius"]
    h = cam_config["height"]
    x = r * math.cos(angle)
    y = r * math.sin(angle)

    eye = Gf.Vec3d(x, y, h)
    target = Gf.Vec3d(0, 0, cam_config["target_height"])
    up = Gf.Vec3d(0, 0, 1)

    view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
    transform_matrix = view_matrix.GetInverse()
    quat = transform_matrix.ExtractRotation().GetQuat()
    quat_np = np.array([quat.GetReal(), quat.GetImaginary()[0],
                        quat.GetImaginary()[1], quat.GetImaginary()[2]])
    eye_np = np.array([eye[0], eye[1], eye[2]])
    return eye_np, quat_np


def save_epoch_metadata(epoch_dir, epoch_idx, asset_name, obj_config, cam_config, light_config, env_path):
    metadata = {
        "epoch": epoch_idx,
        "asset": asset_name,
        "object": {
            "scale": obj_config["scale"],
            "rotation": list(obj_config["rotation"]),
            "drop_position": list(obj_config["drop_position"]),
            "mass": obj_config["mass"],
        },
        "camera": {
            "radius": cam_config["radius"],
            "height": cam_config["height"],
            "angular_speed": cam_config["angular_speed"],
            "target_height": cam_config["target_height"],
        },
        "light": {
            "dome_intensity": light_config["dome_intensity"],
            "fill_intensity": light_config["fill_intensity"],
            "fill_angle": light_config["fill_angle"],
        },
        "environment": env_path,
    }
    meta_path = os.path.join(epoch_dir, "metadata.json")
    with open(meta_path, 'w') as f:
        json.dump(metadata, f, indent=2)


print("=" * 60)
print("数据泛化任务启动")
print(f"  输入资产: {asset_name}")
print(f"  总轮次: {NUM_EPOCHS}")
print(f"  每轮帧数: {FRAMES_PER_EPOCH}")
print(f"  输出目录: {DATASET_OUTPUT_DIR}")
print(f"  深度可视化: {'开启' if ENABLE_DEPTH_VIS else '关闭'}")
print(f"  点云可视化: {'开启' if ENABLE_POINTCLOUD_VIS else '关闭'}")
print("=" * 60)

world = World(stage_units_in_meters=1.0)

for epoch in range(NUM_EPOCHS):
    print(f"\n{'='*60}")
    print(f"Epoch {epoch + 1}/{NUM_EPOCHS}")
    print(f"{'='*60}")

    obj_config = get_random_object_config()
    cam_config = get_random_camera_config()
    light_config = get_random_light_config()
    env_rel_path = get_random_environment()

    print(f"  资产: {asset_name}")
    print(f"  环境: {env_rel_path}")
    print(f"  物体: scale={obj_config['scale']:.2f}, "
          f"rot=({obj_config['rotation'][0]:.0f},{obj_config['rotation'][1]:.0f},{obj_config['rotation'][2]:.0f}), "
          f"mass={obj_config['mass']:.2f}kg")
    print(f"  相机: radius={cam_config['radius']:.2f}m, "
          f"height={cam_config['height']:.2f}m, speed={cam_config['angular_speed']:.4f}")
    print(f"  灯光: dome={light_config['dome_intensity']:.0f}, fill={light_config['fill_intensity']:.0f}")

    epoch_dir = os.path.join(DATASET_OUTPUT_DIR, f"epoch_{epoch:04d}")
    rgb_dir = os.path.join(epoch_dir, "rgb")
    depth_dir = os.path.join(epoch_dir, "depth")
    ply_dir = os.path.join(epoch_dir, "pointcloud")
    for d in [rgb_dir, depth_dir, ply_dir]:
        os.makedirs(d, exist_ok=True)

    save_epoch_metadata(epoch_dir, epoch, asset_name, obj_config, cam_config, light_config, env_rel_path)

    clear_stage_objects()

    setup_environment(env_rel_path)
    setup_ground_and_lights(light_config)
    setup_object(usd_path, obj_config)
    camera_path, robot_head_path = setup_sensors()
    cam_rp, rgb_annotator, depth_annotator = setup_annotators(camera_path)

    world.reset()
    world.play()

    print(f"  物理稳定中 ({PHYSICS_SETTLE_FRAMES} 帧)...")
    for _ in range(PHYSICS_SETTLE_FRAMES):
        world.step(render=True)

    video_out_path = os.path.join(epoch_dir, "orbit_scan.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_out_path, fourcc, VIDEO_FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))

    robot_head_controller = XFormPrim(prim_path=robot_head_path)

    print(f"  开始采集 {FRAMES_PER_EPOCH} 帧...")
    for frame in range(FRAMES_PER_EPOCH):
        world.step(render=True)

        eye_np, quat_np = compute_camera_pose(frame, cam_config)
        robot_head_controller.set_world_pose(position=eye_np, orientation=quat_np)

        rep.orchestrator.step()

        rgb_data = rgb_annotator.get_data()
        if rgb_data is not None and rgb_data.size > 0:
            bgr_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2BGR)
            video_writer.write(bgr_image)

            if frame % IMAGE_SAVE_INTERVAL == 0:
                rgb_np = rgb_data[:, :, :3].copy()
                rgb_filename = os.path.join(rgb_dir, f"rgb_{frame:04d}.png")
                cv2.imwrite(rgb_filename, cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR))

        if frame % IMAGE_SAVE_INTERVAL == 0:
            dist_data = depth_annotator.get_data()
            if dist_data is not None and dist_data.size > 0:
                depth_np = dist_data.copy()
                if depth_np.ndim == 3:
                    depth_np = depth_np[:, :, 0]
                depth_filename = os.path.join(depth_dir, f"depth_{frame:04d}.npy")
                np.save(depth_filename, depth_np)

        if frame % POINTCLOUD_SAMPLE_INTERVAL == 0:
            dist_data = depth_annotator.get_data()
            if dist_data is not None and dist_data.size > 0:
                depth_np = dist_data.copy()
                if depth_np.ndim == 3:
                    depth_np = depth_np[:, :, 0]
                xyz = depth_to_pointcloud(depth_np)
                if xyz is not None and len(xyz) > 0:
                    ply_filename = os.path.join(ply_dir, f"pointcloud_{frame:04d}.ply")
                    save_ply(xyz, ply_filename)
                    print(f"    点云保存: {ply_filename} (点数: {len(xyz)})")
                else:
                    print(f"    帧 {frame}: 深度图转点云为空")

        if frame % 30 == 0:
            print(f"    帧 {frame}/{FRAMES_PER_EPOCH}")

    video_writer.release()

    rgb_annotator.detach()
    depth_annotator.detach()
    cam_rp.destroy()

    world.stop()

    if ENABLE_DEPTH_VIS:
        visualize_depth(epoch_dir)

    if ENABLE_POINTCLOUD_VIS:
        visualize_pointcloud(epoch_dir, sample_count=DEPTH_VIS_SAMPLE_COUNT)

    print(f"  Epoch {epoch + 1} 完成 -> {epoch_dir}")

print(f"\n{'='*60}")
print(f"全部 {NUM_EPOCHS} 轮数据泛化完成！")
print(f"输出目录: {DATASET_OUTPUT_DIR}")
print(f"{'='*60}")

simulation_app.close()
