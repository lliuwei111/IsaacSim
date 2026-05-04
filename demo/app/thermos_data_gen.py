import os
import math
import asyncio
import struct
import json
import numpy as np

import omni.kit.app
import omni.kit.asset_converter
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Gf

GLB_FILE_PATH = "/workspace/IsaacSim/sample_tremos.glb"
USD_OUTPUT_PATH = "/workspace/IsaacSim/sample_tremos.usd"
OUTPUT_DIR = "/workspace/IsaacSim/thermos_dataset"
RGB_DIR = os.path.join(OUTPUT_DIR, "rgb")
DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")

NUM_CAPTURES = 120
IMAGE_WIDTH = 1024
IMAGE_HEIGHT = 768
MIN_RADIUS = 0.3
MAX_RADIUS = 1.0
MIN_HEIGHT = 0.2
MAX_HEIGHT = 0.8
CUP_HEIGHT = 0.2
CUP_Z_OFFSET = CUP_HEIGHT / 2


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

        import re
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
            ref_basename = os.path.basename(ref)
            for img in glb_images:
                out_path = os.path.join(usd_dir, ref)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                with open(out_path, "wb") as out:
                    out.write(img["data"])
                print(f"提取纹理: {out_path} ({len(img['data'])} bytes)")
                break

        if not referenced_textures:
            for img in glb_images:
                out_name = f"thermos_texture{img['index']}{img['ext']}"
                out_path = os.path.join(textures_dir, out_name)
                with open(out_path, "wb") as out:
                    out.write(img["data"])
                print(f"提取纹理(默认命名): {out_path} ({len(img['data'])} bytes)")
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

    os.makedirs(RGB_DIR, exist_ok=True)
    os.makedirs(DEPTH_DIR, exist_ok=True)

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
        attributes={"inputs:intensity": 3000.0}
    )

    create_prim(
        prim_path="/World/FillLight",
        prim_type="DistantLight",
        attributes={
            "inputs:intensity": 1500.0,
            "inputs:angle": 45.0,
        },
        position=np.array([2.0, 2.0, 3.0]),
    )

    set_camera_view(eye=np.array([1.0, 1.0, 0.5]), target=np.array([0.0, 0.0, 0.0]))

    print("正在加载杯子模型...")
    stage = omni.usd.get_context().get_stage()

    thermos_prim_path = "/World/Thermos"
    add_reference_to_stage(usd_path=USD_OUTPUT_PATH, prim_path=thermos_prim_path)

    thermos_prim = UsdGeom.Xform.Get(stage, thermos_prim_path)
    xform_api = UsdGeom.XformCommonAPI(thermos_prim)
    xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, CUP_Z_OFFSET))
    xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera_rig_path = "/World/CameraRig"
    UsdGeom.Xform.Define(stage, camera_rig_path)

    camera_path = camera_rig_path + "/CaptureCamera"
    cam = UsdGeom.Camera.Define(stage, camera_path)
    cam.GetFocalLengthAttr().Set(24.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(15.2908)

    print("正在创建渲染管线...")
    rp = rep.create.render_product(camera_path, (IMAGE_WIDTH, IMAGE_HEIGHT))

    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(rp)

    distance_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    distance_annotator.attach(rp)

    print("正在初始化仿真...")
    await world.reset_async()
    world.play()

    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()

    rep.orchestrator.set_capture_on_play(False)

    camera_rig = XFormPrim(prim_path=camera_rig_path)

    print(f"开始采集 {NUM_CAPTURES} 张图片...")
    print(f"输出目录: {OUTPUT_DIR}")
    print(f"视角范围: 半径 {MIN_RADIUS}~{MAX_RADIUS}m, 高度 {MIN_HEIGHT}~{MAX_HEIGHT}m")

    for i in range(NUM_CAPTURES):
        t = i / NUM_CAPTURES
        angle = t * 2 * math.pi * 3

        radius = MIN_RADIUS + (MAX_RADIUS - MIN_RADIUS) * (0.5 + 0.5 * math.sin(t * 2 * math.pi))
        height = MIN_HEIGHT + (MAX_HEIGHT - MIN_HEIGHT) * (0.5 + 0.5 * math.cos(t * 4 * math.pi))

        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        eye = Gf.Vec3d(x, y, height)
        target = Gf.Vec3d(0.0, 0.0, CUP_Z_OFFSET)
        up = Gf.Vec3d(0.0, 0.0, 1.0)

        view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
        transform_matrix = view_matrix.GetInverse()
        quat = transform_matrix.ExtractRotation().GetQuat()
        quat_np = np.array([
            quat.GetReal(),
            quat.GetImaginary()[0],
            quat.GetImaginary()[1],
            quat.GetImaginary()[2]
        ])
        eye_np = np.array([eye[0], eye[1], eye[2]])

        camera_rig.set_world_pose(position=eye_np, orientation=quat_np)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        await rep.orchestrator.step_async(rt_subframes=64, delta_time=0.0, pause_timeline=False)

        rgb_data = rgb_annotator.get_data()
        if rgb_data is not None and rgb_data.size > 0:
            rgb_np = rgb_data[:, :, :3].copy()
            rgb_filename = os.path.join(RGB_DIR, f"rgb_{i:04d}.png")
            from PIL import Image as PILImage
            pil_img = PILImage.fromarray(rgb_np)
            pil_img.save(rgb_filename)

        dist_data = distance_annotator.get_data()
        if dist_data is not None and dist_data.size > 0:
            depth_np = dist_data.copy()
            if depth_np.ndim == 3:
                depth_np = depth_np[:, :, 0]
            depth_filename = os.path.join(DEPTH_DIR, f"depth_{i:04d}.npy")
            np.save(depth_filename, depth_np)

        if (i + 1) % 10 == 0:
            print(f"已采集 {i + 1}/{NUM_CAPTURES} 张 (半径={radius:.2f}m, 高度={height:.2f}m, 角度={math.degrees(angle):.0f}°)")

    print(f"采集完成！共 {NUM_CAPTURES} 张图片")
    print(f"RGB 图片: {RGB_DIR}")
    print(f"深度数据: {DEPTH_DIR}")

    rgb_annotator.detach()
    distance_annotator.detach()
    rp.destroy()

    timeline.stop()
    await omni.kit.app.get_app().next_update_async()


asyncio.ensure_future(setup_and_run())
