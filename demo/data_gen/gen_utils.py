import os
import asyncio
import numpy as np


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
        np.savetxt(f, points[:, :3], fmt='%f %f %f')


async def convert_glb_to_usd(input_glb, output_usd):
    import omni.kit.asset_converter
    task_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    context.use_meter_as_world_unit = True
    context.ignore_up_axis = False
    context.import_materials = True
    task = task_manager.create_converter_task(input_glb, output_usd, None, context)
    success = await task.wait_until_finished()
    return success


def ensure_usd_asset(glb_path, usd_dir):
    os.makedirs(usd_dir, exist_ok=True)
    if not os.path.exists(glb_path):
        print(f"[错误] GLB 文件不存在: {glb_path}")
        return None
    usd_path = os.path.join(usd_dir, os.path.splitext(os.path.basename(glb_path))[0] + ".usd")
    return usd_path


def convert_glb_if_needed(glb_path, usd_path):
    if os.path.exists(usd_path):
        return True
    print(f"转换 GLB -> USD: {os.path.basename(glb_path)}")
    loop = asyncio.get_event_loop()
    success = loop.run_until_complete(convert_glb_to_usd(glb_path, usd_path))
    if success:
        print(f"  成功: {usd_path}")
    else:
        print(f"  失败: {glb_path}")
    return success
