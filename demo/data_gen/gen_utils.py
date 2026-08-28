import os
import asyncio
import numpy as np
import cv2


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


def visualize_depth(epoch_dir):
    depth_dir = os.path.join(epoch_dir, "depth")
    rgb_dir = os.path.join(epoch_dir, "rgb")
    vis_dir = os.path.join(epoch_dir, "depth_vis")
    if not os.path.isdir(depth_dir):
        return
    os.makedirs(vis_dir, exist_ok=True)

    depth_files = sorted([f for f in os.listdir(depth_dir) if f.endswith('.npy')])
    for df in depth_files:
        depth = np.load(os.path.join(depth_dir, df))
        d_min, d_max = depth.min(), depth.max()
        if d_max - d_min < 1e-6:
            norm = np.zeros_like(depth, dtype=np.uint8)
        else:
            norm = ((depth - d_min) / (d_max - d_min) * 255).astype(np.uint8)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        base = os.path.splitext(df)[0]
        cv2.imwrite(os.path.join(vis_dir, f'{base}_color.png'), color)
        cv2.imwrite(os.path.join(vis_dir, f'{base}_gray.png'), norm)

        rgb_name = df.replace('depth_', 'rgb_').replace('.npy', '.png')
        rgb_path = os.path.join(rgb_dir, rgb_name)
        if os.path.exists(rgb_path):
            rgb = cv2.imread(rgb_path)
            h, w = rgb.shape[:2]
            color_resized = cv2.resize(color, (w, h))
            side_by_side = np.hstack([rgb, color_resized])
            cv2.imwrite(os.path.join(vis_dir, f'{base}_compare.png'), side_by_side)

    print(f"    深度可视化: {vis_dir} ({len(depth_files)} 帧)")


def visualize_pointcloud(epoch_dir, sample_count=50000):
    pc_dir = os.path.join(epoch_dir, "pointcloud")
    vis_dir = os.path.join(epoch_dir, "pointcloud_vis")
    if not os.path.isdir(pc_dir):
        return
    os.makedirs(vis_dir, exist_ok=True)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ply_files = sorted([f for f in os.listdir(pc_dir) if f.endswith('.ply')])
    for pf in ply_files:
        pts = np.loadtxt(os.path.join(pc_dir, pf), skiprows=12)
        if len(pts) == 0:
            continue

        step = max(1, len(pts) // sample_count)
        pts = pts[::step]

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        colors = z

        fig = plt.figure(figsize=(20, 6))

        ax1 = fig.add_subplot(131, projection='3d')
        sc1 = ax1.scatter(x, y, z, c=colors, cmap='jet', s=0.3, alpha=0.6)
        ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
        ax1.set_title('Top-down (XY)')
        ax1.view_init(elev=90, azim=-90)
        fig.colorbar(sc1, ax=ax1, shrink=0.5, label='Z (m)')

        ax2 = fig.add_subplot(132, projection='3d')
        sc2 = ax2.scatter(x, y, z, c=colors, cmap='jet', s=0.3, alpha=0.6)
        ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
        ax2.set_title('Front (XZ)')
        ax2.view_init(elev=0, azim=-90)
        fig.colorbar(sc2, ax=ax2, shrink=0.5, label='Z (m)')

        ax3 = fig.add_subplot(133, projection='3d')
        sc3 = ax3.scatter(x, y, z, c=colors, cmap='jet', s=0.3, alpha=0.6)
        ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')
        ax3.set_title('Perspective')
        ax3.view_init(elev=30, azim=-60)
        fig.colorbar(sc3, ax=ax3, shrink=0.5, label='Z (m)')

        base = os.path.splitext(pf)[0]
        fig.suptitle(f'{base} ({len(pts)} pts)', fontsize=14)
        plt.tight_layout()
        out_path = os.path.join(vis_dir, f'{base}_3views.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        plt.close(fig)

    print(f"    点云可视化: {vis_dir} ({len(ply_files)} 帧)")


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
