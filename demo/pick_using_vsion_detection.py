import sys
import numpy as np
import cv2
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")
enable_extension("omni.replicator.core")
enable_extension("isaacsim.sensors.rtx")

for _ in range(10):
    simulation_app.update()

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.viewports import set_camera_view
from isaacsim.core.utils.prims import create_prim
from pxr import UsdGeom, Gf
import omni.usd

viewport_api = get_active_viewport()
while viewport_api is None:
    simulation_app.update()
    viewport_api = get_active_viewport()

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

franka = world.scene.add(
    Franka(prim_path="/World/Franka", name="franka_robot")
)

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.3, 0.3, 0.025]),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0.0, 0.0, 1.0]),
        mass=0.1
    )
)

stage = omni.usd.get_context().get_stage()
camera_path = "/World/Franka/camera"
cam = UsdGeom.Camera.Define(stage, camera_path)
cam.GetFocalLengthAttr().Set(24.0)
cam.GetHorizontalApertureAttr().Set(20.955)
cam.GetClippingRangeAttr().Set((0.01, 1000000.0))

xform_api = UsdGeom.XformCommonAPI(cam)
xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.1))
xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

cam_rp = rep.create.render_product(camera_path, (IMAGE_WIDTH, IMAGE_HEIGHT))

rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach(cam_rp)

depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
depth_annotator.attach(cam_rp)

controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)

print("正在编译底层运动学图 (RMPflow)，请稍候...")
world.reset()

pos_A = np.array([0.3, 0.3, 0.025])
pos_B = np.array([0.3, -0.3, 0.025])
current_place_pos = pos_B

def detect_blue_object(rgb_data):
    """通过颜色检测蓝色物体，返回2D像素坐标"""
    if rgb_data is None or rgb_data.size == 0:
        return None
    
    rgb_image = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2RGB)
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
    return None

def pixel_to_world(cx, cy, depth_value, camera_prim, image_width, image_height):
    """将2D像素坐标和深度值转换为3D世界坐标"""
    focal_length = camera_prim.GetFocalLengthAttr().Get()
    horizontal_aperture = camera_prim.GetHorizontalApertureAttr().Get()
    
    fx = focal_length * image_width / horizontal_aperture
    fy = fx
    
    x_cam = (cx - image_width / 2.0) * depth_value / fx
    y_cam = (cy - image_height / 2.0) * depth_value / fy
    z_cam = depth_value
    
    camera_prim_path = str(camera_prim.GetPath())
    from omni.isaac.core.prims import XFormPrim
    camera_xform = XFormPrim(prim_path=camera_prim_path)
    cam_pos, cam_orient = camera_xform.get_world_pose()
    
    from scipy.spatial.transform import Rotation as R
    rot = R.from_quat([cam_orient[1], cam_orient[2], cam_orient[3], cam_orient[0]])
    rotation_matrix = rot.as_matrix()
    
    point_in_camera = np.array([x_cam, -y_cam, -z_cam])
    point_in_world = rotation_matrix @ point_in_camera + cam_pos
    
    return point_in_world

def get_depth_at_pixel(depth_data, cx, cy, kernel_size=5):
    """获取像素点周围的平均深度值"""
    half_k = kernel_size // 2
    h, w = depth_data.shape
    
    x_start = max(0, cx - half_k)
    x_end = min(w, cx + half_k + 1)
    y_start = max(0, cy - half_k)
    y_end = min(h, cy + half_k + 1)
    
    region = depth_data[y_start:y_end, x_start:x_end]
    valid_depths = region[region > 0]
    
    if len(valid_depths) > 0:
        return np.median(valid_depths)
    return None

print("====================================================")
print("✅ 视觉引导抓取系统就绪！")
print("👉 请在浏览器中打开 http://localhost:8211 查看")
print("====================================================")

frame_count = 0
detected_position = None
detection_confidence = 0

try:
    import scipy
except ImportError:
    print("⚠️ 正在安装 scipy...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy", "-q"])
    import scipy

while simulation_app.is_running():
    world.step(render=True)
    
    if world.is_playing():
        frame_count += 1
        
        if frame_count % 3 == 0:
            rep.orchestrator.step()
            
            rgb_data = rgb_annotator.get_data()
            depth_data = depth_annotator.get_data()
            
            if rgb_data is not None and depth_data is not None:
                pixel_pos = detect_blue_object(rgb_data)
                
                if pixel_pos:
                    cx, cy = pixel_pos
                    depth_value = get_depth_at_pixel(depth_data, cx, cy)
                    
                    if depth_value and depth_value > 0.1 and depth_value < 2.0:
                        detected_position = pixel_to_world(
                            cx, cy, depth_value, cam,
                            IMAGE_WIDTH, IMAGE_HEIGHT
                        )
                        detected_position[2] = max(detected_position[2], 0.025)
                        detection_confidence = min(detection_confidence + 1, 10)
                        
                        if frame_count % 30 == 0:
                            print(f"🎯 检测到物体: 像素({cx}, {cy}), 深度: {depth_value:.3f}m")
                            print(f"   世界坐标: [{detected_position[0]:.3f}, {detected_position[1]:.3f}, {detected_position[2]:.3f}]")
        
        if detected_position is not None and detection_confidence >= 3:
            actions = controller.forward(
                picking_position=detected_position,
                placing_position=current_place_pos,
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0, 0, 0.02]),
            )
            franka.apply_action(actions)
            
            if controller.is_done():
                print("🎯 一次搬运完成！")
                
                if np.allclose(current_place_pos, pos_B):
                    current_place_pos = pos_A
                else:
                    current_place_pos = pos_B
                
                controller.reset()
                detection_confidence = 0

world.stop()
simulation_app.close()