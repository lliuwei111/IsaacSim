import sys
from isaacsim import SimulationApp

CAMERA_STAGE_PATH = "/Camera"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

# 必须是 headless: True 才能在 Docker 中推流给浏览器
CONFIG = {"renderer": "RaytracedLighting", "headless": True}

simulation_app = SimulationApp(CONFIG)

import carb
import omni
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, stage
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

# 1. 启用 ROS 2 桥接扩展
extensions.enable_extension("isaacsim.ros2.bridge")

# 🌟 修复一：显式启用 WebRTC 推流扩展
extensions.enable_extension("omni.kit.livestream.webrtc")

# 强制引擎空跑几帧，确保所有底层插件和网络推流端口就绪
for _ in range(10):
    simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# 加载场景
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# 创建相机
camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim)
xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim.GetHorizontalApertureAttr().Set(21)
camera_prim.GetVerticalApertureAttr().Set(16)
camera_prim.GetProjectionAttr().Set("perspective")
camera_prim.GetFocalLengthAttr().Set(24)
camera_prim.GetFocusDistanceAttr().Set(400)

simulation_app.update()

# 构建 OmniGraph 节点来推流给 ROS 2
keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": ROS_CAMERA_GRAPH_PATH,
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
            ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
            ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
        ],
    },
)

og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

# 🌟 修复二：致命 BUG 修复核心区 🌟
# 在 Headless 模式下，Viewport 初始化是异步的，必须阻塞等待它加载出来，绝不能用 if 悄悄跳过
viewport_api = get_active_viewport()
while viewport_api is None:
    print("等待渲染视口 (Viewport) 初始化...")
    simulation_app.update()
    viewport_api = get_active_viewport()

print("Viewport 初始化成功，正在绑定推流相机...")
# 强制将 WebRTC 抓取的视口与我们创建的 "/Camera" 绑定
viewport_api.set_active_camera(CAMERA_STAGE_PATH)

import omni.syntheticdata._syntheticdata as sd

rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path()
)

rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
    sd.SensorType.DistanceToImagePlane.name
)
depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path()
)

camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
)

rgb_step_size = 5
depth_step_size = 60
info_step_size = 1

og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(rgb_step_size)
og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(depth_step_size)
og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

simulation_context.initialize_physics()
simulation_context.play()

frame = 0
print("开始渲染推流循环...")

while simulation_app.is_running():
    simulation_context.step(render=True)

    if simulation_context.is_playing():
        # 每一帧缓慢转动相机
        xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        frame = frame + 1

simulation_context.stop()
simulation_app.close()