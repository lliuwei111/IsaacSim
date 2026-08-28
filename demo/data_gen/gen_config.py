import os
import random

INPUT_GLB_PATH = "/workspace/IsaacSim/demo/asset/data_gen/sample_tremos.glb"
USD_ASSETS_DIR = "/workspace/IsaacSim"
DATASET_OUTPUT_DIR = "/workspace/IsaacSim/data_gen_output"

NUM_EPOCHS = 10
FRAMES_PER_EPOCH = 150
POINTCLOUD_SAMPLE_INTERVAL = 30
IMAGE_SAVE_INTERVAL = 10
IMAGE_WIDTH = 1024
IMAGE_HEIGHT = 768
VIDEO_FPS = 30.0

OBJECT_SCALE_RANGE = (0.5, 2.0)
OBJECT_ROTATION_RANGE = ((0, 360), (0, 360), (0, 360))
OBJECT_DROP_HEIGHT_RANGE = (1.5, 3.0)
OBJECT_MASS_RANGE = (0.2, 2.0)

CAMERA_RADIUS_RANGE = (2.0, 6.0)
CAMERA_HEIGHT_RANGE = (0.5, 2.5)
CAMERA_ANGULAR_SPEED_RANGE = (0.005, 0.02)
CAMERA_TARGET_HEIGHT_RANGE = (0.3, 1.2)

LIGHT_INTENSITY_RANGE = (500, 3000)
FILL_LIGHT_INTENSITY_RANGE = (300, 2000)

ENVIRONMENTS = [
    "/Isaac/Environments/Simple_Room/simple_room.usd",
    "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
]

PHYSICS_SETTLE_FRAMES = 60

ENABLE_DEPTH_VIS = True
ENABLE_POINTCLOUD_VIS = True
DEPTH_VIS_SAMPLE_COUNT = 50000

SEED = None

RANDOM_STATE = random.Random(SEED)


def get_random_object_config():
    return {
        "scale": RANDOM_STATE.uniform(*OBJECT_SCALE_RANGE),
        "rotation": (
            RANDOM_STATE.uniform(*OBJECT_ROTATION_RANGE[0]),
            RANDOM_STATE.uniform(*OBJECT_ROTATION_RANGE[1]),
            RANDOM_STATE.uniform(*OBJECT_ROTATION_RANGE[2]),
        ),
        "drop_position": (
            RANDOM_STATE.uniform(-0.5, 0.5),
            RANDOM_STATE.uniform(-0.5, 0.5),
            RANDOM_STATE.uniform(*OBJECT_DROP_HEIGHT_RANGE),
        ),
        "mass": RANDOM_STATE.uniform(*OBJECT_MASS_RANGE),
    }


def get_random_camera_config():
    return {
        "radius": RANDOM_STATE.uniform(*CAMERA_RADIUS_RANGE),
        "height": RANDOM_STATE.uniform(*CAMERA_HEIGHT_RANGE),
        "angular_speed": RANDOM_STATE.uniform(*CAMERA_ANGULAR_SPEED_RANGE),
        "target_height": RANDOM_STATE.uniform(*CAMERA_TARGET_HEIGHT_RANGE),
    }


def get_random_light_config():
    return {
        "dome_intensity": RANDOM_STATE.uniform(*LIGHT_INTENSITY_RANGE),
        "fill_intensity": RANDOM_STATE.uniform(*FILL_LIGHT_INTENSITY_RANGE),
        "fill_angle": RANDOM_STATE.uniform(20, 60),
    }


def get_random_environment():
    return RANDOM_STATE.choice(ENVIRONMENTS)
