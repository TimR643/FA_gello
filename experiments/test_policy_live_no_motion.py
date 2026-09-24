import time
import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets import LeRobotDataset
from lerobot.policies.factory import make_policy

from gello.env import RobotEnv
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.zmq_core.camera_node import ZMQClientCamera


CHECKPOINT = "/home/tim/lerobot_outputs/panda_red_sorting_act_10k/checkpoints/010000/pretrained_model"
DATASET_ROOT = "/home/tim/lerobot_data/panda_red_sorting_v1"
REPO_ID = "tim/panda_red_sorting_v1"


def image_to_tensor(img: np.ndarray, device: str) -> torch.Tensor:
    # img kommt live als H,W,C uint8 RGB
    x = torch.from_numpy(img).float() / 255.0
    x = x.permute(2, 0, 1)  # H,W,C -> C,H,W
    return x.unsqueeze(0).to(device)


def state_to_tensor(state: np.ndarray, device: str) -> torch.Tensor:
    x = torch.from_numpy(np.asarray(state, dtype=np.float32))
    return x.unsqueeze(0).to(device)


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Using device:", device)

    dataset = LeRobotDataset(
        repo_id=REPO_ID,
        root=DATASET_ROOT,
    )

    cfg = PreTrainedConfig.from_pretrained(CHECKPOINT)
    cfg.device = device

    policy = make_policy(cfg=cfg, ds_meta=dataset.meta)
    policy.to(device)
    policy.eval()

    robot = ZMQClientRobot(port=6001, host="127.0.0.1")
    cameras = {
        "wrist": ZMQClientCamera(port=5000, host="127.0.0.1"),
    }
    env = RobotEnv(robot, camera_dict=cameras)

    print("Reading live observations and predicting actions.")
    print("The robot will NOT move in this script.")

    for i in range(50):
        obs = env.get_obs()

        if i == 0:
            print("OBS KEYS:", list(obs.keys()))

        state = obs["joint_positions"]

        if "wrist_rgb" in obs:
            img = obs["wrist_rgb"]
        elif "wrist" in obs:
            img = obs["wrist"]
        elif "base_rgb" in obs:
            img = obs["base_rgb"]
        elif "base" in obs:
            img = obs["base"]
        else:
            raise KeyError(f"No camera image key found. Available keys: {list(obs.keys())}")

        batch = {
            "observation.state": state_to_tensor(state, device),
            "observation.images.wrist": image_to_tensor(img, device),
        }

        with torch.no_grad():
            pred_action = policy.select_action(batch)

        pred_action = pred_action.squeeze(0).detach().cpu().numpy()

        print(f"\nStep {i}")
        print("current state:", np.round(state, 3))
        print("pred action  :", np.round(pred_action, 3))
        print("delta        :", np.round(pred_action - state, 3))

        time.sleep(0.2)


if __name__ == "__main__":
    main()
