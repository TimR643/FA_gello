import argparse
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


PANDA_LOWER = np.array([
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973,
    0.0,
], dtype=np.float32)

PANDA_UPPER = np.array([
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973,
    2.2,
], dtype=np.float32)


def image_to_tensor(img: np.ndarray, device: str) -> torch.Tensor:
    x = torch.from_numpy(img).float() / 255.0
    x = x.permute(2, 0, 1)
    return x.unsqueeze(0).to(device)


def state_to_tensor(state: np.ndarray, device: str) -> torch.Tensor:
    x = torch.from_numpy(np.asarray(state, dtype=np.float32))
    return x.unsqueeze(0).to(device)


def get_image_from_obs(obs: dict) -> np.ndarray:
    if "wrist_rgb" in obs:
        return obs["wrist_rgb"]
    if "wrist" in obs:
        return obs["wrist"]
    if "base_rgb" in obs:
        return obs["base_rgb"]
    if "base" in obs:
        return obs["base"]

    raise KeyError(f"No image key found. Available keys: {list(obs.keys())}")


def parse_vec(text: str, dtype=float) -> np.ndarray:
    values = [v.strip() for v in text.split(",") if v.strip()]
    return np.array([dtype(v) for v in values])


def apply_action_mapping(action: np.ndarray, permutation: np.ndarray, signs: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    mapped = action[permutation].copy()
    mapped = mapped * signs + offsets
    return mapped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--max-delta", type=float, default=0.015)
    parser.add_argument("--max-gripper-delta", type=float, default=0.03)
    parser.add_argument(
        "--action-permutation",
        type=str,
        default="0,1,2,3,4,5,6,7",
        help="Comma-separated action index permutation to map policy action into robot joint order.",
    )
    parser.add_argument(
        "--action-signs",
        type=str,
        default="1,1,1,1,1,1,1,1",
        help="Comma-separated +/-1 signs applied after permutation.",
    )
    parser.add_argument(
        "--action-offsets",
        type=str,
        default="0,0,0,0,0,0,0,0",
        help="Comma-separated additive offsets in radians/meters after permutation and sign.",
    )
    parser.add_argument(
        "--hold-gripper",
        action="store_true",
        help="Override gripper command with current gripper state.",
    )
    args = parser.parse_args()

    permutation = parse_vec(args.action_permutation, int)
    signs = parse_vec(args.action_signs, float)
    offsets = parse_vec(args.action_offsets, float)

    if permutation.shape != (8,):
        raise ValueError(f"--action-permutation must contain exactly 8 values, got {permutation}")
    if signs.shape != (8,):
        raise ValueError(f"--action-signs must contain exactly 8 values, got {signs}")
    if offsets.shape != (8,):
        raise ValueError(f"--action-offsets must contain exactly 8 values, got {offsets}")

    if sorted(permutation.tolist()) != list(range(8)):
        raise ValueError(f"--action-permutation must be a permutation of 0..7, got {permutation.tolist()}")

    signs_are_binary = np.isin(signs, [-1.0, 1.0]).all()
    if not signs_are_binary:
        raise ValueError(f"--action-signs entries must be only -1 or 1, got {signs.tolist()}")

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

    print("\nSAFE POLICY ROLLOUT")
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("max_delta joints:", args.max_delta)
    print("max_delta gripper:", args.max_gripper_delta)
    print("hold_gripper:", args.hold_gripper)
    print("action_permutation:", permutation.tolist())
    print("action_signs:", signs.astype(int).tolist())
    print("action_offsets:", np.round(offsets, 4).tolist())
    print("\nKeep your hand on the Franka enabling switch / emergency stop.")
    print("First test should be WITHOUT objects in the workspace.")

    if not args.execute:
        print("\nDry run only. Add --execute to actually move the robot.")

    input("\nPress ENTER to start...")

    dt = 1.0 / args.hz
    steps = int(args.duration * args.hz)

    for i in range(steps):
        obs = env.get_obs()

        state = np.asarray(obs["joint_positions"], dtype=np.float32)
        img = get_image_from_obs(obs)

        batch = {
            "observation.state": state_to_tensor(state, device),
            "observation.images.wrist": image_to_tensor(img, device),
        }

        with torch.no_grad():
            pred_action = policy.select_action(batch)

        pred_action = pred_action.squeeze(0).detach().cpu().numpy().astype(np.float32)

        if pred_action.shape != (8,):
            raise RuntimeError(f"Expected action shape (8,), got {pred_action.shape}")

        if not np.all(np.isfinite(pred_action)):
            raise RuntimeError(f"Policy produced NaN/Inf action: {pred_action}")

        mapped_action = apply_action_mapping(pred_action, permutation, signs, offsets)
        if args.hold_gripper:
            mapped_action[7] = state[7]

        raw_delta = mapped_action - state

        clipped_delta = raw_delta.copy()
        clipped_delta[:7] = np.clip(clipped_delta[:7], -args.max_delta, args.max_delta)
        clipped_delta[7] = np.clip(
            clipped_delta[7],
            -args.max_gripper_delta,
            args.max_gripper_delta,
        )

        target = state + clipped_delta
        target = np.clip(target, PANDA_LOWER, PANDA_UPPER)

        print(f"\nStep {i + 1}/{steps}")
        print("state      :", np.round(state, 3))
        print("policy raw :", np.round(pred_action, 3))
        print("policy map :", np.round(mapped_action, 3))
        print("raw delta  :", np.round(raw_delta, 3))
        print("sent target:", np.round(target, 3))

        if args.execute:
            env.step(target)

        time.sleep(dt)

    print("\nFinished safe rollout.")


if __name__ == "__main__":
    main()
