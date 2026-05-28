import argparse
import time
import numpy as np
import torch
import cv2

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets import LeRobotDataset
from lerobot.policies.factory import make_policy

from gello.env import RobotEnv
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.zmq_core.camera_node import ZMQClientCamera


CHECKPOINT = "/home/tim/pick_red_block_output/checkpoints/100000/pretrained_model"
DATASET_ROOT = "/home/tim/lerobot_data/pick_red_block"
REPO_ID = "tim/pick_red_block"


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


def prepare_image_for_lerobot(img: np.ndarray, device: str) -> torch.Tensor:
    """
    Konvertiert das Kamerabild in das von LeRobot erwartete Format.
    GELLO/OpenCV liefert BGR in (H, W, C). LeRobot erwartet RGB in (C, H, W) normiert auf [0, 1].
    """
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = torch.from_numpy(img_rgb).float() / 255.0
    x = x.permute(2, 0, 1)
    return x.unsqueeze(0).to(device)


def state_to_tensor(state: np.ndarray, device: str) -> torch.Tensor:
    x = torch.from_numpy(np.asarray(state, dtype=np.float32))
    return x.unsqueeze(0).to(device)


def get_image_from_obs(obs: dict) -> np.ndarray:
    if "wrist" in obs: return obs["wrist"]
    if "wrist_rgb" in obs: return obs["wrist_rgb"]
    if "base" in obs: return obs["base"]
    if "base_rgb" in obs: return obs["base_rgb"]
    raise KeyError(f"Kein Bild-Key gefunden. Verfügbare Keys: {list(obs.keys())}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--max-delta", type=float, default=0.06)
    parser.add_argument("--max-gripper-delta", type=float, default=0.08)
    parser.add_argument("--chunk-size", type=int, default=20, help="Anzahl der vorausschauend geplanten Schritte.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Using device:", device)

    dataset = LeRobotDataset(
        repo_id=REPO_ID,
        root=DATASET_ROOT,
    )

    cfg = PreTrainedConfig.from_pretrained(CHECKPOINT)
    cfg.device = device

    # Dynamisches Überschreiben der Chunk-Size für reaktiveres Verhalten
    if hasattr(cfg, "chunk_size"):
        print(f"-> Überschreibe trainierte Chunk-Size ({cfg.chunk_size}) temporär auf: {args.chunk_size}")
        cfg.chunk_size = args.chunk_size
    elif hasattr(cfg, "action_chunk_size"):
        print(f"-> Überschreibe trainierte Action-Chunk-Size ({cfg.action_chunk_size}) temporär auf: {args.chunk_size}")
        cfg.action_chunk_size = args.chunk_size

    policy = make_policy(cfg=cfg, ds_meta=dataset.meta)
    policy.to(device)
    policy.eval()
    policy.reset()

    robot = ZMQClientRobot(port=6001, host="127.0.0.1")
    cameras = {
        "wrist": ZMQClientCamera(port=5000, host="127.0.0.1"),
    }

    env = RobotEnv(robot, camera_dict=cameras)

    print("\nSAFE POLICY ROLLOUT (REAKTIVE GELENK-ANZEIGE)")
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("chunk_size:", args.chunk_size)
    print("\n!! Notaus / Zustimmtaster bereit halten !!")

    input("\nDrücke ENTER zum Starten...")

    dt = 1.0 / args.hz
    steps = int(args.duration * args.hz)

    for i in range(steps):
        start_time = time.time()
        obs = env.get_obs()

        state = np.asarray(obs["joint_positions"], dtype=np.float32)
        img = get_image_from_obs(obs)

        batch = {
            "observation.state": state_to_tensor(state, device),
            "observation.images.wrist": prepare_image_for_lerobot(img, device),
        }

        with torch.no_grad():
            pred_action = policy.select_action(batch)

        pred_action = pred_action.squeeze(0).detach().cpu().numpy().astype(np.float32)

        raw_delta = pred_action - state

        clipped_delta = raw_delta.copy()
        clipped_delta[:7] = np.clip(clipped_delta[:7], -args.max_delta, args.max_delta)
        clipped_delta[7] = np.clip(
            clipped_delta[7],
            -args.max_gripper_delta,
            args.max_gripper_delta,
        )

        target = state + clipped_delta
        target = np.clip(target, PANDA_LOWER, PANDA_UPPER)

        # Ausführliche Terminal-Ausgabe aller Gelenkwinkel für perfektes Debugging
        print(f"\n--- Schritt {i + 1}/{steps} ---")
        print(f"J1: Ist {state[0]:6.3f} -> Soll {target[0]:6.3f} (Policy: {pred_action[0]:6.3f})")
        print(f"J2: Ist {state[1]:6.3f} -> Soll {target[1]:6.3f} (Policy: {pred_action[1]:6.3f})")
        print(f"J3: Ist {state[2]:6.3f} -> Soll {target[2]:6.3f} (Policy: {pred_action[2]:6.3f})")
        print(f"J4: Ist {state[3]:6.3f} -> Soll {target[3]:6.3f} (Policy: {pred_action[3]:6.3f})")
        print(f"J5: Ist {state[4]:6.3f} -> Soll {target[4]:6.3f} (Policy: {pred_action[4]:6.3f})")
        print(f"J6: Ist {state[5]:6.3f} -> Soll {target[5]:6.3f} (Policy: {pred_action[5]:6.3f})")
        print(f"J7: Ist {state[6]:6.3f} -> Soll {target[6]:6.3f} (Policy: {pred_action[6]:6.3f})")
        print(f"GR: Ist {state[7]:6.3f} -> Soll {target[7]:6.3f} (Policy: {pred_action[7]:6.3f})")

        if args.execute:
            env.step(target)

        elapsed = time.time() - start_time
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)

    print("\nRollout beendet.")


if __name__ == "__main__":
    main()