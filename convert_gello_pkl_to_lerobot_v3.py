from pathlib import Path
import argparse
import pickle
import shutil

import numpy as np

from lerobot.datasets import LeRobotDataset


JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
    "gripper",
]


def load_pkl(path: Path) -> dict:
    with open(path, "rb") as f:
        return pickle.load(f)


def find_episodes(raw_root: Path):
    """
    Unterstützt zwei Fälle:

    Fall A:
        raw_root/
            0507_104424/
                *.pkl
            0507_105012/
                *.pkl

    Fall B:
        raw_root/
            *.pkl

    Bei Fall B wird raw_root als eine Episode behandelt.
    """
    direct_pkls = sorted(raw_root.glob("*.pkl"))

    if direct_pkls:
        return [raw_root]

    episode_dirs = []
    for p in sorted(raw_root.iterdir()):
        if p.is_dir() and list(p.glob("*.pkl")):
            episode_dirs.append(p)

    return episode_dirs


def get_sorted_frames(episode_dir: Path):
    # Deine Dateien heißen z. B.:
    # 2026-05-07T10:44:24.687412.pkl
    # alphabetisches Sortieren passt hier zeitlich.
    return sorted(episode_dir.glob("*.pkl"))


def validate_frame(data: dict, pkl_path: Path):
    required_keys = ["joint_positions", "control"]

    for key in required_keys:
        if key not in data:
            raise KeyError(f"{pkl_path} enthält den Key '{key}' nicht.")

    state = np.asarray(data["joint_positions"], dtype=np.float32)
    action = np.asarray(data["control"], dtype=np.float32)

    if state.shape != (8,):
        raise ValueError(f"{pkl_path}: joint_positions hat shape {state.shape}, erwartet (8,)")

    if action.shape != (8,):
        raise ValueError(f"{pkl_path}: control hat shape {action.shape}, erwartet (8,)")

    if not np.all(np.isfinite(state)):
        raise ValueError(f"{pkl_path}: joint_positions enthält NaN oder Inf")

    if not np.all(np.isfinite(action)):
        raise ValueError(f"{pkl_path}: control enthält NaN oder Inf")

    return state, action


def convert(raw_root: Path, output_root: Path, repo_id: str, fps: int, task: str):
    raw_root = raw_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()

    if output_root.exists():
        print(f"Output-Ordner existiert bereits und wird gelöscht: {output_root}")
        shutil.rmtree(output_root)

    episode_dirs = find_episodes(raw_root)

    if not episode_dirs:
        raise RuntimeError(f"Keine .pkl-Dateien gefunden unter: {raw_root}")

    print(f"Gefundene Episoden: {len(episode_dirs)}")
    for i, ep in enumerate(episode_dirs):
        print(f"  Episode {i}: {ep}")

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (8,),
            "names": JOINT_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": JOINT_NAMES,
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        robot_type="panda_gello",
        root=output_root,
        use_videos=False,
    )

    total_frames = 0

    for episode_index, episode_dir in enumerate(episode_dirs):
        frame_paths = get_sorted_frames(episode_dir)

        if not frame_paths:
            print(f"Überspringe leere Episode: {episode_dir}")
            continue

        print(f"\nKonvertiere Episode {episode_index}: {episode_dir}")
        print(f"Frames: {len(frame_paths)}")

        for frame_index, pkl_path in enumerate(frame_paths):
            data = load_pkl(pkl_path)
            state, action = validate_frame(data, pkl_path)

            frame = {
                "observation.state": state,
                "action": action,
                "task": task,
            }

            dataset.add_frame(frame)
            total_frames += 1

        dataset.save_episode()

    dataset.finalize()

    print("\nFertig.")
    print(f"Episoden: {len(episode_dirs)}")
    print(f"Frames gesamt: {total_frames}")
    print(f"Output: {output_root}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--raw-root",
        type=Path,
        required=True,
        help="Ordner mit GELLO .pkl-Dateien oder Unterordnern pro Episode.",
    )

    parser.add_argument(
        "--output-root",
        type=Path,
        required=True,
        help="Zielordner für das LeRobot-v3-Dataset.",
    )

    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Dataset-ID, z. B. tim/panda_gello_test.",
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=10,
        help="Aufnahmefrequenz. Muss ungefähr zu deiner GELLO-Aufzeichnung passen.",
    )

    parser.add_argument(
        "--task",
        type=str,
        default="Teleoperate the Panda robot with GELLO.",
        help="Task-Beschreibung für LeRobot.",
    )

    args = parser.parse_args()

    convert(
        raw_root=args.raw_root,
        output_root=args.output_root,
        repo_id=args.repo_id,
        fps=args.fps,
        task=args.task,
    )


if __name__ == "__main__":
    main()