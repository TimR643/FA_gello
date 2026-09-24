from pathlib import Path
import argparse
import pickle
import shutil
from typing import Iterable

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

IMAGE_H = 480
IMAGE_W = 640
IMAGE_C = 3

SUPPORTED_CAMERAS = ("wrist", "base")


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
    # Dateinamen wie:
    # 2026-05-18T13:38:09.360192.pkl
    # alphabetisches Sortieren passt zeitlich.
    return sorted(episode_dir.glob("*.pkl"))


def validate_cameras(cameras: Iterable[str]) -> list[str]:
    cameras = list(cameras)

    if not cameras:
        raise ValueError("Mindestens eine Kamera muss angegeben werden, z. B. --cameras wrist")

    invalid = [cam for cam in cameras if cam not in SUPPORTED_CAMERAS]
    if invalid:
        raise ValueError(
            f"Ungültige Kamera(s): {invalid}. Unterstützt sind: {SUPPORTED_CAMERAS}"
        )

    # Duplikate entfernen, Reihenfolge behalten
    unique = []
    for cam in cameras:
        if cam not in unique:
            unique.append(cam)

    return unique


def validate_frame(data: dict, pkl_path: Path, cameras: list[str]):
    required_keys = [
        "joint_positions",
        "control",
    ]

    for cam in cameras:
        required_keys.append(f"{cam}_rgb")

    for key in required_keys:
        if key not in data:
            raise KeyError(f"{pkl_path} enthält den Key '{key}' nicht.")

    state = np.asarray(data["joint_positions"], dtype=np.float32)
    action = np.asarray(data["control"], dtype=np.float32)

    if state.shape != (8,):
        raise ValueError(
            f"{pkl_path}: joint_positions hat shape {state.shape}, erwartet (8,)"
        )

    if action.shape != (8,):
        raise ValueError(
            f"{pkl_path}: control hat shape {action.shape}, erwartet (8,)"
        )

    if not np.all(np.isfinite(state)):
        raise ValueError(f"{pkl_path}: joint_positions enthält NaN oder Inf")

    if not np.all(np.isfinite(action)):
        raise ValueError(f"{pkl_path}: control enthält NaN oder Inf")

    rgb_images = {}

    for cam in cameras:
        key = f"{cam}_rgb"
        rgb = np.asarray(data[key])

        if rgb.shape != (IMAGE_H, IMAGE_W, IMAGE_C):
            raise ValueError(
                f"{pkl_path}: {key} hat shape {rgb.shape}, "
                f"erwartet {(IMAGE_H, IMAGE_W, IMAGE_C)}"
            )

        if rgb.dtype != np.uint8:
            rgb = rgb.astype(np.uint8)

        rgb_images[cam] = rgb

    return state, action, rgb_images


def build_features(cameras: list[str]):
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

    for cam in cameras:
        features[f"observation.images.{cam}"] = {
            "dtype": "video",
            "shape": (IMAGE_H, IMAGE_W, IMAGE_C),
            "names": ["height", "width", "channel"],
        }

    return features


def create_or_load_dataset(
    output_root: Path,
    repo_id: str,
    fps: int,
    cameras: list[str],
    append: bool,
    robot_type: str,
):
    features = build_features(cameras)

    if output_root.exists() and append:
        print(f"Bestehender LeRobot-Datensatz wird erweitert: {output_root}")

        dataset = LeRobotDataset.resume(
            repo_id=repo_id,
            root=output_root,
        )

        return dataset

    if output_root.exists() and not append:
        print(f"Output-Ordner existiert bereits und wird gelöscht: {output_root}")
        shutil.rmtree(output_root)

    print(f"Neuer LeRobot-Datensatz wird erstellt: {output_root}")
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        robot_type=robot_type,
        root=output_root,
        use_videos=True,
    )

    return dataset


def convert(
    raw_root: Path,
    output_root: Path,
    repo_id: str,
    fps: int,
    task: str,
    cameras: list[str],
    append: bool,
    robot_type: str,
):
    raw_root = raw_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    cameras = validate_cameras(cameras)

    episode_dirs = find_episodes(raw_root)

    if not episode_dirs:
        raise RuntimeError(f"Keine .pkl-Dateien gefunden unter: {raw_root}")

    print(f"Raw root: {raw_root}")
    print(f"Output root: {output_root}")
    print(f"Repo ID: {repo_id}")
    print(f"FPS: {fps}")
    print(f"Append: {append}")
    print(f"Cameras: {cameras}")

    print(f"\nGefundene Episoden: {len(episode_dirs)}")
    for i, ep in enumerate(episode_dirs):
        print(f"  Episode {i}: {ep}")

    dataset = create_or_load_dataset(
        output_root=output_root,
        repo_id=repo_id,
        fps=fps,
        cameras=cameras,
        append=append,
        robot_type=robot_type,
    )

    total_frames = 0
    converted_episodes = 0

    for episode_index, episode_dir in enumerate(episode_dirs):
        frame_paths = get_sorted_frames(episode_dir)

        if not frame_paths:
            print(f"Überspringe leere Episode: {episode_dir}")
            continue

        print(f"\nKonvertiere Episode {episode_index}: {episode_dir}")
        print(f"Frames: {len(frame_paths)}")

        for frame_index, pkl_path in enumerate(frame_paths):
            data = load_pkl(pkl_path)
            state, action, rgb_images = validate_frame(data, pkl_path, cameras)

            frame = {
                "observation.state": state,
                "action": action,
                "task": task,
            }

            for cam in cameras:
                frame[f"observation.images.{cam}"] = rgb_images[cam]

            dataset.add_frame(frame)
            total_frames += 1

        dataset.save_episode()
        converted_episodes += 1

    dataset.finalize()

    print("\nFertig.")
    print(f"Neu konvertierte Episoden: {converted_episodes}")
    print(f"Neue Frames gesamt: {total_frames}")
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

    parser.add_argument(
        "--cameras",
        nargs="+",
        default=["wrist"],
        choices=SUPPORTED_CAMERAS,
        help="Welche RGB-Kameras exportiert werden sollen. Beispiele: --cameras wrist oder --cameras wrist base",
    )

    parser.add_argument(
        "--append",
        action="store_true",
        help="Neue Episoden an bestehenden LeRobot-Datensatz anhängen, statt den Output-Ordner zu löschen.",
    )

    parser.add_argument(
        "--robot-type",
        type=str,
        default="panda_gello",
        help="robot_type-Metadatenfeld für LeRobotDataset.create.",
    )

    args = parser.parse_args()

    convert(
        raw_root=args.raw_root,
        output_root=args.output_root,
        repo_id=args.repo_id,
        fps=args.fps,
        task=args.task,
        cameras=args.cameras,
        append=args.append,
        robot_type=args.robot_type,
    )


if __name__ == "__main__":
    main()