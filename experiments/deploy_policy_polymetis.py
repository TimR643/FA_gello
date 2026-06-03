"""Deploy a LeRobot policy on a Polymetis Panda and record rollouts.

This script mirrors the CRISP ``deploy_policy`` workflow for this repository's
Polymetis/GELLO stack: it loads a pretrained LeRobot policy, runs it on the real
Panda through Polymetis, records each episode in LeRobot format, and optionally
asks for success/failure evaluation after every episode.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import logging
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, Optional, Sequence, Tuple

import numpy as np
import torch

from gello.env import RobotEnv
from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    SafetyConfig,
    load_lerobot_policy,
    validate_policy_batch_keys,
)
from gello.utils.control_utils import LeRobotDatasetWriter
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.robot_node import ZMQClientRobot

LOGGER = logging.getLogger(__name__)


class CsvEpisodeEvaluator:
    """Small evaluation helper with the same episode-level CSV semantics."""

    def __init__(self, output_file: str) -> None:
        self.output_file = Path(output_file)
        self._start_time: Optional[float] = None
        self._active = False

    @contextmanager
    def start_eval(self, *, overwrite: bool, activate: bool) -> Iterator[None]:
        self._active = activate
        if self._active:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            if overwrite or not self.output_file.exists():
                with self.output_file.open("w", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(
                        f,
                        fieldnames=[
                            "episode",
                            "success",
                            "duration_s",
                            "timestamp",
                            "notes",
                        ],
                    )
                    writer.writeheader()
        yield

    def start_timer(self) -> None:
        self._start_time = time.monotonic()

    def evaluate(self, *, episode: int) -> None:
        if not self._active:
            return

        duration = 0.0
        if self._start_time is not None:
            duration = time.monotonic() - self._start_time

        answer = input("Episode erfolgreich? [y/n]: ").strip().lower()
        while answer not in {"y", "yes", "j", "ja", "n", "no", "nein"}:
            answer = input("Bitte y/j oder n eingeben: ").strip().lower()
        success = answer in {"y", "yes", "j", "ja"}
        notes = input("Notizen (optional): ").strip()

        with self.output_file.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["episode", "success", "duration_s", "timestamp", "notes"],
            )
            writer.writerow(
                {
                    "episode": episode,
                    "success": success,
                    "duration_s": f"{duration:.3f}",
                    "timestamp": dt.datetime.now().isoformat(timespec="seconds"),
                    "notes": notes,
                }
            )


class KeyboardEpisodeRecorder:
    """Keyboard-controlled LeRobot episode recorder.

    Press ``S`` in the pygame window to start an episode and ``Q`` to stop/save
    it. The class deliberately keeps policy inference and robot control inside
    the same frame loop so every saved frame receives the policy action that was
    sent to the Polymetis robot.
    """

    state: str = "idle"

    def __init__(
        self,
        *,
        writer: LeRobotDatasetWriter,
        num_episodes: int,
        fps: int,
    ) -> None:
        from gello.data_utils.keyboard_interface import KBReset

        self.writer = writer
        self.num_episodes = num_episodes
        self.fps = fps
        self.episode_count = 0
        self._kb = KBReset()

        print("Keyboard recording manager ready:")
        print("  S: Episode starten")
        print("  Q: Episode stoppen und speichern")

    def __enter__(self) -> "KeyboardEpisodeRecorder":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.writer.finalize()

    def wait_until_ready(self) -> None:
        LOGGER.info("Recording manager ready. Waiting for keyboard commands.")

    def done(self) -> bool:
        return self.episode_count >= self.num_episodes

    def record_episode(
        self,
        *,
        frame_fn: Any,
        on_start: Any,
        on_end: Any,
    ) -> None:
        LOGGER.info("Press S in the pygame window to start the next episode.")
        self.state = "waiting"
        while True:
            if self._kb.update() == "start":
                break
            time.sleep(0.02)

        self.state = "recording"
        on_start()
        period = 1.0 / self.fps

        while True:
            started = time.time()
            kb_state = self._kb.update()
            if kb_state == "normal":
                break

            obs, action = frame_fn()
            self.writer.add_frame(obs, action)

            remaining = period - (time.time() - started)
            if remaining > 0:
                time.sleep(remaining)

        self.state = "saving"
        self.writer.save_episode()
        self.episode_count += 1
        on_end()
        self.state = "idle"


def parse_cameras(value: str) -> Tuple[str, ...]:
    cameras = tuple(camera.strip() for camera in value.split(",") if camera.strip())
    if not cameras:
        raise argparse.ArgumentTypeError("At least one camera must be specified")
    supported = {"wrist", "base"}
    unsupported = set(cameras) - supported
    if unsupported:
        raise argparse.ArgumentTypeError(
            f"Unsupported cameras {sorted(unsupported)}; supported: {sorted(supported)}"
        )
    return cameras


def prompt_value(
    message: str, *, default: Optional[str] = None, options: Sequence[str] = ()
) -> str:
    if options:
        print(message)
        for idx, option in enumerate(options, start=1):
            print(f"  {idx}: {option}")
        if default is not None:
            print(f"Default: {default}")
    elif default is not None:
        message = f"{message} [{default}]"

    value = input(message + " ").strip()
    if not value and default is not None:
        return default
    if options and value.isdigit():
        index = int(value) - 1
        if 0 <= index < len(options):
            return options[index]
    return value


def find_pretrained_model() -> str:
    models_path = Path("outputs/train")
    if not models_path.exists() or not models_path.is_dir():
        raise FileNotFoundError(
            "'outputs/train' directory does not exist. Provide --path explicitly."
        )

    models = sorted(
        [
            str(model)
            for model in models_path.glob("**/pretrained_model")
            if model.is_dir()
        ],
        key=lambda item: item.lower(),
    )
    if not models:
        raise FileNotFoundError(
            "No 'pretrained_model' directory found below outputs/train. Provide --path."
        )
    return prompt_value(
        "Please select a model to use for deployment:",
        options=models,
        default=models[0],
    )


def resolve_pretrained_path(path: str) -> str:
    """Resolve a train output/checkpoint directory to a LeRobot pretrained_model path."""

    requested = Path(path).expanduser()
    candidates = [requested, requested / "pretrained_model"]

    checkpoints_dir = requested / "checkpoints"
    if checkpoints_dir.is_dir():
        checkpoint_models = [
            checkpoint / "pretrained_model"
            for checkpoint in checkpoints_dir.iterdir()
            if (checkpoint / "pretrained_model").is_dir()
        ]

        def checkpoint_sort_key(model_path: Path) -> Tuple[int, str]:
            checkpoint_name = model_path.parent.name
            return (
                int(checkpoint_name) if checkpoint_name.isdigit() else -1,
                checkpoint_name,
            )

        candidates.extend(
            sorted(checkpoint_models, key=checkpoint_sort_key, reverse=True)
        )

    for candidate in candidates:
        if (candidate / "config.json").is_file():
            resolved = str(candidate)
            if resolved != path:
                LOGGER.info("Resolved --path %s to pretrained model %s", path, resolved)
            return resolved

    searched = "\n".join(f"  - {candidate}" for candidate in candidates)
    raise FileNotFoundError(
        "Could not find a LeRobot policy config.json for --path. Pass the exact "
        ".../checkpoints/<step>/pretrained_model directory, or pass a train output "
        "directory containing checkpoints/*/pretrained_model/config.json.\n"
        f"Requested --path: {requested}\nSearched:\n{searched}"
    )


def make_camera_clients(
    *,
    camera_host: str,
    wrist_camera_port: int,
    base_camera_port: int,
    cameras: Tuple[str, ...],
) -> Dict[str, ZMQClientCamera]:
    clients: Dict[str, ZMQClientCamera] = {}
    for camera in cameras:
        if camera == "wrist":
            clients[camera] = ZMQClientCamera(port=wrist_camera_port, host=camera_host)
        elif camera == "base":
            clients[camera] = ZMQClientCamera(port=base_camera_port, host=camera_host)
    return clients


def make_policy_processors(
    args: argparse.Namespace, policy: Any, device: str
) -> Tuple[Callable[[Dict[str, Any]], Dict[str, Any]], Callable[[Any], Any]]:
    """Return runtime pre/post-processors needed around policy.select_action."""

    if not args.smolvla:
        return lambda batch: batch, lambda action: action

    from lerobot.policies import make_pre_post_processors

    preprocess, postprocess = make_pre_post_processors(
        policy.config,
        args.path,
        preprocessor_overrides={"device_processor": {"device": str(device)}},
    )

    def transform(batch: Dict[str, Any]) -> Dict[str, Any]:
        batch["task"] = [args.task]
        batch["robot_type"] = [""]

        if (
            "observation.images.wrist" in batch
            and "observation.images.camera1" not in batch
        ):
            batch["observation.images.camera1"] = batch.pop("observation.images.wrist")

        if "observation.images.camera1" not in batch:
            raise KeyError(
                "Missing observation.images.camera1 after wrist->camera1 mapping. "
                f"Available keys: {sorted(batch.keys())}"
            )

        camera1 = batch["observation.images.camera1"]
        batch.setdefault("observation.images.camera2", torch.zeros_like(camera1))
        batch.setdefault("observation.images.camera3", torch.zeros_like(camera1))
        return preprocess(batch)

    return transform, postprocess


def load_policy_for_args(
    args: argparse.Namespace, policy_dataset_root: str, policy_repo_id: str
) -> Any:
    """Load policy exactly like the working real-robot rollout scripts."""

    return load_lerobot_policy(
        checkpoint=args.path,
        dataset_root=policy_dataset_root,
        repo_id=policy_repo_id,
        device=args.device,
    )


def reset_policy(policy: Any) -> None:
    if hasattr(policy, "reset"):
        policy.reset()


def home_robot(env: RobotEnv) -> None:
    robot = env.robot()
    if hasattr(robot, "robot") and hasattr(robot.robot, "go_home"):
        robot.robot.go_home()
    if hasattr(robot, "gripper") and hasattr(robot.gripper, "goto"):
        robot.gripper.goto(width=0.09, speed=1.0, force=1.0)


def close_robot(env: RobotEnv) -> None:
    robot = env.robot()
    if hasattr(robot, "robot") and hasattr(robot.robot, "terminate_current_policy"):
        robot.robot.terminate_current_policy()
    if hasattr(robot, "close"):
        robot.close()


def make_robot_client(args: argparse.Namespace) -> Any:
    """Create the robot connection for the selected deployment topology.

    The default is ``zmq`` because this project usually runs LeRobot inference on
    the HPC while the realtime laptop owns Polymetis and exposes the Panda over
    a ZMQ robot server.  ``polymetis`` remains available for the rare case where
    this script is executed directly on the realtime laptop.
    """

    if args.robot_backend == "zmq":
        return ZMQClientRobot(port=args.robot_port, host=args.robot_host)

    if args.robot_backend == "polymetis":
        from gello.robots.panda import PandaRobot

        return PandaRobot(robot_ip=args.robot_ip)

    raise ValueError(f"Unsupported robot backend: {args.robot_backend}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Deploy a pretrained LeRobot policy on Polymetis and record data."
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        default=None,
        help="Repository ID for the recorded dataset",
    )
    parser.add_argument(
        "--robot-type", type=str, default="franka", help="Type of robot being used"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="Frames per second for recording and control",
    )
    parser.add_argument(
        "--num-episodes", type=int, default=10, help="Number of episodes to record"
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        default=False,
        help="Accepted for CRISP CLI compatibility; existing LeRobot datasets are resumed automatically",
    )
    parser.add_argument(
        "--push-to-hub",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Whether to push the recorded dataset to the Hugging Face Hub",
    )
    parser.add_argument(
        "--recording-manager-type",
        type=str,
        default="keyboard",
        choices=["keyboard"],
        help="Recording manager to use",
    )
    parser.add_argument(
        "--joint-control",
        action="store_true",
        help="Accepted for CRISP CLI compatibility; Polymetis script uses joint targets",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logger level",
    )
    parser.add_argument(
        "--path",
        type=str,
        default=None,
        help="Path to pretrained_model or a train output directory containing checkpoints/*/pretrained_model",
    )
    parser.add_argument(
        "--env-config",
        type=str,
        default=None,
        help="Accepted for CRISP CLI compatibility; unused for Polymetis",
    )
    parser.add_argument(
        "--policy-config",
        type=str,
        default=None,
        help="Accepted for CRISP CLI compatibility; LeRobot reads config from --path",
    )
    parser.add_argument(
        "--env-namespace",
        type=str,
        default=None,
        help="Accepted for CRISP CLI compatibility; unused for Polymetis",
    )
    parser.add_argument(
        "--evaluate",
        action="store_true",
        default=False,
        help="Evaluate success/failure after each episode",
    )
    parser.add_argument(
        "--lerobot-root",
        type=str,
        default="~/lerobot_data",
        help="Root path for the recorded LeRobot dataset",
    )
    parser.add_argument(
        "--policy-dataset-root",
        type=str,
        default=None,
        help="Dataset root whose metadata should be used to load the policy; defaults to --lerobot-root",
    )
    parser.add_argument(
        "--policy-repo-id",
        type=str,
        default=None,
        help="Dataset repo_id whose metadata should be used to load the policy; defaults to --repo-id",
    )
    parser.add_argument(
        "--robot-backend",
        type=str,
        choices=["zmq", "polymetis"],
        default="zmq",
        help="Robot connection to use. Use 'zmq' on the HPC and 'polymetis' only on the realtime laptop.",
    )
    parser.add_argument(
        "--robot-host",
        type=str,
        default="127.0.0.1",
        help="Host for the ZMQ robot server or SSH tunnel endpoint",
    )
    parser.add_argument(
        "--robot-port", type=int, default=6001, help="ZMQ robot server port"
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default="100.97.47.74",
        help="Polymetis robot server IP address; only used with --robot-backend polymetis",
    )
    parser.add_argument(
        "--smolvla",
        action="store_true",
        default=False,
        help="Enable SmolVLA runtime preprocessing: wrist->camera1 and zero camera2/camera3.",
    )
    parser.add_argument(
        "--camera-host",
        type=str,
        default="127.0.0.1",
        help="Host for ZMQ camera servers",
    )
    parser.add_argument(
        "--wrist-camera-port", type=int, default=5000, help="ZMQ wrist camera port"
    )
    parser.add_argument(
        "--base-camera-port", type=int, default=5001, help="ZMQ base camera port"
    )
    parser.add_argument(
        "--cameras",
        type=parse_cameras,
        default=("wrist",),
        help="Comma-separated cameras to record/use, e.g. wrist or wrist,base",
    )
    parser.add_argument(
        "--task",
        type=str,
        default="Pick up the lego block.",
        help="Task description stored in LeRobot frames",
    )
    parser.add_argument(
        "--device", type=str, default=None, help="Torch device for policy inference"
    )
    parser.add_argument(
        "--expected-state-dim",
        type=int,
        default=8,
        help="Expected policy state dimension",
    )
    parser.add_argument(
        "--expected-image-height",
        type=int,
        default=480,
        help="Expected RGB image height",
    )
    parser.add_argument(
        "--expected-image-width", type=int, default=640, help="Expected RGB image width"
    )
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.015,
        help="Max joint movement per frame",
    )
    parser.add_argument(
        "--max-gripper-delta",
        type=float,
        default=0.03,
        help="Max gripper movement per frame",
    )
    parser.add_argument(
        "--action-mode",
        type=str,
        default="absolute_joint_position",
        choices=["absolute_joint_position", "delta_joint_position"],
        help="Interpretation of policy actions",
    )
    parser.add_argument(
        "--execute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Send safe policy targets to Polymetis; use --no-execute for dry-run recording",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    LOGGER.info("%s", "-" * 40)
    LOGGER.info("Arguments:")
    for arg, value in vars(args).items():
        LOGGER.info("  %-30s: %s", arg, value)
    LOGGER.info("%s", "-" * 40)

    if args.repo_id is None:
        args.repo_id = prompt_value(
            "Please enter the repository ID for the recorded dataset (e.g. username/dataset_name):"
        )
    if args.path is None:
        LOGGER.info("No path provided. Searching for models in 'outputs/train'.")
        args.path = find_pretrained_model()
    args.path = resolve_pretrained_path(args.path)

    policy_repo_id = args.policy_repo_id or args.repo_id
    policy_dataset_root = args.policy_dataset_root or args.lerobot_root

    evaluation_file = "evaluation_results.csv"
    if args.evaluate:
        datetime_now = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = (
            f"evaluation_results_{args.path.replace('/', '_')}_{datetime_now}"
        )
        evaluation_file = prompt_value(
            "Please enter the output file for evaluation results",
            default=default_name,
        )
        if not evaluation_file.endswith(".csv"):
            evaluation_file += ".csv"

    env: Optional[RobotEnv] = None
    writer: Optional[LeRobotDatasetWriter] = None
    bundle = None
    try:
        LOGGER.info("Loading policy.")
        bundle = load_policy_for_args(args, policy_dataset_root, policy_repo_id)
        batch_transform, action_transform = make_policy_processors(
            args, bundle.policy, bundle.device
        )
        adapter = LeRobotObservationAdapter(
            device=bundle.device,
            camera_keys=args.cameras,
            expected_state_dim=args.expected_state_dim,
            expected_image_shape=(
                args.expected_image_height,
                args.expected_image_width,
                3,
            ),
        )
        if not args.smolvla:
            validate_policy_batch_keys(adapter, bundle.dataset.meta)
        executor = SafeJointActionExecutor(
            SafetyConfig(
                max_joint_delta=args.max_joint_delta,
                max_gripper_delta=args.max_gripper_delta,
                action_mode=args.action_mode,
            )
        )

        LOGGER.info(
            "Connecting to robot backend %s (robot_host=%s, robot_port=%s, robot_ip=%s).",
            args.robot_backend,
            args.robot_host,
            args.robot_port,
            args.robot_ip,
        )
        robot = make_robot_client(args)
        env = RobotEnv(
            robot,
            control_rate_hz=args.fps,
            camera_dict=make_camera_clients(
                camera_host=args.camera_host,
                wrist_camera_port=args.wrist_camera_port,
                base_camera_port=args.base_camera_port,
                cameras=args.cameras,
            ),
        )

        LOGGER.info("Preparing LeRobot dataset writer.")
        writer = LeRobotDatasetWriter(
            root=args.lerobot_root,
            repo_id=args.repo_id,
            fps=args.fps,
            task=args.task,
            robot_type=args.robot_type,
            camera_keys=args.cameras,
        )

        evaluator = CsvEpisodeEvaluator(output_file="eval/" + evaluation_file)
        recording_manager = KeyboardEpisodeRecorder(
            writer=writer,
            num_episodes=args.num_episodes,
            fps=args.fps,
        )
        recording_manager.wait_until_ready()

        LOGGER.info("Homing robot before starting with recording.")
        home_robot(env)

        def infer_and_step() -> Tuple[Dict[str, Any], np.ndarray]:
            obs = env.get_obs()
            batch = adapter.make_batch(obs)
            state = adapter.state_from_obs(obs)
            with torch.no_grad():
                batch = batch_transform(batch)
                policy_action = bundle.policy.select_action(batch)
                policy_action = action_transform(policy_action)
            safe = executor.make_safe_target(policy_action, state)
            if args.execute:
                env.step(safe.target)
            return obs, safe.target

        def on_start() -> None:
            reset_policy(bundle.policy)
            evaluator.start_timer()

        def on_end() -> None:
            LOGGER.info("Homing robot after episode.")
            home_robot(env)
            LOGGER.info(
                "Waiting for user to decide on success/failure if evaluating..."
            )
            evaluator.evaluate(episode=recording_manager.episode_count)

        with evaluator.start_eval(overwrite=True, activate=args.evaluate):
            with recording_manager:
                while not recording_manager.done():
                    LOGGER.info(
                        "→ Episode %s / %s",
                        recording_manager.episode_count + 1,
                        recording_manager.num_episodes,
                    )
                    recording_manager.record_episode(
                        frame_fn=infer_and_step,
                        on_start=on_start,
                        on_end=on_end,
                    )
                    LOGGER.info("Episode finished.")

        if (
            args.push_to_hub
            and writer is not None
            and hasattr(writer.dataset, "push_to_hub")
        ):
            LOGGER.info("Pushing dataset to the Hugging Face Hub.")
            writer.dataset.push_to_hub()

        LOGGER.info("Finished recording.")
    except Exception:
        LOGGER.exception("Polymetis policy deployment failed.")
        raise
    finally:
        if env is not None:
            LOGGER.info("Closing Polymetis environment.")
            close_robot(env)


if __name__ == "__main__":
    main()
