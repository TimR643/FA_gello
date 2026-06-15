"""Safely run a trained LeRobot policy on the real GELLO/ZMQ robot stack.

Default mode is a dry run: live observations are read and the policy is queried,
but no command is sent to the robot unless ``--execute`` is passed.  The script
supports ACT policies with dataset camera names directly and SmolVLA policies
with optional camera1/camera2/camera3 runtime remapping.
"""

from __future__ import annotations

import copy
import time
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Mapping, Optional, Tuple

import numpy as np
import torch
import tyro
from lerobot.policies import make_pre_post_processors

from gello.env import RobotEnv
from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    PolicyBundle,
    SafetyConfig,
    diagnose_action_state_interpretation,
    summarize_rgb_image,
)
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    checkpoint: str
    dataset_root: str
    repo_id: str

    robot_host: str = "127.0.0.1"
    camera_host: Optional[str] = None
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    cameras: Tuple[str, ...] = ("wrist",)
    policy_kind: str = "auto"
    smolvla_image_keys: Tuple[str, ...] = ("camera1", "camera2", "camera3")
    smolvla_empty_camera_keys: Tuple[str, ...] = ("empty_camera_0", "empty_camera_1")

    duration: float = 10.0
    hz: float = 2.0
    execute: bool = False
    require_enter: bool = True
    device: Optional[str] = None

    expected_state_dim: int = 8
    expected_image_height: int = 480
    expected_image_width: int = 640

    max_joint_delta: float = 0.005
    max_gripper_delta: float = 0.01
    action_mode: str = "absolute_joint_position"
    print_action_state_diagnostics: bool = True
    print_camera_color_diagnostics: bool = False
    camera_color_margin: float = 25.0
    camera_color_min_fraction: float = 0.002
    debug_image_dir: Optional[str] = None
    debug_save_image_every_n_steps: int = 20
    debug_counterfactual_tasks: Tuple[str, ...] = ()
    reset_policy_every_step: bool = False
    zero_live_cameras: Tuple[str, ...] = ()

    task: str = "Move right when the red block is visible, otherwise move left."


def _per_step_delta_rate(delta: float, hz: float) -> float:
    """Return the implied per-second joint/gripper delta at the control rate."""

    return float(delta * hz)


def _print_realtime_safety_warnings(args: Args) -> None:
    """Print warnings for rollout limits that are likely to break realtime control."""

    joint_rate = _per_step_delta_rate(args.max_joint_delta, args.hz)
    gripper_rate = _per_step_delta_rate(args.max_gripper_delta, args.hz)
    print("implied_max_joint_delta_per_second:", round(joint_rate, 4))
    print("implied_max_gripper_delta_per_second:", round(gripper_rate, 4))
    if args.execute and args.action_mode == "absolute_joint_position" and joint_rate > 0.25:
        print(
            "WARNING: max_joint_delta * hz is very high for realtime Franka "
            "execution. Large absolute-target jumps can trigger realtime "
            "breaks/reflex stops. Start near the training pose and reduce "
            "--max-joint-delta, e.g. 0.005-0.02 at 10 Hz."
        )



def _print_policy_queue_warnings(args: Args, *, use_smolvla_runtime: bool) -> None:
    """Warn about debug options that can make chunked policies repeat first actions."""

    if use_smolvla_runtime and args.execute and args.reset_policy_every_step:
        print(
            "WARNING: --reset-policy-every-step is a debugging option for "
            "chunked policies. In execute mode it repeatedly discards the "
            "policy action queue, so the robot may execute the first predicted "
            "chunk action over and over instead of following the planned chunk. "
            "Remove this flag for normal SmolVLA rollouts."
        )

def _make_camera_clients(args: Args) -> dict[str, ZMQClientCamera]:
    host = args.camera_host or args.robot_host
    clients: dict[str, ZMQClientCamera] = {}

    for camera in args.cameras:
        if camera == "wrist":
            clients[camera] = ZMQClientCamera(port=args.wrist_camera_port, host=host)
        elif camera == "base":
            clients[camera] = ZMQClientCamera(port=args.base_camera_port, host=host)
        else:
            raise ValueError(
                f"Unsupported camera {camera!r}; expected 'wrist' or 'base'."
            )

    return clients


def _metadata_mapping(dataset_meta: Any, name: str) -> Any:
    value = getattr(dataset_meta, name, None)
    if value is None and isinstance(dataset_meta, Mapping):
        value = dataset_meta.get(name)
    return value


def _copy_meta_with_updates(dataset_meta: Any, features: Any, stats: Any) -> Any:
    try:
        patched = copy.copy(dataset_meta)
        setattr(patched, "features", features)
        if stats is not None:
            setattr(patched, "stats", stats)
        return patched
    except Exception:
        attrs = dict(getattr(dataset_meta, "__dict__", {}))
        attrs["features"] = features
        if stats is not None:
            attrs["stats"] = stats
        return SimpleNamespace(**attrs)


def _make_smolvla_dataset_meta(
    dataset_meta: Any, cameras: Tuple[str, ...], smolvla_image_keys: Tuple[str, ...]
) -> Any:
    """Patch dataset metadata to match runtime SmolVLA camera renaming.

    Training commonly uses LeRobot's ``--rename_map`` to expose dataset cameras
    such as ``wrist`` and ``base`` as SmolVLA keys such as ``camera1`` and
    ``camera2``.  At deployment time this script performs the same renaming on
    live batches; this helper mirrors it for the dataset metadata passed to
    ``make_policy`` so LeRobot's policy/dataset feature validation sees the
    trained feature names.
    """

    features = _metadata_mapping(dataset_meta, "features")
    if features is None:
        return dataset_meta

    patched_features = copy.deepcopy(dict(features))
    feature_source = None

    for camera, smolvla_key in zip(cameras, smolvla_image_keys):
        source_key = f"observation.images.{camera}"
        target_key = f"observation.images.{smolvla_key}"
        if source_key in patched_features:
            source_feature = copy.deepcopy(patched_features.pop(source_key))
            patched_features[target_key] = source_feature
            feature_source = copy.deepcopy(source_feature)
        elif target_key in patched_features:
            feature_source = copy.deepcopy(patched_features[target_key])

    if feature_source is None:
        for key, value in patched_features.items():
            if key.startswith("observation.images."):
                feature_source = copy.deepcopy(value)
                break

    if feature_source is not None:
        for smolvla_key in smolvla_image_keys:
            target_key = f"observation.images.{smolvla_key}"
            patched_features.setdefault(target_key, copy.deepcopy(feature_source))

    stats = _metadata_mapping(dataset_meta, "stats")
    patched_stats = None
    if stats is not None:
        patched_stats = copy.deepcopy(dict(stats))
        stat_source = None
        for camera, smolvla_key in zip(cameras, smolvla_image_keys):
            source_key = f"observation.images.{camera}"
            target_key = f"observation.images.{smolvla_key}"
            if source_key in patched_stats:
                source_stat = copy.deepcopy(patched_stats.pop(source_key))
                patched_stats[target_key] = source_stat
                stat_source = copy.deepcopy(source_stat)
            elif target_key in patched_stats:
                stat_source = copy.deepcopy(patched_stats[target_key])

        if stat_source is not None:
            for smolvla_key in smolvla_image_keys:
                target_key = f"observation.images.{smolvla_key}"
                patched_stats.setdefault(target_key, copy.deepcopy(stat_source))

    return _copy_meta_with_updates(dataset_meta, patched_features, patched_stats)


def _policy_type_name(cfg: Any) -> str:
    """Return a lowercase LeRobot policy type/name from config metadata."""

    for attr in ("type", "policy_type", "name"):
        value = getattr(cfg, attr, None)
        if value:
            return str(value).lower()
    return cfg.__class__.__name__.lower()


def _load_policy(
    *,
    checkpoint: str,
    dataset_root: str,
    repo_id: str,
    cameras: Tuple[str, ...],
    smolvla_image_keys: Tuple[str, ...],
    smolvla_empty_camera_keys: Tuple[str, ...],
    force_smolvla_mapping: bool = False,
    device: Optional[str] = None,
) -> PolicyBundle:
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.factory import make_policy

    resolved_device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    dataset = LeRobotDataset(repo_id=repo_id, root=dataset_root)
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = resolved_device
    policy_type = _policy_type_name(cfg)
    use_smolvla_mapping = force_smolvla_mapping or policy_type == "smolvla"
    smolvla_feature_keys = smolvla_image_keys + smolvla_empty_camera_keys
    policy_meta = (
        _make_smolvla_dataset_meta(
            dataset.meta, cameras=cameras, smolvla_image_keys=smolvla_feature_keys
        )
        if use_smolvla_mapping
        else dataset.meta
    )
    policy = make_policy(cfg=cfg, ds_meta=policy_meta)
    policy.to(resolved_device)
    policy.eval()
    return PolicyBundle(policy=policy, dataset=dataset, device=resolved_device)


def _prepare_smolvla_batch(
    batch: dict,
    task: str,
    cameras: Tuple[str, ...],
    smolvla_image_keys: Tuple[str, ...],
    smolvla_empty_camera_keys: Tuple[str, ...] = (),
) -> dict:
    """Prepare live GELLO/LeRobot batch for SmolVLA inference.

    The live robot adapter creates keys such as:
        observation.images.wrist
        observation.images.base

    SmolVLA trainings in this repository commonly use generic camera keys such
    as camera1/camera2/camera3. Runtime mapping is positional: the cameras
    passed via ``--cameras`` are mapped, in insertion order, to
    ``--smolvla-image-keys``. Missing SmolVLA image keys are filled with black
    dummy images so checkpoints trained with empty cameras can still run.
    """

    batch["task"] = [task]
    batch["robot_type"] = [""]

    live_image_keys = [f"observation.images.{camera}" for camera in cameras]
    missing_live_keys = [key for key in live_image_keys if key not in batch]
    if missing_live_keys:
        raise KeyError(
            "Missing live image observations for SmolVLA inference: "
            f"{missing_live_keys}. Available keys: {list(batch.keys())}"
        )

    first_image = batch[live_image_keys[0]]
    for camera, smolvla_key in zip(cameras, smolvla_image_keys):
        live_key = f"observation.images.{camera}"
        target_key = f"observation.images.{smolvla_key}"
        if target_key not in batch:
            batch[target_key] = batch[live_key]
        if live_key != target_key:
            batch.pop(live_key, None)

    for smolvla_key in smolvla_image_keys + smolvla_empty_camera_keys:
        target_key = f"observation.images.{smolvla_key}"
        if target_key not in batch:
            batch[target_key] = torch.zeros_like(first_image)

    return batch




def _reset_policy(policy: Any) -> None:
    """Reset policy-side action queues/state when the policy supports it."""

    if hasattr(policy, "reset"):
        policy.reset()

def _write_rgb_ppm(path: Path, image: np.ndarray) -> None:
    """Write an RGB image as binary PPM without extra image dependencies."""

    img = np.asarray(image, dtype=np.uint8)
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 RGB image, got shape {img.shape}")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"P6\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii")
    with path.open("wb") as handle:
        handle.write(header)
        handle.write(np.ascontiguousarray(img).tobytes())

def main(args: Args) -> None:
    bundle = _load_policy(
        checkpoint=args.checkpoint,
        dataset_root=args.dataset_root,
        repo_id=args.repo_id,
        cameras=args.cameras,
        smolvla_image_keys=args.smolvla_image_keys,
        smolvla_empty_camera_keys=args.smolvla_empty_camera_keys,
        force_smolvla_mapping=args.policy_kind == "smolvla",
        device=args.device,
    )

    policy_type = _policy_type_name(bundle.policy.config)
    if args.policy_kind not in {"auto", "act", "smolvla"}:
        raise ValueError("policy_kind must be one of: auto, act, smolvla")
    effective_policy_kind = policy_type if args.policy_kind == "auto" else args.policy_kind
    use_smolvla_runtime = effective_policy_kind == "smolvla"

    preprocess = postprocess = None
    if use_smolvla_runtime:
        preprocess, postprocess = make_pre_post_processors(
            bundle.policy.config,
            args.checkpoint,
            preprocessor_overrides={"device_processor": {"device": str(bundle.device)}},
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

    safety = SafetyConfig(
        max_joint_delta=args.max_joint_delta,
        max_gripper_delta=args.max_gripper_delta,
        action_mode=args.action_mode,
    )
    executor = SafeJointActionExecutor(safety)

    robot = ZMQClientRobot(port=args.robot_port, host=args.robot_host)

    env = RobotEnv(
        robot,
        control_rate_hz=args.hz,
        camera_dict=_make_camera_clients(args),
    )

    print(f"\nLEROBOT {effective_policy_kind.upper()} REAL-ROBOT ROLLOUT")
    print("checkpoint:", args.checkpoint)
    print("dataset_root:", args.dataset_root)
    print("repo_id:", args.repo_id)
    print("device:", bundle.device)
    print("live cameras:", args.cameras)
    print("policy_type:", policy_type)
    print("policy_kind:", effective_policy_kind)
    print("SmolVLA image keys:", args.smolvla_image_keys)
    print("SmolVLA empty camera keys:", args.smolvla_empty_camera_keys)
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("action_mode:", args.action_mode)
    print("max_joint_delta:", args.max_joint_delta)
    print("max_gripper_delta:", args.max_gripper_delta)
    _print_realtime_safety_warnings(args)
    _print_policy_queue_warnings(args, use_smolvla_runtime=use_smolvla_runtime)
    print("print_action_state_diagnostics:", args.print_action_state_diagnostics)
    print("print_camera_color_diagnostics:", args.print_camera_color_diagnostics)
    print("camera_color_margin:", args.camera_color_margin)
    print("camera_color_min_fraction:", args.camera_color_min_fraction)
    print("debug_image_dir:", args.debug_image_dir)
    print("debug_save_image_every_n_steps:", args.debug_save_image_every_n_steps)
    print("debug_counterfactual_tasks:", args.debug_counterfactual_tasks)
    print("reset_policy_every_step:", args.reset_policy_every_step)
    print("zero_live_cameras:", args.zero_live_cameras)
    print("task:", args.task)

    if use_smolvla_runtime:
        print("\nRuntime image mapping:")
        for live_camera, smolvla_key in zip(args.cameras, args.smolvla_image_keys):
            print(f"  observation.images.{live_camera} -> observation.images.{smolvla_key}")
        zero_smolvla_keys = (
            args.smolvla_image_keys[len(args.cameras):]
            + args.smolvla_empty_camera_keys
        )
        for smolvla_key in zero_smolvla_keys:
            print(f"  observation.images.{smolvla_key} -> zeros_like(first live camera)")
    else:
        print("\nRuntime image mapping: using dataset camera names directly")

    if not args.execute:
        print("\nDRY RUN: policy will be evaluated, but the robot will not move.")
    else:
        print("\nEXECUTE MODE: the real robot can move.")
        print("Keep your hand on the enabling switch / emergency stop.")
        print("Start with an empty workspace and conservative deltas.")

    if args.require_enter:
        input("\nPress ENTER to start...")

    steps = int(args.duration * args.hz)
    if steps <= 0:
        raise ValueError("duration * hz must produce at least one step.")

    dt = 1.0 / args.hz
    _reset_policy(bundle.policy)

    for step in range(steps):
        started = time.time()

        obs = env.get_obs()
        batch = adapter.make_batch(obs)
        for camera in args.zero_live_cameras:
            key = f"observation.images.{camera}"
            if key not in batch:
                raise KeyError(
                    f"Cannot zero missing live camera {camera!r}; "
                    f"available batch keys: {list(batch.keys())}"
                )
            batch[key] = torch.zeros_like(batch[key])
        state = adapter.state_from_obs(obs)

        if args.reset_policy_every_step:
            _reset_policy(bundle.policy)

        with torch.no_grad():
            if use_smolvla_runtime:
                batch = _prepare_smolvla_batch(
                    batch,
                    args.task,
                    args.cameras,
                    args.smolvla_image_keys,
                    args.smolvla_empty_camera_keys,
                )
                batch = preprocess(batch)

            policy_action = bundle.policy.select_action(batch)
            if use_smolvla_runtime:
                policy_action = postprocess(policy_action)

        safe = executor.make_safe_target(policy_action, state)
        diagnostics = diagnose_action_state_interpretation(policy_action, state)
        counterfactual_results = []
        if args.debug_counterfactual_tasks:
            with torch.no_grad():
                for counterfactual_task in args.debug_counterfactual_tasks:
                    _reset_policy(bundle.policy)
                    cf_batch = adapter.make_batch(obs)
                    for camera in args.zero_live_cameras:
                        key = f"observation.images.{camera}"
                        cf_batch[key] = torch.zeros_like(cf_batch[key])
                    if use_smolvla_runtime:
                        cf_batch = _prepare_smolvla_batch(
                            cf_batch,
                            counterfactual_task,
                            args.cameras,
                            args.smolvla_image_keys,
                            args.smolvla_empty_camera_keys,
                        )
                        cf_batch = preprocess(cf_batch)
                    cf_action = bundle.policy.select_action(cf_batch)
                    if use_smolvla_runtime:
                        cf_action = postprocess(cf_action)
                    cf_safe = executor.make_safe_target(cf_action, state)
                    counterfactual_results.append((counterfactual_task, cf_safe))
                _reset_policy(bundle.policy)

        print(f"\nStep {step + 1}/{steps}")
        print("state         :", np.round(state, 3))
        print("policy_action :", np.round(safe.policy_action, 3))
        print("raw_delta     :", np.round(safe.raw_delta, 3))
        print("clipped_delta :", np.round(safe.clipped_delta, 3))
        print("target        :", np.round(safe.target, 3))
        if args.print_camera_color_diagnostics:
            for camera in args.cameras:
                summary = summarize_rgb_image(
                    obs[f"{camera}_rgb"], dominance_margin=args.camera_color_margin
                )
                red_visible = summary.red_fraction >= args.camera_color_min_fraction
                green_visible = summary.green_fraction >= args.camera_color_min_fraction
                if red_visible and green_visible:
                    color_guess = "mixed"
                elif red_visible:
                    color_guess = "red"
                elif green_visible:
                    color_guess = "green"
                else:
                    color_guess = "none"
                print(
                    f"camera {camera:>5s} : "
                    f"rgb_mean=({summary.red_mean:.1f}, "
                    f"{summary.green_mean:.1f}, {summary.blue_mean:.1f}) "
                    f"red_dom={summary.red_dominance:.1f} "
                    f"green_dom={summary.green_dominance:.1f} "
                    f"red_px={100.0 * summary.red_fraction:.2f}% "
                    f"green_px={100.0 * summary.green_fraction:.2f}% "
                    f"guess={color_guess}"
                )

        if (
            args.debug_image_dir
            and args.debug_save_image_every_n_steps > 0
            and step % args.debug_save_image_every_n_steps == 0
        ):
            debug_dir = Path(args.debug_image_dir)
            for camera in args.cameras:
                _write_rgb_ppm(
                    debug_dir / f"step_{step + 1:04d}_{camera}.ppm",
                    obs[f"{camera}_rgb"],
                )
        if args.print_action_state_diagnostics:
            print("abs_delta_l2  :", round(diagnostics.absolute_delta_l2, 3))
            print("action_l2     :", round(diagnostics.action_l2, 3))
            print("delta_target  :", np.round(diagnostics.delta_target, 3))
            if diagnostics.likely_delta_action and args.action_mode == "absolute_joint_position":
                print(
                    "WARNING      : policy output is small while absolute delta is large; "
                    "this looks more like delta_joint_position than absolute_joint_position."
                )

        for counterfactual_task, cf_safe in counterfactual_results:
            print("counterfactual:", counterfactual_task)
            print("  cf_action   :", np.round(cf_safe.policy_action, 3))
            print("  cf_raw_delta:", np.round(cf_safe.raw_delta, 3))
            print("  cf_target   :", np.round(cf_safe.target, 3))

        if args.execute:
            env.step(safe.target)

        remaining = dt - (time.time() - started)
        if remaining > 0:
            time.sleep(remaining)

    print("\nFinished LeRobot SmolVLA real-robot rollout.")


if __name__ == "__main__":
    main(tyro.cli(Args))
