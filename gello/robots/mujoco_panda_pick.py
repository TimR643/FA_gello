"""MuJoCo Franka-style pick simulation for pipeline dry-runs.

This module intentionally implements the same 8-DoF robot and wrist-camera
interfaces used by the real Panda/GELLO stack, while keeping the scene fully
self-contained.  It is not a high-fidelity Franka model; it is a deterministic
pipeline simulator for validating observations, actions, cameras, and LeRobot
recording before running on hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from gello.cameras.camera import CameraDriver
from gello.robots.robot import Robot

PANDA_SIM_HOME = np.array(
    [0.0, -0.55, 0.0, -1.95, 0.0, 1.55, 0.78, 0.08], dtype=np.float32
)
PANDA_SIM_PREGRASP = np.array(
    [0.0, -0.72, 0.0, -2.16, 0.0, 1.47, 0.78, 0.08], dtype=np.float32
)
PANDA_SIM_GRASP = np.array(
    [0.0, -0.91, 0.0, -2.31, 0.0, 1.37, 0.78, 0.08], dtype=np.float32
)
PANDA_SIM_LIFT = np.array(
    [0.0, -0.47, 0.0, -1.70, 0.0, 1.28, 0.78, 0.0], dtype=np.float32
)
PANDA_SIM_PLACE = np.array(
    [0.48, -0.50, -0.16, -1.76, 0.04, 1.35, 0.62, 0.0], dtype=np.float32
)


@dataclass(frozen=True)
class MujocoPickSceneConfig:
    """Configuration for the deterministic MuJoCo pick scene."""

    image_height: int = 480
    image_width: int = 640
    control_timestep: float = 0.002
    physics_substeps: int = 10
    render_depth: bool = True
    xml_dump_path: Optional[str] = "mujoco_panda_pick_scene.xml"


class MujocoPandaPickRobot(Robot):
    """Franka-like 8-DoF MuJoCo robot with a cube/table pick scene."""

    def __init__(self, config: Optional[MujocoPickSceneConfig] = None):
        self.config = config or MujocoPickSceneConfig()

        import mujoco

        self._mujoco = mujoco
        xml = self._build_xml()
        if self.config.xml_dump_path:
            Path(self.config.xml_dump_path).write_text(xml)

        self._model = mujoco.MjModel.from_xml_string(xml)
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(
            self._model,
            height=self.config.image_height,
            width=self.config.image_width,
        )
        self._num_dofs = 8
        self._joint_cmd = PANDA_SIM_HOME.astype(np.float64).copy()
        self._grasped = False
        self._cube_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "cube"
        )
        self._cube_joint_qpos_addr = self._model.jnt_qposadr[
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
        ]
        self._wrist_site_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SITE, "wrist_camera_mount"
        )
        self._ee_site_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SITE, "pinch_site"
        )
        self._wrist_camera_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist"
        )
        self.reset()

    def reset(self) -> None:
        self._mujoco.mj_resetData(self._model, self._data)
        self._data.qpos[:7] = PANDA_SIM_HOME[:7]
        self._data.qpos[7] = PANDA_SIM_HOME[7]
        # The single gripper command is mirrored into two visual fingers through
        # the actuator control value; qpos[7] stores the deployment gripper state.
        self._data.ctrl[: self._num_dofs] = self._joint_cmd
        self._data.qpos[self._cube_joint_qpos_addr : self._cube_joint_qpos_addr + 7] = (
            np.array([0.50, 0.0, 0.825, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        )
        self._mujoco.mj_forward(self._model, self._data)
        self._grasped = False

    def num_dofs(self) -> int:
        return self._num_dofs

    def get_joint_state(self) -> np.ndarray:
        return self._joint_positions()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        joint_state = np.asarray(joint_state, dtype=np.float64)
        if joint_state.shape != (self._num_dofs,):
            raise ValueError(f"Expected 8-DoF command, got shape {joint_state.shape}")
        self._joint_cmd = joint_state.copy()
        self._data.ctrl[: self._num_dofs] = self._joint_cmd
        for _ in range(self.config.physics_substeps):
            self._maybe_attach_or_release_cube()
            if self._grasped:
                self._move_cube_with_gripper()
            self._mujoco.mj_step(self._model, self._data)
        self._mujoco.mj_forward(self._model, self._data)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self._joint_positions()
        joint_velocities = np.zeros(self._num_dofs, dtype=np.float32)
        joint_velocities[:7] = self._data.qvel[:7].astype(np.float32)

        ee_pos = self._data.site_xpos[self._ee_site_id].copy()
        ee_quat = np.zeros(4, dtype=np.float64)
        self._mujoco.mju_mat2Quat(ee_quat, self._data.site_xmat[self._ee_site_id])
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]).astype(np.float32),
            "gripper_position": np.float32(joint_positions[-1]),
            "cube_position": self._data.xpos[self._cube_body_id].astype(np.float32),
            "cube_grasped": np.asarray([self._grasped], dtype=bool),
        }

    def render_wrist(self) -> Tuple[np.ndarray, np.ndarray]:
        self._renderer.update_scene(self._data, camera=self._wrist_camera_id)
        rgb = self._renderer.render().astype(np.uint8)
        if self.config.render_depth:
            self._renderer.enable_depth_rendering()
            depth = self._renderer.render().astype(np.float32)
            self._renderer.disable_depth_rendering()
            depth = (depth * 1000.0).astype(np.uint16)[:, :, None]
        else:
            depth = np.zeros(
                (self.config.image_height, self.config.image_width, 1), dtype=np.uint16
            )
        return rgb, depth

    def _joint_positions(self) -> np.ndarray:
        joints = np.zeros(self._num_dofs, dtype=np.float32)
        joints[:7] = self._data.qpos[:7].astype(np.float32)
        joints[7] = np.float32(self._joint_cmd[7])
        return joints

    def _maybe_attach_or_release_cube(self) -> None:
        gripper_closed = self._joint_cmd[7] < 0.025
        cube_pos = self._data.xpos[self._cube_body_id]
        ee_pos = self._data.site_xpos[self._ee_site_id]
        if (
            not self._grasped
            and gripper_closed
            and np.linalg.norm(cube_pos - ee_pos) < 0.14
        ):
            self._grasped = True
        elif self._grasped and not gripper_closed:
            self._grasped = False

    def _move_cube_with_gripper(self) -> None:
        wrist_pos = self._data.site_xpos[self._wrist_site_id]
        target = wrist_pos + np.array([0.03, 0.0, -0.105], dtype=np.float64)
        qpos = self._data.qpos
        qvel = self._data.qvel
        qpos[self._cube_joint_qpos_addr : self._cube_joint_qpos_addr + 3] = target
        qpos[self._cube_joint_qpos_addr + 3 : self._cube_joint_qpos_addr + 7] = (
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        )
        qvel_addr = self._model.jnt_dofadr[
            self._mujoco.mj_name2id(
                self._model, self._mujoco.mjtObj.mjOBJ_JOINT, "cube_free"
            )
        ]
        qvel[qvel_addr : qvel_addr + 6] = 0.0

    def _build_xml(self) -> str:
        timestep = self.config.control_timestep
        return f"""
<mujoco model="pipeline_panda_pick">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="{self.config.image_width}" offheight="{self.config.image_height}"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.25 0.25" rgb2="0.35 0.35 0.35" width="512" height="512"/>
    <material name="floor_mat" texture="grid" texrepeat="4 4" reflectance="0.15"/>
    <material name="robot_mat" rgba="0.86 0.86 0.82 1"/>
    <material name="joint_mat" rgba="0.12 0.12 0.14 1"/>
    <material name="cube_mat" rgba="0.9 0.12 0.08 1"/>
    <material name="table_mat" rgba="0.55 0.37 0.22 1"/>
  </asset>
  <worldbody>
    <light name="key" pos="0 -2.5 3" dir="0 1 -1" diffuse="0.9 0.9 0.9"/>
    <light name="fill" pos="-2 1 2" dir="1 -0.3 -1" diffuse="0.35 0.35 0.35"/>
    <geom name="floor" type="plane" size="3 3 0.02" material="floor_mat"/>
    <geom name="table" type="box" pos="0.55 0 0.38" size="0.45 0.40 0.04" material="table_mat" friction="1 0.005 0.0001"/>
    <body name="cube" pos="0.50 0 0.825">
      <freejoint name="cube_free"/>
      <geom name="red_cube" type="box" size="0.03 0.03 0.03" material="cube_mat" mass="0.08" friction="1.5 0.01 0.0001"/>
    </body>
    <body name="panda_base" pos="0 0 0.42">
      <geom type="cylinder" size="0.10 0.035" material="joint_mat"/>
      <body name="link1" pos="0 0 0.08">
        <joint name="panda_joint1" type="hinge" axis="0 0 1" range="-2.8973 2.8973" damping="4"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.25" size="0.045" material="robot_mat"/>
        <body name="link2" pos="0 0 0.25">
          <joint name="panda_joint2" type="hinge" axis="0 1 0" range="-1.7628 1.7628" damping="4"/>
          <geom type="capsule" fromto="0 0 0 0.18 0 0.0" size="0.04" material="robot_mat"/>
          <body name="link3" pos="0.18 0 0">
            <joint name="panda_joint3" type="hinge" axis="0 0 1" range="-2.8973 2.8973" damping="3"/>
            <geom type="capsule" fromto="0 0 0 0.18 0 0" size="0.038" material="robot_mat"/>
            <body name="link4" pos="0.18 0 0">
              <joint name="panda_joint4" type="hinge" axis="0 1 0" range="-3.0718 -0.0698" damping="3"/>
              <geom type="capsule" fromto="0 0 0 0.18 0 0" size="0.036" material="robot_mat"/>
              <body name="link5" pos="0.18 0 0">
                <joint name="panda_joint5" type="hinge" axis="0 0 1" range="-2.8973 2.8973" damping="2"/>
                <geom type="capsule" fromto="0 0 0 0.16 0 0" size="0.034" material="robot_mat"/>
                <body name="link6" pos="0.16 0 0">
                  <joint name="panda_joint6" type="hinge" axis="0 1 0" range="-0.0175 3.7525" damping="2"/>
                  <geom type="capsule" fromto="0 0 0 0.12 0 0" size="0.032" material="robot_mat"/>
                  <body name="wrist" pos="0.12 0 0">
                    <joint name="panda_joint7" type="hinge" axis="1 0 0" range="-2.8973 2.8973" damping="1"/>
                    <geom type="sphere" size="0.045" material="joint_mat"/>
                    <site name="wrist_camera_mount" pos="0.02 0 0.035" size="0.01"/>
                    <camera name="wrist" mode="fixed" pos="0.015 0 0.055" xyaxes="0 -1 0 0 0 -1" fovy="70"/>
                    <body name="gripper" pos="0.08 0 0">
                      <joint name="gripper" type="slide" axis="0 1 0" range="0 0.08" damping="1"/>
                      <site name="pinch_site" pos="0.055 0 0" size="0.012" rgba="0 1 0 1"/>
                      <geom type="box" pos="0.025 0.045 0" size="0.055 0.008 0.018" material="joint_mat"/>
                      <geom type="box" pos="0.025 -0.045 0" size="0.055 0.008 0.018" material="joint_mat"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="act1" joint="panda_joint1" kp="250"/>
    <position name="act2" joint="panda_joint2" kp="250"/>
    <position name="act3" joint="panda_joint3" kp="220"/>
    <position name="act4" joint="panda_joint4" kp="220"/>
    <position name="act5" joint="panda_joint5" kp="160"/>
    <position name="act6" joint="panda_joint6" kp="160"/>
    <position name="act7" joint="panda_joint7" kp="120"/>
    <position name="act8" joint="gripper" kp="80"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0 -0.55 0 -1.95 0 1.55 0.78 0.08 0.50 0 0.825 1 0 0 0" ctrl="0 -0.55 0 -1.95 0 1.55 0.78 0.08"/>
  </keyframe>
</mujoco>
"""


class MujocoWristCamera(CameraDriver):
    """Camera driver exposing the simulated camera as ``wrist_rgb``/depth."""

    def __init__(self, robot: MujocoPandaPickRobot):
        self._robot = robot

    def read(
        self, img_size: Optional[Tuple[int, int]] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        image, depth = self._robot.render_wrist()
        if img_size is not None and image.shape[:2] != img_size:
            import cv2

            image = cv2.resize(image, (img_size[1], img_size[0]))
            depth = cv2.resize(depth[:, :, 0], (img_size[1], img_size[0]))[:, :, None]
        return image, depth


class HardcodedPandaPickAgent:
    """Deterministic replacement for GELLO teleoperation during simulation."""

    def __init__(self, steps_per_waypoint: int = 80):
        if steps_per_waypoint <= 0:
            raise ValueError("steps_per_waypoint must be positive")
        self._trajectory = self._make_trajectory(steps_per_waypoint)
        self._idx = 0

    @property
    def done(self) -> bool:
        return self._idx >= len(self._trajectory) - 1

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        del obs
        action = self._trajectory[min(self._idx, len(self._trajectory) - 1)]
        self._idx += 1
        return action.copy()

    def _make_trajectory(self, steps_per_waypoint: int) -> np.ndarray:
        waypoints = [
            PANDA_SIM_HOME,
            PANDA_SIM_PREGRASP,
            PANDA_SIM_GRASP,
            PANDA_SIM_GRASP.copy(),
            PANDA_SIM_LIFT,
            PANDA_SIM_PLACE,
        ]
        waypoints[3][-1] = 0.0
        pieces = []
        for start, stop in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(0.0, 1.0, steps_per_waypoint, endpoint=False):
                smooth = 0.5 - 0.5 * math.cos(math.pi * alpha)
                pieces.append((1.0 - smooth) * start + smooth * stop)
        pieces.append(waypoints[-1])
        return np.asarray(pieces, dtype=np.float32)
