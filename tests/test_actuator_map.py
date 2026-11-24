# SPDX-FileCopyrightText: 2026 Quack Robotics contributors
# SPDX-License-Identifier: Apache-2.0
"""Name->index actuator map test for MuJoCoHal.

Asserts that the vendored Open Duck Mini v2 robot.xml (pinned at upstream
b23317a) exposes exactly the 16 actuators MuJoCoHal expects, in the
expected order. The HAL maps by name; this test catches any upstream
reordering or renaming before it reaches the control loop.

Needs the meshes: run scripts/fetch_upstream_meshes.py first.
"""
import hashlib
import pathlib
import unittest

import mujoco

ROOT = pathlib.Path(__file__).resolve().parent.parent
XML = ROOT / "third_party" / "open_duck_mini_v2" / "robot.xml"
PINNED_SHA256 = "a134222e3b789f491b5a4a4781c6d19c91c86ad4d8481795b6b03f5eab6e0c7a"

# Expected actuator table (index -> name), upstream order.
ACTUATORS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "left_antenna", "right_antenna",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
ANTENNAS = ("left_antenna", "right_antenna")  # SG90 on PWM, not on the policy output
POLICY_JOINTS = tuple(name for name in ACTUATORS if name not in ANTENNAS)  # 14 -> serial bus


class TestUpstreamActuatorMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(XML))

    def test_file_is_pinned_upstream(self):
        self.assertEqual(hashlib.sha256(XML.read_bytes()).hexdigest(), PINNED_SHA256)

    def test_nu_is_16(self):
        self.assertEqual(self.model.nu, 16)

    def test_actuator_order_matches_table(self):
        for index, name in enumerate(ACTUATORS):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.assertEqual(actuator_id, index, f"{name}: expected index {index}, got {actuator_id}")

    def test_policy_and_antenna_split(self):
        self.assertEqual(len(POLICY_JOINTS), 14)
        self.assertEqual(len(ANTENNAS), 2)
        self.assertEqual(set(POLICY_JOINTS) | set(ANTENNAS), set(ACTUATORS))
