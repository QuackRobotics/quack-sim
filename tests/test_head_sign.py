# SPDX-FileCopyrightText: 2026 Quack Robotics contributors
# SPDX-License-Identifier: Apache-2.0
"""Head joint sign convention test (forward kinematics only).

Pins the sign of the four head joints in the vendored Open Duck Mini v2
robot.xml (upstream b23317a), measured in the trunk frame (+x forward,
+z up) at the zero pose:
  neck_pitch +  -> nose up (and back)
  head_pitch +  -> nose up (and back)
  head_yaw   +  -> nose to +y (robot's left)
The HAL head clamp and the expression overlay (Luxo's "+pitch = nose up")
rely on this. If upstream flips an axis, this fails before the clamp does.

Needs the meshes: run scripts/fetch_upstream_meshes.py first.
"""
import pathlib
import unittest

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
XML = ROOT / "third_party" / "open_duck_mini_v2" / "robot.xml"
STEP_DEG = 5.0
MIN_MOVE_MM = 3.0  # observed ~8.5 mm for 5 deg; well above numerical noise


def head_points_trunk_mm(model, data, joint_deg):
    """All head mesh vertices (antennas excluded) in the trunk frame, mm."""
    data.qpos[:] = 0
    for name, deg in joint_deg.items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = np.radians(deg)
    mujoco.mj_kinematics(model, data)
    head = model.body("head_assembly").id
    trunk = model.body("trunk_assembly").id
    pts = []
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        b = model.geom_bodyid[g]
        if b != head:  # direct head geoms only: the shell carries the nose
            continue
        mid = model.geom_dataid[g]
        a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        pts.append(model.mesh_vert[a:a + n] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])
    p = np.vstack(pts)
    return (p - data.xpos[trunk]) @ data.xmat[trunk].reshape(3, 3) * 1000.0


class TestHeadSign(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(XML))
        cls.data = mujoco.MjData(cls.model)
        p0 = head_points_trunk_mm(cls.model, cls.data, {})
        mid = np.abs(p0[:, 1] - p0[:, 1].mean()) < 5.0
        cls.nose = np.where(mid)[0][p0[mid, 0].argmax()]  # frontmost centreline vertex
        cls.p0 = p0

    def delta(self, joint, deg):
        p = head_points_trunk_mm(self.model, self.data, {joint: deg})
        return p[self.nose] - self.p0[self.nose]

    def test_nose_is_forward(self):
        self.assertGreater(self.p0[self.nose, 0], self.p0[:, 0].mean())

    def test_neck_pitch_positive_is_nose_up(self):
        self.assertGreater(self.delta("neck_pitch", STEP_DEG)[2], MIN_MOVE_MM)
        self.assertLess(self.delta("neck_pitch", -STEP_DEG)[2], -MIN_MOVE_MM)

    def test_head_pitch_positive_is_nose_up(self):
        self.assertGreater(self.delta("head_pitch", STEP_DEG)[2], MIN_MOVE_MM)
        self.assertLess(self.delta("head_pitch", -STEP_DEG)[2], -MIN_MOVE_MM)

    def test_head_yaw_positive_is_nose_left(self):
        self.assertGreater(self.delta("head_yaw", STEP_DEG)[1], MIN_MOVE_MM)


if __name__ == "__main__":
    unittest.main()
