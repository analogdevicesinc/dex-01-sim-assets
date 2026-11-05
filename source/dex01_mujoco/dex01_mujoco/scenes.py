# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Scene pieces shared by the MuJoCo examples."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .hand_model import OUTPUT_DIR, build_hand_mjcf

# Spur gear built from convex primitives, so MuJoCo collides with its real outline (a single mesh
# would be reduced to its convex hull, a plain disk). Proportions approximate a medium Factory gear;
# NVIDIA's USD gear is not redistributable, so it is not used here.
GEAR_HUB_RADIUS_M = 0.02
GEAR_HALF_THICKNESS_M = 0.005
GEAR_TEETH = 16
GEAR_TOOTH_HALF_SIZE_M = (0.0035, 0.0022)  # radial, tangential

# Solver settings for the compliant contact the depth model needs: a short time constant keeps
# steady-state penetration sub-millimetre while staying above two physics steps.
CONTACT_SOLREF = "0.01 1"
CONTACT_SOLIMP = "0.95 0.99 0.0005"

OPTION_XML = f"""
  <option timestep="0.0041666667" integrator="implicitfast" cone="elliptic" impratio="10">
    <flag multiccd="enable"/>
  </option>
  <default>
    <geom solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}" friction="1 0.005 0.0001"/>
  </default>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="4096"/>
    <map znear="0.001"/>
  </visual>
"""

ENVIRONMENT_XML = """
    <light pos="0 0 1.5" dir="0 0 -1" diffuse="0.8 0.8 0.8" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.82 0.84 0.86 1" contype="1" conaffinity="1"/>
"""


def gear_body_xml(name="gear", scale=1.0, mass=0.5, pos=(0, 0, 0.1), free=True, rgba="0.75 0.62 0.25 1"):
    """MJCF body for the gear; its origin is the hub centre, the underside is at -half thickness.

    With ``free=False`` the gear is a mocap body: fixed in place unless its pose is written.
    """
    radius = GEAR_HUB_RADIUS_M * scale
    half_h = GEAR_HALF_THICKNESS_M * scale
    tooth_r, tooth_t = (s * scale for s in GEAR_TOOTH_HALF_SIZE_M)
    hub_volume = math.pi * radius**2 * 2 * half_h
    tooth_volume = 8 * tooth_r * tooth_t * half_h
    density = mass / (hub_volume + GEAR_TEETH * tooth_volume)
    contact = 'contype="1" conaffinity="3"'
    mocap = "" if free else ' mocap="true"'
    lines = [f'    <body name="{name}" pos="{" ".join(map(str, pos))}"{mocap}>']
    if free:
        lines.append(f'      <freejoint name="{name}_free"/>')
    lines.append(
        f'      <geom name="{name}_hub" type="cylinder" size="{radius} {half_h}" density="{density}"'
        f' rgba="{rgba}" {contact}/>'
    )
    for k in range(GEAR_TEETH):
        angle = 2 * math.pi * k / GEAR_TEETH
        centre = radius + tooth_r * 0.6
        lines.append(
            f'      <geom name="{name}_tooth{k}" type="box" size="{tooth_r} {tooth_t} {half_h}" density="{density}"'
            f' pos="{centre * math.cos(angle)} {centre * math.sin(angle)} 0" euler="0 0 {angle}" rgba="{rgba}"'
            f" {contact}/>"
        )
    lines.append("    </body>")
    return "\n".join(lines)


def hand_scene(extra_worldbody: str, hand_pos, hand_quat, kp=200.0, kd=5.0, armature=0.01):
    """Compile the DG-5F + DEX-01 hand with extra world bodies and joint position actuators.

    ``kp`` matches the Isaac Lab demo. Its ``damping=80`` targets PhysX's implicit drive; against
    MuJoCo's armature it would give a 0.4 s lag, so ``kd`` defaults to near-critical damping here.
    """
    hand_xml = build_hand_mjcf()
    scene_path = OUTPUT_DIR / "_scene.xml"
    scene_path.write_text(f"""<mujoco model="dg5f_dex01_scene">
  <include file="{hand_xml.name}"/>
  {OPTION_XML}
  <worldbody>
    {ENVIRONMENT_XML}
    {extra_worldbody}
  </worldbody>
</mujoco>
""")
    spec = mujoco.MjSpec.from_file(str(scene_path))
    base = spec.body("rl_dg_mount")
    base.pos = list(hand_pos)
    base.quat = list(hand_quat)
    for joint in spec.joints:
        if joint.name.startswith("rj_dg_"):
            actuator = spec.add_actuator()
            actuator.name = joint.name
            actuator.target = joint.name
            actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
            actuator.set_to_position(kp=kp)
            actuator.ctrlrange = joint.range
            actuator.ctrllimited = True
            # Matches the Isaac Lab demo's 17 N m effort limit.
            actuator.forcerange = [-17.0, 17.0]
            actuator.forcelimited = True
    model = spec.compile()
    # Joint damping is integrated implicitly; armature stands in for the gearmotor's reflected inertia
    # and keeps the stiff position servo stable at the physics timestep.
    for j in range(model.njnt):
        if model.joint(j).name.startswith("rj_dg_"):
            model.dof_damping[model.jnt_dofadr[j]] = kd
            model.dof_armature[model.jnt_dofadr[j]] = armature
    return model


def body_lowest_point_offset(model, body_name):
    """Distance from a body's origin down to its lowest collision geom extent (axis-aligned bodies)."""
    body = model.body(body_name).id
    bottoms = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == body and model.geom_type[g] in (
            mujoco.mjtGeom.mjGEOM_BOX,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
        ):
            half = model.geom_size[g][2] if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX else model.geom_size[g][1]
            bottoms.append(model.geom_pos[g][2] - half)
    return -float(np.min(bottoms))
