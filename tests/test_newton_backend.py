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

"""Exercise actual Newton/Warp sensing; run with the pinned Isaac Lab environment.

Set DEX01_REQUIRE_NEWTON_TESTS=1 to make missing dependencies an error rather
than an optional skip. Synthetic contacts isolate kinematics tests; the live
fixture below independently exercises solver filtering, contact and release.
"""

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "source/dex01_sim_asset"))
try:
    import isaaclab  # noqa: F401
    import newton  # noqa: F401
    import warp as wp
except ImportError:
    if os.environ.get("DEX01_REQUIRE_NEWTON_TESTS") == "1":
        raise
    pytest.skip("Requires the pinned Isaac Lab/Newton environment", allow_module_level=True)

import isaaclab.sim as sim
import trimesh
from dex01_sim_asset.newton import TactileSensor, TactileSensorCfg
from dex01_sim_asset.newton.assets import write_indenter
from dex01_sim_asset.taxel_patterns import GridPatternCfg
from isaaclab.app import launch_simulation
from isaaclab.assets import RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sim.schemas import UsdPhysicsCollisionCfg, UsdPhysicsRigidBodyCfg
from isaaclab.utils import configclass
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from scipy.spatial.transform import Rotation

DEVICE = os.environ.get("DEX01_NEWTON_TEST_DEVICE", "cuda:0")
if DEVICE == "cpu" or not torch.cuda.is_available():
    if os.environ.get("DEX01_REQUIRE_NEWTON_TESTS") == "1":
        raise RuntimeError("The Newton SDF integration suite requires a CUDA GPU")
    pytest.skip("Newton texture-SDF integration requires CUDA", allow_module_level=True)


def tensor(value):
    return torch.tensor(np.asarray(value), dtype=torch.float32, device=DEVICE)


@pytest.fixture
def scene_and_sim():
    """Two independently filtered environments and a second counter object."""
    mesh = trimesh.creation.icosphere(subdivisions=3, radius=0.003)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        pad = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pad",
            spawn=sim.CuboidCfg(
                size=(0.04, 0.04, 0.01),
                rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                mass_props=sim.MassPropertiesCfg(mass=0.02),
                collision_props=UsdPhysicsCollisionCfg(),
            ),
        )
        probe = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Probe",
            spawn=sim.UsdFileCfg(usd_path=write_indenter(mesh), rigid_props=UsdPhysicsRigidBodyCfg()),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 0.008)),
        )
        distractor = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Distractor",
            spawn=sim.SphereCfg(
                radius=0.003,
                rigid_props=UsdPhysicsRigidBodyCfg(),
                mass_props=sim.MassPropertiesCfg(mass=0.1),
                collision_props=UsdPhysicsCollisionCfg(),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.01, 0, 0.008)),
        )
        contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Pad", filter_prim_paths_expr=["{ENV_REGEX_NS}/Probe"])
        distractor_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Pad", filter_prim_paths_expr=["{ENV_REGEX_NS}/Distractor"]
        )

    physics = NewtonCfg(
        use_cuda_graph=DEVICE != "cpu",
        solver_cfg=MJWarpSolverCfg(
            use_mujoco_contacts=False, use_mujoco_cpu=DEVICE == "cpu", integrator="implicitfast", nconmax=160, njmax=960
        ),
    )
    with launch_simulation(physics) as resolved:
        context = SimulationContext(SimulationCfg(device=DEVICE, dt=1 / 240, physics=resolved))
        try:
            scene = InteractiveScene(SceneCfg(num_envs=2, env_spacing=0.1))
            sensor = TactileSensor(
                TactileSensorCfg(
                    prim_path="/World/envs/env_.*/Pad",
                    mesh_prim_path="/World/envs/env_.*/Probe",
                    pattern_cfg=GridPatternCfg(size=(0.01, 0.01), resolution=(0.0005, 0.0005)),
                    offset_pos=(0, 0, 0.005),
                )
            )
            context.reset()

            def state(asset):
                return lambda: (
                    asset.data.root_link_pose_w.torch,
                    asset.data.root_com_pos_w.torch,
                    asset.data.root_com_vel_w.torch,
                )

            sensor.bind(state(scene["pad"]), state(scene["probe"]), scene["contact"], mesh.vertices, mesh.faces)
            yield scene, context, sensor
        finally:
            context.clear_instance()


def step(scene, context, sensor, count=1):
    for _ in range(count):
        scene.write_data_to_sim()
        context.step(render=False)
        scene.update(1 / 240)
        sensor.update(1 / 240)
        sensor.data


def test_live_filtering_release_subset_and_reset(scene_and_sim):
    scene, context, sensor = scene_and_sim
    step(scene, context, sensor, 240)
    torch.testing.assert_close(sensor.data.normal_forces.sum(-1), tensor([0.981, 0.981]), rtol=0.05, atol=0)
    old = sensor.data.normal_forces[0].clone()
    sensor.reset(env_ids=[1])
    assert torch.equal(sensor._data.normal_forces[0], old)
    assert sensor._data.normal_forces[1].sum() == 0
    sensor._update_buffers_impl(wp.array([False, True], dtype=wp.bool, device=DEVICE))
    assert torch.equal(sensor._data.normal_forces[0], old)
    assert sensor._data.normal_forces[1].sum() > 0.9
    probe = scene["probe"]
    pose = probe.data.root_link_pose_w.torch[1:2].clone()
    pose[:, 2] = 0.1
    probe.write_root_pose_to_sim_index(root_pose=pose, env_ids=[1])
    probe.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=DEVICE), env_ids=[1])
    step(scene, context, sensor)
    # A real distractor continues contacting the pad, while the configured probe is separated.
    assert scene["distractor_contact"].data.normal_force_matrix_w.torch[1].norm() > 0.1
    assert sensor.get_filtered_normal_force_w()[1].norm() == 0
    for name in ("normal_forces", "depths", "depth_dots"):
        assert getattr(sensor.data, name)[1].abs().sum() == 0
    assert sensor.data.normal_forces[0].sum() > 0.9
    sensor.reset()
    assert sensor._data.normal_forces.abs().sum() == 0
    step(scene, context, sensor)
    assert sensor.data.normal_forces[0].sum() > 0.9
    assert sensor.data.normal_forces[1].sum() == 0


@pytest.mark.parametrize("scale", [0.5, 1.0, 2.0])
def test_actual_adapter_transforms_depth_rate_and_quaternions(scene_and_sim, scale):
    _, _, sensor = scene_and_sim
    box = trimesh.creation.box(extents=(0.02, 0.02, 0.02))
    body_rot = Rotation.from_euler("xyz", [0.2, -0.3, 0.4])
    collider_rot = Rotation.from_euler("xyz", [-0.1, 0.25, 0.15])
    sensor_rot = Rotation.from_euler("xyz", [0.3, 0.1, -0.2])
    body_pos = np.array([0.02, -0.01, 0.03])
    offset = np.array([0.004, -0.002, 0.001])
    mesh_rot = body_rot * collider_rot
    mesh_pos = body_pos + body_rot.apply(offset)
    # Repeat two local query points through the current full taxel pattern, including an outside sample.
    query = np.tile([[0.009, 0.002, 0.001], [0.013, 0.002, 0.001]], (sensor.num_taxels // 2 + 1, 1))[
        : sensor.num_taxels
    ]
    world = mesh_rot.apply(query * scale) + mesh_pos
    local_sensor = sensor_rot.inv().apply(world)
    sensor._points_l = tensor(local_sensor)
    counter_pose = tensor(np.tile(np.r_[body_pos, body_rot.as_quat()], (2, 1)))
    sensing_pose = tensor(np.tile(np.r_[[0, 0, 0], sensor_rot.as_quat()], (2, 1)))
    sensor_com = tensor([[0.001, 0.002, -0.001]] * 2)
    counter_com = tensor([body_pos + [0.002, -0.001, 0.003]] * 2)
    sensor_vel = tensor([[0.01, -0.02, 0.03, -0.2, 0.3, 0.1]] * 2)
    counter_vel = tensor([[0.02, -0.03, 0.01, 0.4, -0.2, 0.3]] * 2)
    contact = SimpleNamespace(
        cfg=SimpleNamespace(
            prim_path=sensor.cfg.prim_path, filter_prim_paths_expr=[sensor.cfg.mesh_prim_path], update_period=0
        ),
        data=SimpleNamespace(normal_force_matrix_w=SimpleNamespace(torch=tensor([[[[0, 0, -1]]]] * 2))),
    )
    sensor.bind(
        lambda: (sensing_pose, sensor_com, sensor_vel),
        lambda: (counter_pose, counter_com, counter_vel),
        contact,
        box.vertices,
        box.faces,
        scale,
        np.r_[offset, collider_rot.as_quat()],
    )
    sensor._update_buffers_impl(wp.array([True, True], dtype=wp.bool, device=DEVICE))

    def box_distance(p):
        q = np.abs(p) - 0.01
        return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(np.max(q, axis=-1), 0)

    expected = np.minimum(box_distance(query) * scale, 0)
    np.testing.assert_allclose(sensor._data.depths[0].cpu(), expected, atol=2e-7)
    sv = sensor_vel[0, :3].cpu().numpy() + np.cross(
        sensor_vel[0, 3:].cpu().numpy(), world - sensor_com[0].cpu().numpy()
    )
    cv = counter_vel[0, :3].cpu().numpy() + np.cross(
        counter_vel[0, 3:].cpu().numpy(), world - counter_com[0].cpu().numpy()
    )
    relative = mesh_rot.inv().apply(sv - cv) / scale
    rate = (box_distance(query + 0.001 * relative) - box_distance(query - 0.001 * relative)) * scale / 0.002
    rate[expected >= 0] = 0
    np.testing.assert_allclose(sensor._data.depth_dots[0].cpu(), rate, atol=1e-5)
    np.testing.assert_allclose(sensor._data.taxel_points_w[0].cpu(), world, atol=2e-7)
    expected_normal = sensor_rot.apply(sensor._normals_l.cpu().numpy())
    np.testing.assert_allclose(sensor._data.taxel_normals_w[0].cpu(), expected_normal, atol=2e-7)
    rotations = sensor._data.taxel_rot_w[0].cpu().numpy()[:, [1, 2, 3, 0]]
    expected_rot = sensor_rot * Rotation.from_quat(sensor._rot_l.cpu().numpy())
    np.testing.assert_allclose(Rotation.from_quat(rotations).as_matrix(), expected_rot.as_matrix(), atol=2e-6)


def test_invalid_binding_preserves_previous_contact_and_geometry(scene_and_sim):
    scene, _, sensor = scene_and_sim
    old = (sensor._mesh, sensor._sensor_state, sensor._contact)
    bad_contact = SimpleNamespace(
        cfg=SimpleNamespace(prim_path=sensor.cfg.prim_path, filter_prim_paths_expr=["/World/Wrong"], update_period=0)
    )
    mesh = trimesh.creation.box()
    with pytest.raises(ValueError, match="paths"):
        sensor.bind(None, None, bad_contact, mesh.vertices, mesh.faces)
    assert (sensor._mesh, sensor._sensor_state, sensor._contact) == old
    with pytest.raises(ValueError, match="quaternion"):
        sensor.bind(None, None, scene["contact"], mesh.vertices, mesh.faces, mesh_pose=[0, 0, 0, 0, 0, 0, 2])
    assert (sensor._mesh, sensor._sensor_state, sensor._contact) == old
    with pytest.raises(ValueError, match="closed"):
        sensor.bind(None, None, scene["contact"], mesh.vertices, mesh.faces[:-1])
    assert (sensor._mesh, sensor._sensor_state, sensor._contact) == old
