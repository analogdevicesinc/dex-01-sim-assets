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

"""Analytic checks of transforms, relative motion, batching and force isolation."""

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "source/dex01_sim_asset/dex01_sim_asset/tactile_sensor.py"


def quat_mul(a, b):
    w1, x1, y1, z1 = a.unbind(-1)
    w2, x2, y2, z2 = b.unbind(-1)
    return torch.stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ),
        -1,
    )


def quat_apply(q, v):
    u = q[..., 1:]
    t = 2 * torch.cross(u, v, dim=-1)
    return v + q[..., :1] * t + torch.cross(u, t, dim=-1)


def quat_inv(q):
    return q * torch.tensor([1.0, -1.0, -1.0, -1.0])


def sensor_class():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TactileSensor")
    namespace = {
        "SensorBase": object,
        "TactileSensorCfg": object,
        "Sequence": list,
        "torch": torch,
        "np": np,
        "quat_mul": quat_mul,
        "quat_apply": quat_apply,
        "quat_inv": quat_inv,
        "convert_quat": lambda q, to: torch.cat((q[..., 3:], q[..., :3]), -1),
    }
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[]
    )
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace["TactileSensor"]


class View:
    def __init__(self, position, rotation=None, velocity=None, com=None):
        self.count = len(position)
        if rotation is None:
            rotation = torch.tensor([[1.0, 0, 0, 0]]).expand(self.count, 4)
        self.transforms = torch.cat((position, rotation[:, [1, 2, 3, 0]]), -1)
        self.velocity = torch.zeros(self.count, 6) if velocity is None else velocity
        self.com = torch.zeros(self.count, 7)
        self.com[:, 6] = 1
        if com is not None:
            self.com[:, :3] = com

    def get_transforms(self):
        return self.transforms

    def get_velocities(self):
        return self.velocity

    def get_coms(self):
        return self.com


class SphereSDF:
    def __init__(self, radius, count):
        self.radius = radius
        self.count = count

    def get_sdf_and_gradients(self, points):
        assert len(points) == self.count
        norm = points.norm(dim=-1, keepdim=True)
        return torch.cat((points / norm.clamp(min=1e-8), norm - self.radius), -1)


@pytest.mark.parametrize("angle", [0.0, 0.4, 1.2])
@pytest.mark.parametrize("scale", [0.5, 1.0, 2.0])
def test_sdf_depth_uses_mesh_frame_world_scale_and_relative_velocity(angle, scale):
    sensor = object.__new__(sensor_class())
    sensor._device = "cpu"
    sensor.num_taxels = 2
    body_position = torch.tensor([[0.13, -0.04, 0.06], [0.2, 0.1, 0.03]])
    q = torch.tensor([[np.cos(angle / 2), 0, 0, np.sin(angle / 2)]], dtype=torch.float32).expand(2, 4)
    local_q = torch.tensor([np.cos(0.3 / 2), 0, np.sin(0.3 / 2), 0], dtype=torch.float32)
    sensor._mesh_view = View(
        body_position,
        q,
        torch.tensor([[0.01, 0.02, -0.03, 0.1, -0.2, 0.3], [0, 0, 0, 0, 0, 0]]),
        com=torch.tensor([[0.017, -0.006, 0.004], [0, 0, 0]]),
    )
    sensor._sdf_pos_l = torch.tensor([0.02, -0.01, 0.015])
    sensor._sdf_rot_l = local_q
    sensor._sdf_scale = np.array([scale] * 3)
    sensor._view = View(
        torch.tensor([[0.1, 0.01, 0.04], [0.0, 0.0, 0.0]]),
        velocity=torch.tensor([[0.04, -0.01, 0.02, -0.2, 0.3, 0.1], [0, 0, 0, 0, 0, 0]]),
        com=torch.tensor([[0.009, 0.003, -0.008], [0, 0, 0]]),
    )
    sensor._sdf_view = SphereSDF(0.01, 2)
    sensor._sensor_com_l = sensor._view.com[:, :3]
    sensor._counter_com_l = sensor._mesh_view.com[:, :3]
    mesh_rotation = quat_mul(q[:1], local_q.expand(1, 4))
    mesh_position = body_position[:1] + quat_apply(q[:1], sensor._sdf_pos_l.expand(1, 3))
    local = torch.tensor([[[0.008, 0, 0], [0, 0.012, 0]]])
    points = quat_apply(mesh_rotation[:, None, :].expand(1, 2, 4), local * scale) + mesh_position[:, None, :]
    depths, rates = sensor._compute_depths(torch.tensor([0]), points)
    assert torch.allclose(depths, torch.tensor([[-0.002 * scale, 0.0]]), atol=1e-7)
    world_normal = quat_apply(mesh_rotation[:, None, :].expand(1, 2, 4), torch.tensor([[[1.0, 0, 0], [0, 1.0, 0]]]))
    sv = sensor._view.velocity[:1]
    bv = sensor._mesh_view.velocity[:1]
    sensor_com = sensor._view.transforms[:1, :3] + sensor._view.com[:1, :3]
    body_com = body_position[:1] + quat_apply(q[:1], sensor._mesh_view.com[:1, :3])
    sensor_velocity = sv[:, :3, None].transpose(1, 2) + torch.cross(
        sv[:, None, 3:], points - sensor_com[:, None, :], dim=-1
    )
    body_velocity = bv[:, :3, None].transpose(1, 2) + torch.cross(
        bv[:, None, 3:], points - body_com[:, None, :], dim=-1
    )
    expected = (world_normal * (sensor_velocity - body_velocity)).sum(-1)
    expected[:, 1] = 0
    assert torch.allclose(rates, expected, atol=1e-5)


def test_filtered_force_distribution_and_subset_orientation():
    sensor = object.__new__(sensor_class())
    sensor._device = "cpu"
    sensor.num_taxels = 2
    sensor._sim_physics_dt = 0.01
    sensor._view = View(torch.zeros(2, 3))
    sensor.taxel_points_l = torch.zeros(2, 2, 3)
    sensor.taxel_axes_l = torch.eye(3).expand(2, 2, 3, 3).clone()
    sensor.offset_rot = torch.tensor([[[1.0, 0, 0, 0], [1.0, 0, 0, 0]], [[0.0, 1.0, 0, 0], [0.0, 1.0, 0, 0]]])
    sensor._contact_view = SimpleNamespace(
        get_contact_force_matrix=lambda dt: torch.tensor([[[0.0, 0.0, -3.0]], [[0.0, 0.0, -6.0]]])
    )
    sensor._compute_depths = lambda ids, points: (torch.tensor([[-0.001, -0.002]]), torch.zeros(1, 2))
    sensor._data = SimpleNamespace(**{
        n: torch.zeros(shape)
        for n, shape in (
            ("taxel_points_w", (2, 2, 3)),
            ("taxel_normals_w", (2, 2, 3)),
            ("taxel_rot_w", (2, 2, 4)),
            ("normal_forces", (2, 2)),
            ("depths", (2, 2)),
            ("depth_dots", (2, 2)),
        )
    })
    sensor._update_buffers_impl(torch.tensor([1]))
    assert torch.allclose(sensor._data.normal_forces[1], torch.tensor([2.0, 4.0]))
    assert sensor._data.normal_forces[0].sum() == 0
    assert torch.equal(sensor._data.taxel_rot_w[1], sensor.offset_rot[1])
