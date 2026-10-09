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

"""Isaac Lab 3.0 tactile sensor with normal-only contact and mesh distance queries."""

from dataclasses import MISSING

import numpy as np
import torch
import warp as wp
from isaaclab.sensors import SensorBase, SensorBaseCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_from_matrix, quat_inv, quat_mul

from ..measurement import distribute_normal_load, tactile_image
from ..tactile_sensor_data import TactileSensorData
from .distance import query_distances


@configclass
class TactileSensorCfg(SensorBaseCfg):
    """Bind one sensing link and counter-object collider per environment."""

    class_type: type | str = "dex01_sim_asset.newton.tactile_sensor:TactileSensor"
    pattern_cfg: object = MISSING
    mesh_prim_path: str = MISSING
    offset_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_rot_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class TactileSensor(SensorBase):
    """Expose legacy Torch/WXYZ tactile outputs from the Newton/XYZW backend.

    Call ``bind`` after scene reset with backend-neutral frame/velocity providers
    and a ContactSensor configured for exactly this counter object. Mesh points
    are in its collider frame; scale must be positive and uniform.
    """

    def __init__(self, cfg):
        self._data = TactileSensorData()
        self._bound = False
        super().__init__(cfg)

    def _initialize_impl(self):
        super()._initialize_impl()
        points, axes, pixels = self.cfg.pattern_cfg.func(self.cfg.pattern_cfg, self._device)
        self.num_taxels = len(points)
        self.taxel2pixel = pixels
        q = torch.tensor(self.cfg.offset_rot_wxyz[1:] + self.cfg.offset_rot_wxyz[:1], device=self._device)
        if not torch.isfinite(q).all() or not torch.isclose(q.norm(), q.new_tensor(1.0), atol=1e-5):
            raise ValueError("Mount quaternion must be finite and normalized WXYZ")
        if not np.isfinite(self.cfg.offset_pos).all():
            raise ValueError("Mount translation must be finite")
        self._points_l = quat_apply(q.expand(len(points), 4), points)
        self._points_l += torch.tensor(self.cfg.offset_pos, device=self._device)
        self._normals_l = quat_apply(q.expand(len(points), 4), axes[:, :, 2])
        self._rot_l = quat_mul(q.expand(len(points), 4), quat_from_matrix(axes))
        for name, width in (("taxel_points_w", 3), ("taxel_normals_w", 3), ("taxel_rot_w", 4)):
            setattr(self._data, name, torch.zeros(self._num_envs, len(points), width, device=self._device))
        for name in ("normal_forces", "depths", "depth_dots"):
            setattr(self._data, name, torch.zeros(self._num_envs, len(points), device=self._device))
        self.filtered_normal_force_w = torch.zeros(self._num_envs, 3, device=self._device)
        self.penetration_normal_w = torch.zeros_like(self.filtered_normal_force_w)
        self._distance = wp.zeros((self._num_envs, len(points)), dtype=float, device=self._device)
        self._rate = wp.zeros_like(self._distance)
        self._query = torch.empty(self._num_envs, len(points), 3, device=self._device)
        self._velocity = torch.empty_like(self._query)

    def bind(self, sensor_state, counter_state, contact, vertices, faces, scale=1.0, mesh_pose=None):
        """Bind state callbacks returning link pose XYZW, COM position, COM velocity.

        ``mesh_pose`` is the fixed collider-to-counter-link transform [m,XYZW].
        Contact observations must use normal_force_matrix_w and one filter body.
        """
        if not self.is_initialized:
            raise RuntimeError("Reset the scene before binding tactile state providers")
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Counter-object scale must be positive and uniform")
        vertices = np.asarray(vertices)
        faces = np.asarray(faces)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
            raise ValueError("Counter mesh vertices must be finite [V,3]")
        if faces.ndim != 2 or faces.shape[1] != 3 or not np.issubdtype(faces.dtype, np.integer):
            raise ValueError("Counter mesh requires integer triangle indices [F,3]")
        if not len(faces) or faces.min() < 0 or faces.max() >= len(vertices):
            raise ValueError("Counter mesh triangle indices are out of bounds")
        edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        if (counts != 2).any():
            raise ValueError("Counter mesh must be closed for parity signed-distance queries")
        triangles = vertices[faces]
        if (
            np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
            <= 1e-15
        ).any():
            raise ValueError("Counter mesh contains degenerate triangles")
        if len(contact.cfg.filter_prim_paths_expr) != 1:
            raise ValueError("Configure exactly one counter-object contact filter")
        if contact.cfg.update_period != 0:
            raise ValueError("Contact sensor must update every physics tick")

        def canonical(path):
            return path.replace("env_[^/]+", "env_.*")

        if canonical(contact.cfg.prim_path) != canonical(self.cfg.prim_path) or canonical(
            contact.cfg.filter_prim_paths_expr[0]
        ) != canonical(self.cfg.mesh_prim_path):
            raise ValueError("Contact sensor paths must match the configured sensing link and counter object")
        collider_pose = torch.tensor(
            (0, 0, 0, 0, 0, 0, 1) if mesh_pose is None else mesh_pose,
            dtype=torch.float32,
            device=self._device,
        )
        if collider_pose.shape != (7,) or not torch.isfinite(collider_pose).all():
            raise ValueError("Collider transform must be a finite seven-component pose")
        if not torch.isclose(collider_pose[3:].norm(), collider_pose.new_tensor(1.0), atol=1e-5):
            raise ValueError("Collider quaternion must have unit length")
        self._sensor_state = sensor_state
        self._counter_state = counter_state
        self._contact = contact
        self._mesh = wp.Mesh(
            points=wp.array(vertices, dtype=wp.vec3, device=self._device),
            indices=wp.array(np.asarray(faces).ravel(), dtype=wp.int32, device=self._device),
        )
        self._scale = float(scale)
        self._mesh_pose = collider_pose
        self._bound = True
        self._is_outdated.fill_(True)

    @property
    def data(self):
        if not self._bound:
            raise RuntimeError("Bind the tactile sensor after scene reset before reading data")
        self._update_outdated_buffers()
        return self._data

    def _update_buffers_impl(self, env_mask):
        if not self._bound:
            return
        sensor_pose, sensor_com, sensor_vel = self._sensor_state()
        counter_pose, counter_com, counter_vel = self._counter_state()
        count = self._num_envs
        for pose, com, velocity in ((sensor_pose, sensor_com, sensor_vel), (counter_pose, counter_com, counter_vel)):
            if pose.shape != (count, 7) or com.shape != (count, 3) or velocity.shape != (count, 6):
                raise ValueError("State providers must return one link pose, COM and velocity per environment")
        q = sensor_pose[:, 3:].unsqueeze(1).expand(count, self.num_taxels, 4)
        points = quat_apply(q, self._points_l.expand(count, -1, -1)) + sensor_pose[:, None, :3]
        normals = quat_apply(q, self._normals_l.expand(count, -1, -1))
        mesh_q = quat_mul(counter_pose[:, 3:], self._mesh_pose[3:].expand(count, 4))
        mesh_p = counter_pose[:, :3] + quat_apply(counter_pose[:, 3:], self._mesh_pose[:3].expand(count, 3))
        inv = quat_inv(mesh_q).unsqueeze(1).expand(count, self.num_taxels, 4)
        query = quat_apply(inv, points - mesh_p[:, None]) / self._scale
        sensor_velocity = sensor_vel[:, None, :3] + torch.cross(
            sensor_vel[:, None, 3:].expand_as(points), points - sensor_com[:, None], dim=-1
        )
        counter_velocity = counter_vel[:, None, :3] + torch.cross(
            counter_vel[:, None, 3:].expand_as(points), points - counter_com[:, None], dim=-1
        )
        velocity = quat_apply(inv, sensor_velocity - counter_velocity) / self._scale
        matrix = self._contact.data.normal_force_matrix_w.torch
        if matrix.shape != (count, 1, 1, 3):
            raise ValueError("Expected exactly one sensing link and counter object per environment")
        force = matrix[:, 0, 0]
        active = force.norm(dim=-1) > 0
        self._query.copy_(query)
        self._velocity.copy_(velocity)
        wp.launch(
            query_distances,
            (count, self.num_taxels),
            inputs=[
                self._mesh.id,
                wp.from_torch(self._query, dtype=wp.vec3),
                wp.from_torch(self._velocity, dtype=wp.vec3),
                wp.from_torch(active, dtype=wp.bool),
                self._scale,
                0.001,
            ],
            outputs=[self._distance, self._rate],
            device=self._device,
        )
        depths = wp.to_torch(self._distance) * active[:, None]
        rates = wp.to_torch(self._rate) * active[:, None]
        loads, normal = distribute_normal_load(depths, normals, force)
        ids = wp.to_torch(env_mask).bool()
        self.filtered_normal_force_w[ids] = force[ids]
        self.penetration_normal_w[ids] = normal[ids]
        self._data.taxel_points_w[ids] = points[ids]
        self._data.taxel_normals_w[ids] = normals[ids]
        rotation = quat_mul(q, self._rot_l.expand(count, -1, -1))
        self._data.taxel_rot_w[ids] = rotation[ids][..., [3, 0, 1, 2]]
        self._data.depths[ids] = depths[ids]
        self._data.depth_dots[ids] = rates[ids]
        self._data.normal_forces[ids] = loads[ids]

    def get_tactile_image(self):
        return tactile_image(self.data.normal_forces, self.taxel2pixel)

    def get_filtered_normal_force_w(self):
        self.data
        return self.filtered_normal_force_w

    def frame_state(self):
        """Return sensing-link pose [m,XYZW], COM position [m], and COM velocity [m/s,rad/s]."""
        if not self._bound:
            raise RuntimeError("Tactile sensor is not bound")
        return self._sensor_state()

    def reset(self, env_ids=None, env_mask=None):
        super().reset(env_ids=env_ids, env_mask=env_mask)
        if not self.is_initialized:
            return
        ids = wp.to_torch(env_mask).bool() if env_mask is not None else (slice(None) if env_ids is None else env_ids)
        for name in ("normal_forces", "depths", "depth_dots"):
            getattr(self._data, name)[ids] = 0
        self.filtered_normal_force_w[ids] = 0
        self.penetration_normal_w[ids] = 0

    def _set_debug_vis_impl(self, debug_vis):
        if debug_vis:
            from isaaclab.markers import VisualizationMarkers

            from ..marker_cfg import TACTILE_SENSOR_MARKER_CFG

            if not hasattr(self, "_markers"):
                cfg = TACTILE_SENSOR_MARKER_CFG.copy()
                cfg.prim_path = "/Visuals/NewtonTactile/" + self.cfg.prim_path.rsplit("/", 1)[-1]
                cfg.markers["taxels"].size = (*self.cfg.pattern_cfg.resolution, 0.0001)
                self._markers = VisualizationMarkers(cfg)
            self._markers.set_visibility(True)
        elif hasattr(self, "_markers"):
            self._markers.set_visibility(False)

    def _debug_vis_callback(self, event):
        if self._bound:
            data = self._data
            self._markers.visualize(
                translations=data.taxel_points_w.reshape(-1, 3),
                orientations=data.taxel_rot_w[..., [1, 2, 3, 0]].reshape(-1, 4),
            )

    def _invalidate_initialize_callback(self, event):
        self._bound = False
        super()._invalidate_initialize_callback(event)
