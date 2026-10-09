# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
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

# ADI modifications are licensed under Apache-2.0.
# Portions adapted from Isaac Lab retain their BSD-3-Clause terms:
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# The full license text is reproduced in NOTICE.

from __future__ import annotations

import re
from collections.abc import Sequence
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
import numpy as np
import omni.log
import torch
from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors.sensor_base import SensorBase
from isaaclab.utils.math import (
    convert_quat,
    quat_apply,
    quat_from_matrix,
    quat_inv,
    quat_mul,
)
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdPhysics

from .measurement import distribute_normal_load, tactile_image
from .tactile_sensor_data import TactileSensorData

if TYPE_CHECKING:
    from .tactile_sensor_cfg import TactileSensorCfg


class TactileSensor(SensorBase):
    cfg: TactileSensorCfg

    def __init__(self, cfg: TactileSensorCfg):
        # The leaf must be a literal name: env_ids index the batched views built from this path, so a
        # regex leaf would put more than one sensor per environment and break that correspondence.
        sensor_path = cfg.prim_path.split("/")[-1]
        sensor_path_is_regex = re.match(r"^[a-zA-Z0-9/_]+$", sensor_path) is None
        if sensor_path_is_regex:
            raise RuntimeError(
                f"Invalid prim path for the tactile sensor: {cfg.prim_path}."
                "\n\tHint: Please ensure that the prim path does not contain any regex patterns in the leaf."
            )
        super().__init__(cfg)
        self._data = TactileSensorData()

    """
    Properties.
    """

    @property
    def data(self) -> TactileSensorData:
        self._update_outdated_buffers()
        return self._data

    """
    Implementation.
    """

    def _initialize_impl(self):
        """Bind the PhysX views the sensor reads: sensor pose, contact against the counter object, SDF."""
        super()._initialize_impl()
        # obtain global simulation view
        self._physics_sim_view = SimulationManager.get_physics_sim_view()

        # create view for the sensor prim
        prim = sim_utils.find_first_matching_prim(self.cfg.prim_path)
        if prim is None:
            raise RuntimeError(f"Failed to find a prim at path expression: {self.cfg.prim_path}")
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(
                f"The prim at path {prim.GetPath().pathString} is not a rigid body."
                "\n\tHint: The tactile sensor reads poses and velocities from a PhysX rigid body view, so it must be"
                " attached to a prim with UsdPhysics.RigidBodyAPI applied (for an articulated hand, point"
                " 'prim_path' at the fingertip link rather than the articulation root)."
            )
        self._view = self._physics_sim_view.create_rigid_body_view(self.cfg.prim_path.replace(".*", "*"))
        self._sensor_com_l = self._view.get_coms()[:, :3].to(self._device)

        # contact view filtering the counter-object path
        self._contact_view = self._physics_sim_view.create_rigid_contact_view(
            self.cfg.prim_path.replace(".*", "*"),
            filter_patterns=[self.cfg.mesh_prim_path.replace(".*", "*")],
        )

        # initialize taxels before building the collider (num_taxels sizes the SDF view)
        self._initialize_taxels_impl()
        self._initialize_collider()

    def _initialize_collider(self):
        """Bind the pose and SDF views for the counter object."""
        mesh_prim_path = self.cfg.mesh_prim_path
        prim_path_0 = mesh_prim_path.replace(".*", "0")

        # The configured path may be the rigid body itself, or a wrapper above one (an articulation
        # link, or a spawn wrapper such as ".../env_0/Asset"), so fall back to searching downwards.
        top_prim = sim_utils.find_first_matching_prim(mesh_prim_path)
        if top_prim is not None and top_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body_path = mesh_prim_path.replace(".*", "*")
        else:
            rigid_body_prim = sim_utils.get_first_matching_child_prim(
                prim_path_0, lambda p: p.HasAPI(UsdPhysics.RigidBodyAPI)
            )
            if rigid_body_prim is None:
                raise RuntimeError(f"Could not find a RigidBodyAPI prim at or under: {mesh_prim_path}")
            body_path = str(rigid_body_prim.GetPath()).replace("env_0", "env_*")
        self._mesh_view = self._physics_sim_view.create_rigid_body_view(body_path)
        self._counter_com_l = self._mesh_view.get_coms()[:, :3].to(self._device)

        sdf_prims = sim_utils.get_all_matching_child_prims(
            prim_path_0,
            lambda p: p.GetTypeName() == "Mesh"
            and p.IsValid()
            and p.HasAPI(UsdPhysics.CollisionAPI)
            and p.HasAPI(UsdPhysics.MeshCollisionAPI)
            and UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == "sdf",
        )
        if not sdf_prims:
            raise RuntimeError(
                f"No SDF collision mesh found at or under '{prim_path_0}'. The tactile sensor measures penetration"
                " against a PhysX signed distance field, so the counter object must carry one."
                "\n\tHint: author physics:approximation = 'sdf' on the collision mesh, or apply"
                " isaaclab.sim.schemas.define_mesh_collision_properties(path, SDFMeshPropertiesCfg())."
            )
        if len(sdf_prims) != 1:
            raise ValueError("Each counter object must have exactly one SDF collision mesh.")

        submesh_prim_path = str(sdf_prims[0].GetPath()).replace("env_0", "env_*")
        # PhysX builds the SDF from the mesh's raw points, so query points must be divided by
        # the scale accumulated over the whole ancestor chain. Reading only the collider prim's
        # own scale op misses a scale authored on a spawn wrapper such as ".../env_0/Asset".
        world_xf = UsdGeom.XformCache().GetLocalToWorldTransform(sdf_prims[0])
        transform = Gf.Transform(world_xf)
        self._sdf_scale = np.array(transform.GetScale(), dtype=float)
        if not np.allclose(self._sdf_scale, self._sdf_scale[0], rtol=1e-5) or self._sdf_scale[0] <= 0:
            raise ValueError("SDF counter objects must have a positive uniform world scale.")
        body_prim = sim_utils.find_first_matching_prim(body_path.replace("env_*", "env_0"))
        body_xf = UsdGeom.XformCache().GetLocalToWorldTransform(body_prim)
        body_transform = Gf.Transform(body_xf)
        body_rotation = body_transform.GetRotation()
        mesh_rotation = transform.GetRotation()
        # Gf composes row-vector rotations left-to-right; Torch quaternions use
        # column-vector composition. This yields body^-1 @ mesh in Torch space.
        relative_rotation = mesh_rotation * body_rotation.GetInverse()
        q = relative_rotation.GetQuat()
        self._sdf_rot_l = torch.tensor([q.GetReal(), *q.GetImaginary()], dtype=torch.float32, device=self._device)
        offset = body_rotation.GetInverse().TransformDir(transform.GetTranslation() - body_transform.GetTranslation())
        self._sdf_pos_l = torch.tensor(list(offset), dtype=torch.float32, device=self._device)
        self._sdf_view = self._physics_sim_view.create_sdf_shape_view(submesh_prim_path, self.num_taxels)
        if self._mesh_view.count != self._view.count or self._sdf_view.count != self._view.count:
            raise ValueError("Expected one counter object and one SDF shape for each sensor instance.")
        omni.log.info(f"[TactileSensor] {mesh_prim_path}: SDF collider, scale {self._sdf_scale}")

    def _initialize_taxels_impl(self):
        """Build the taxel layout in the sensor frame and size the output buffers."""
        taxel_points, taxel_axes, taxel2pixel = self.cfg.pattern_cfg.func(self.cfg.pattern_cfg, self._device)
        self.num_taxels = len(taxel_points)
        self.taxel2pixel = taxel2pixel

        # apply offset transformation to the taxels
        offset_pos = torch.tensor(list(self.cfg.offset.pos), device=self._device)
        offset_quat = torch.tensor(list(self.cfg.offset.rot), device=self._device)

        taxel_points = quat_apply(offset_quat.repeat(self.num_taxels, 1), taxel_points)
        taxel_points += offset_pos
        taxel_axes = quat_apply(offset_quat.expand(self.num_taxels, 3, 4), taxel_axes.transpose(-1, -2)).transpose(
            -1, -2
        )

        # points and normals in local frame
        self.taxel_points_l = taxel_points.repeat(self._view.count, 1, 1)
        self.taxel_axes_l = taxel_axes.repeat(self._view.count, 1, 1, 1)
        self.offset_rot = quat_from_matrix(taxel_axes).repeat(self._view.count, 1, 1)

        # fill the data buffer
        self._data.taxel_points_w = torch.zeros(self._view.count, self.num_taxels, 3, device=self._device)
        self._data.taxel_normals_w = torch.zeros(self._view.count, self.num_taxels, 3, device=self._device)

        self._data.normal_forces = torch.zeros(self._view.count, self.num_taxels, device=self._device)
        self._data.depths = torch.zeros(self._view.count, self.num_taxels, device=self._device)
        self._data.depth_dots = torch.zeros(self._view.count, self.num_taxels, device=self._device)

        self._data.taxel_rot_w = torch.zeros(self._view.count, self.num_taxels, 4, device=self._device)

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        """Measure taxel penetration into the counter object and redistribute the net contact force."""
        pos_w, quat_w = self._view.get_transforms()[env_ids].split([3, 4], dim=-1)
        quat_w = convert_quat(quat_w, to="wxyz")

        # Sensor taxel positions / normals in world frame
        taxel_quat = quat_w.unsqueeze(1).expand(-1, self.num_taxels, -1)
        taxel_points_w = quat_apply(taxel_quat, self.taxel_points_l[env_ids])
        taxel_normals_w = quat_apply(taxel_quat, self.taxel_axes_l[env_ids, :, :, 2])
        taxel_points_w += pos_w.unsqueeze(1)

        # Contact gating: (len(env_ids),) — True where the counter object touches the sensor.
        # get_contact_force_matrix returns (num_sensors, num_filter_objects, 3), one filter here.
        contact_force_matrix = self._contact_view.get_contact_force_matrix(dt=self._sim_physics_dt)
        contact_mask = contact_force_matrix[env_ids, 0].norm(dim=-1) > 0.0

        if contact_mask.any():
            depths, depth_dots = self._compute_depths(env_ids, taxel_points_w)
            depths = depths * contact_mask.unsqueeze(1)
            depth_dots = depth_dots * contact_mask.unsqueeze(1)
        else:
            depths = torch.zeros(len(env_ids), self.num_taxels, device=self._device)
            depth_dots = torch.zeros_like(depths)

        # Store geometry outputs
        self._data.taxel_points_w[env_ids] = taxel_points_w
        self._data.taxel_normals_w[env_ids] = taxel_normals_w
        self._data.taxel_rot_w[env_ids] = quat_mul(
            quat_w.unsqueeze(1).repeat(1, self.num_taxels, 1), self.offset_rot[env_ids]
        )

        # Force from contact view — use summed net force for distribution
        net_force_w = contact_force_matrix[env_ids, 0]
        # Resolve the net force along the penetration-weighted mean of the taxel normals in
        # contact. A single taxel's normal is only representative on a flat pad.
        normal_forces, _ = distribute_normal_load(depths, taxel_normals_w, net_force_w)

        self._data.normal_forces[env_ids] = normal_forces
        self._data.depths[env_ids] = depths
        self._data.depth_dots[env_ids] = depth_dots

    def _compute_depths(self, env_ids: Sequence[int], taxel_points_w: torch.Tensor):
        """Query the PhysX SDF for taxel penetration depth and its rate of change.

        Returns ``(depths, depth_dots)``, both of shape ``(len(env_ids), num_taxels)``. Depths are
        clamped at zero, so a negative value is a penetration and zero means no contact.
        """
        body_pos_w, body_quat_w = self._mesh_view.get_transforms().split([3, 4], dim=-1)
        body_quat_w = convert_quat(body_quat_w, to="wxyz")
        mesh_pos_w = body_pos_w + quat_apply(body_quat_w, self._sdf_pos_l.expand(len(body_pos_w), 3))
        mesh_quat_w = quat_mul(body_quat_w, self._sdf_rot_l.expand(len(body_pos_w), 4))
        query = torch.zeros(self._view.count, self.num_taxels, 3, device=self._device)
        query[env_ids] = (
            quat_apply(
                quat_inv(mesh_quat_w[env_ids]).unsqueeze(1).expand(-1, self.num_taxels, -1),
                taxel_points_w - mesh_pos_w[env_ids].unsqueeze(1),
            )
            / self._sdf_scale[0]
        )
        # PhysX's SDF view queries every shape; only select after submitting a full batch.
        dist_and_grad = self._sdf_view.get_sdf_and_gradients(query)[env_ids]
        raw_depths = dist_and_grad[:, :, 3] * self._sdf_scale[0]
        depths = raw_depths.clamp(max=0.0)
        sensor_pose = self._view.get_transforms()[env_ids]
        sensor_quat = convert_quat(sensor_pose[:, 3:], to="wxyz")
        sensor_pos_w = sensor_pose[:, :3] + quat_apply(sensor_quat, self._sensor_com_l[env_ids])
        counter_com_w = body_pos_w[env_ids] + quat_apply(body_quat_w[env_ids], self._counter_com_l[env_ids])
        sensor_linvel, sensor_angvel = self._view.get_velocities()[env_ids].split([3, 3], dim=-1)
        body_linvel, body_angvel = self._mesh_view.get_velocities()[env_ids].split([3, 3], dim=-1)
        sensor_velocity = sensor_linvel.unsqueeze(1) + torch.cross(
            sensor_angvel.unsqueeze(1), taxel_points_w - sensor_pos_w.unsqueeze(1), dim=-1
        )
        counter_velocity = body_linvel.unsqueeze(1) + torch.cross(
            body_angvel.unsqueeze(1), taxel_points_w - counter_com_w.unsqueeze(1), dim=-1
        )
        # PhysX's interpolated gradients are not necessarily derivatives of its
        # interpolated distance. Differentiate the sampled SDF along relative
        # motion instead, preserving the same distance convention as depths.
        velocity_m = (
            quat_apply(
                quat_inv(mesh_quat_w[env_ids]).unsqueeze(1).expand(-1, self.num_taxels, -1),
                sensor_velocity - counter_velocity,
            )
            / self._sdf_scale[0]
        )
        # Time half-width (seconds) for the symmetric SDF derivative. One ms
        # avoids float32 cancellation for sub-mm distances; it is a numerical
        # sampling choice, not the physics timestep or a material time constant.
        epsilon = 0.001
        forward = query.clone()
        backward = query.clone()
        forward[env_ids] += epsilon * velocity_m
        backward[env_ids] -= epsilon * velocity_m
        d_forward = self._sdf_view.get_sdf_and_gradients(forward)[env_ids, :, 3].clone()
        d_backward = self._sdf_view.get_sdf_and_gradients(backward)[env_ids, :, 3]
        depth_dots = (d_forward - d_backward) * self._sdf_scale[0] / (2 * epsilon)
        depth_dots = torch.where(raw_depths < 0.0, depth_dots, 0.0)
        return depths, depth_dots

    def _set_debug_vis_impl(self, debug_vis: bool):
        # set visibility of markers
        # note: parent only deals with callbacks. not their visibility
        if debug_vis:
            if not hasattr(self, "taxel_visualizer"):
                self.cfg.visualizer_cfg.markers["taxels"].size = (
                    0.95 * self.cfg.pattern_cfg.resolution[0],
                    0.95 * self.cfg.pattern_cfg.resolution[1],
                    0.0001,
                )
                self.taxel_visualizer = VisualizationMarkers(self.cfg.visualizer_cfg)
            # set their visibility to true
            self.taxel_visualizer.set_visibility(True)
        else:
            if hasattr(self, "taxel_visualizer"):
                self.taxel_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        self.taxel_visualizer.visualize(
            translations=self._data.taxel_points_w.reshape(-1, 3),
            orientations=self._data.taxel_rot_w.reshape(-1, 4),
        )

    def get_tactile_image(self):
        return tactile_image(self.data.normal_forces, self.taxel2pixel)

    def get_filtered_normal_force_w(self):
        """Return the counter-object filtered normal contact force [N], shape [E,3]."""
        return self._contact_view.get_contact_force_matrix(dt=self._sim_physics_dt)[:, 0]

    def __str__(self) -> str:
        """Returns: A string containing information about the instance."""
        return (
            f"Tactile-sensor @ '{self.cfg.prim_path}': \n"
            f"\tview type              : {self._view.__class__}\n"
            f"\tupdate period (s)      : {self.cfg.update_period}\n"
            f"\tcounter object         : {self.cfg.mesh_prim_path}\n"
            f"\tnumber of sensors      : {self._view.count}\n"
            f"\tnumber of taxels/sensor: {self.num_taxels}\n"
            f"\ttotal number of taxels : {self.num_taxels * self._view.count}"
        )

    """
    Internal simulation callbacks.
    """

    def _invalidate_initialize_callback(self, event):
        """Invalidates the scene elements."""
        # call parent
        super()._invalidate_initialize_callback(event)
        # set all existing views to None to invalidate them
        self._view = None
        self._contact_view = None
        self._mesh_view = None
        self._sdf_view = None
