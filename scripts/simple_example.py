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

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Simple Example of Tactile Sensor on Cube Prim with Gear Indenter.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument(
    "--max-steps", type=int, default=0, help="Stop after this many simulation steps (0 runs continuously)."
)

parser.add_argument("--no-tactile-ui", action="store_true", help="Hide the streamed tactile overlay.")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import isaaclab.sim as sim_utils
from dex01_sim_asset import TactileSensorCfg, taxel_patterns, vis_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.sim.spawners import materials
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

sensor_size = (0.2, 0.1, 0.01)
taxel_grid_size = (32, 32)
taxel_grid_res = (sensor_size[0] / taxel_grid_size[0], sensor_size[1] / taxel_grid_size[1])


@configclass
class FingerPadSceneCfg(InteractiveSceneCfg):
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.6, 0.6, 0.8))
    )

    asset = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Asset",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Factory/factory_gear_medium.usd",
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled=False,
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=3666.0,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=192,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.001,
                rest_offset=0.0,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),  # needs to be zero
        ),
    )

    finger_pad = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/FingerPad",
        spawn=sim_utils.CuboidCfg(
            size=sensor_size,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,
                rigid_body_enabled=True,
                linear_damping=0.0,
                angular_damping=0.0,
                solver_position_iteration_count=192,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.001,
                rest_offset=0.0,
            ),
            physics_material=materials.RigidBodyMaterialCfg(
                compliant_contact_stiffness=1.0e4,
                compliant_contact_damping=5.0e1,
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1), metallic=0.2),
            activate_contact_sensors=True,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
        ),
    )

    sensor = TactileSensorCfg(
        prim_path="{ENV_REGEX_NS}/FingerPad",
        offset=TactileSensorCfg.OffsetCfg(pos=(0, 0, 0.5 * sensor_size[2])),
        mesh_prim_path="/World/envs/env_.*/Asset/factory_gear_medium",
        pattern_cfg=taxel_patterns.GridPatternCfg(
            resolution=taxel_grid_res,
            size=(sensor_size[0] - taxel_grid_res[0], sensor_size[1] - taxel_grid_res[1]),
        ),
        debug_vis=True,
    )


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device,
        dt=1.0 / 120.0,
        gravity=(0.0, 0.0, -9.81),
        physx=sim_utils.PhysxCfg(
            solver_type=1,
            max_position_iteration_count=192,  # Important to avoid interpenetration.
            bounce_threshold_velocity=0.2,
            friction_offset_threshold=0.01,
            friction_correlation_distance=0.00625,
            gpu_max_rigid_contact_count=2**23,
            gpu_max_rigid_patch_count=2**23,
            gpu_max_num_partitions=1,  # Important for stable simulation.
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=(0.3, 0, 0.1), target=(0, 0, 0.0))

    scene_cfg = FingerPadSceneCfg(
        num_envs=args_cli.num_envs,
        env_spacing=2 * max(sensor_size[:2]),
    )

    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print(scene["sensor"])

    viewer = vis_utils.ViewportTactileViewer(
        [scene["sensor"]], ["Pad"], enabled=not args_cli.no_tactile_ui and sim.has_gui()
    )
    sim_dt = sim.get_physics_dt()
    settle_timesteps = 100

    step = 0
    while simulation_app.is_running():
        if step % settle_timesteps == 0:
            root_state = scene["asset"].data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            root_state[:, 2] += 0.03

            scene["asset"].write_root_pose_to_sim(root_state[:, :7])
            scene["asset"].write_root_velocity_to_sim(root_state[:, 7:])

            scene.reset()
            print("reset")

        scene.write_data_to_sim()
        sim.step(render=sim.has_gui())
        scene.update(sim_dt)
        if step % 6 == 0:
            viewer.update()
        step += 1

        if step % 60 == 0:
            forces = scene["sensor"].data.normal_forces
            print(f"peak taxel force {forces.max().item():7.4f} N   total {forces[0].sum().item():7.4f} N")
        if args_cli.max_steps and step >= args_cli.max_steps:
            break

    viewer.close()


if __name__ == "__main__":
    # run the main function
    try:
        main()
    finally:
        context = SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True
        simulation_app.close()
