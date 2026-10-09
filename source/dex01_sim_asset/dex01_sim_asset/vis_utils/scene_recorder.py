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


class SceneRecorder:
    """Buffers RGB and taxel-force frames for one environment, then hands them to a visualizer.

    Call :meth:`step` once per simulation step and :meth:`save_video` at the end.

    Recording requires the simulation to run with ``--enable_cameras``, and the ffmpeg writer
    requires ffmpeg on the PATH. Turn off ``debug_vis`` on the sensors first, otherwise the taxel
    markers appear in the recorded RGB frames.
    """

    def __init__(self, camera, sensors: list, visualizer, env_id: int = 0):
        self.env_id = env_id

        self.visualizer = visualizer
        self.camera = camera
        self.sensors = sensors

        self.sensor_frames = [[] for _ in sensors]
        self.cam_frames = []

    def reset_buffers(self):
        self.sensor_frames = [[] for _ in self.sensors]
        self.cam_frames = []

    def step(self):
        self.cam_frames.append(self.camera.data.output["rgb"][self.env_id].clone().cpu().numpy())

        for si, sensor in enumerate(self.sensors):
            tac_image = sensor.get_tactile_image()[self.env_id].clone().cpu().numpy()
            self.sensor_frames[si].append(tac_image)

    def save_video(self, name: str, max_force: float = None, start=30):
        print(f"[INFO]: Saving video with {len(self.cam_frames) - 1} frames...")
        path = self.visualizer(self.cam_frames, self.sensor_frames, name=name, max_force=max_force, start=start)
        print(f"[INFO]: Video saved at {path}")

    def __len__(self):
        return len(self.cam_frames)
