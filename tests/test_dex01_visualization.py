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

"""Check that displayed taxel frames preserve every occupied firmware cell."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_visualizers():
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    path = ROOT / "source/dex01_sim_asset/dex01_sim_asset/vis_utils/visualizers.py"
    spec = importlib.util.spec_from_file_location("_recorded_visualizers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("count,start", [(1, 0), (3, 0), (3, 1)])
def test_recording_includes_final_frame(tmp_path, count, start):
    from PIL import Image

    module = load_visualizers()
    camera = [np.full((8, 8, 3), 20 + i * 90, dtype=np.uint8) for i in range(count)]
    sensors = [[np.full((32, 32), i * 0.01) for i in range(count)] for _ in range(5)]
    visualizer = module.FiveFingerDex01Visualizer(writer="gif", fps=20)
    result = visualizer(camera, sensors, str(tmp_path / "recording"), start=start)
    with Image.open(result) as video:
        assert video.n_frames == count - start


def test_recording_rejects_misaligned_sequences(tmp_path):
    module = load_visualizers()
    visualizer = module.FiveFingerDex01Visualizer(writer="gif")
    with pytest.raises(ValueError, match="camera frame count"):
        visualizer([np.zeros((8, 8, 3), dtype=np.uint8)] * 2, [[np.zeros((32, 32))]] * 5, str(tmp_path / "bad"))
    with pytest.raises(ValueError, match="existing frame"):
        visualizer([], [[]] * 5, str(tmp_path / "empty"))


def test_display_mask_preserves_all_740_cells_and_hand_plot():
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = ROOT / "source/dex01_sim_asset/dex01_sim_asset/vis_utils/visualizers.py"
    spec = importlib.util.spec_from_file_location("_dex01_visualizers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with np.load(ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz") as data:
        pixels = data["taxel2pixel"]
    frame = np.arange(1024, dtype=np.float32).reshape(32, 32)
    displayed, mask = module.prettify_dex01(frame)
    assert mask.sum() == 740
    assert mask[pixels[:, 0], pixels[:, 1]].all()
    assert np.array_equal(displayed, frame)
    fig, ax = plt.subplots()
    images = module.dex01_hand_plot(fig, ax, [frame] * 5)
    assert len(images) == 5
    for image in images:
        plotted = image.get_array()
        assert np.array_equal(plotted.data, frame)
        assert np.array_equal(plotted.mask, ~mask)
    plt.close(fig)
    with pytest.raises(ValueError, match="32, 32"):
        module.prettify_dex01(np.zeros((31, 32)))
