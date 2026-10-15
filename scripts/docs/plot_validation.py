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

"""Create documentation figures from checked reports and the released taxel pattern."""

import argparse
import hashlib
import json
import subprocess
import sys
from importlib.metadata import version
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("docs/media"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[2]
    report = json.loads(args.hand_report.read_text())
    if report.get("indenter") != "flat" or report["taxels_per_finger"] != [738] * 5:
        parser.error("Expected the checked five-finger flat-load report")
    inputs = {
        "pattern_sha256": repo / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz",
        "contact_pattern_sha256": repo / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_contact.npz",
        "asset_sha256": repo / "assets/robots/tesollo/dex01.usda",
        "hand_asset_sha256": repo / "assets/robots/tesollo/dg5f_right.usda",
    }
    for field, path in inputs.items():
        if report.get(field) != hashlib.sha256(path.read_bytes()).hexdigest():
            parser.error(f"The hand report does not describe this checkout's {field}")
    checker = repo / "scripts/verify_dex01_sim.py"
    subprocess.run(
        [sys.executable, str(checker), "--check-hand-report", str(args.hand_report.resolve())],
        check=True,
    )
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 11,
        "svg.fonttype": "none",
        "svg.hashsalt": "dex01-documentation",
    })
    expected = 0.1 * 9.81
    cycle_steps = report["steps_per_finger"]
    fig, axes = plt.subplots(2, 4, figsize=(12, 6.5), sharey="row")
    colors = ["#0067b9", "#00877a", "#6e56a5", "#b66800"]
    peak = max(max(entry["forces_n"]) for entry in report["observations"])
    full_limit = max(expected * 1.2, peak * 1.08)
    settled_values = [
        entry["forces_n"][entry["target_finger"] - 1]
        for entry in report["observations"]
        if entry["step"] % cycle_steps >= cycle_steps * 0.8
    ]
    lower = min(min(settled_values), expected * 0.96)
    upper = max(max(settled_values), expected * 1.04)
    for column, (finger, color) in enumerate(zip(range(2, 6), colors)):
        ax = axes[0, column]
        steady_ax = axes[1, column]
        entries = [entry for entry in report["observations"] if entry["target_finger"] == finger]
        time = np.array([entry["step"] % cycle_steps for entry in entries]) * report["physics_dt_s"]
        load = np.array([entry["forces_n"][finger - 1] for entry in entries])
        ax.plot(time, load, color=color, linewidth=1.6)
        ax.axhline(expected, color="#34495e", linestyle="--", linewidth=1)
        settled = np.array([entry["step"] % cycle_steps >= cycle_steps * 0.8 for entry in entries])
        ax.set_title(f"Finger {finger}\nsettled median {np.median(load[settled]):.4f} N")
        ax.set_xlabel("Time in load cycle (s)")
        ax.set_ylim(0, full_limit)
        steady_ax.plot(time[settled], load[settled], color=color, linewidth=1.6)
        steady_ax.axhline(expected, color="#34495e", linestyle="--", linewidth=1)
        steady_ax.set_ylim(lower, upper)
        steady_ax.set_xlabel("Time in load cycle (s)")
        steady_ax.set_title("Settled load · final 20%")
        steady_ax.grid(alpha=0.2)
        steady_ax.spines[["top", "right"]].set_visible(False)
        ax.grid(alpha=0.2)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0, 0].set_ylabel("Full transient load (N)")
    axes[1, 0].set_ylabel("Settled normal load (N)")
    fig.suptitle("Physical-force reference: 0.1 kg × 9.81 m/s² = 0.981 N", fontsize=14)
    fig.tight_layout()
    fig.savefig(args.output / "known-load.svg", bbox_inches="tight", facecolor="white", metadata={"Date": None})
    plt.close(fig)
    pattern = repo / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_contact.npz"
    with np.load(pattern, allow_pickle=False) as data:
        poses, pixels = data["poses"], data["taxel2pixel"]
    if poses.shape != (738, 3, 4) or pixels.shape != (738, 2) or len(np.unique(pixels, axis=0)) != 738:
        raise ValueError("Expected the complete released 738-taxel layout")
    points = poses[:, :, 3] * 1000
    fig = plt.figure(figsize=(10, 4.6))
    surface = fig.add_subplot(121, projection="3d")
    surface.scatter(*points.T, c=pixels[:, 0], cmap="viridis", s=5, depthshade=False)
    surface.set_xlabel("x (mm)")
    surface.set_ylabel("y (mm)")
    surface.set_zlabel("z (mm)")
    surface.set_title("738 samples on the exterior\nSensor pattern coordinates")
    surface.view_init(elev=25, azim=-65)
    surface.set_box_aspect(np.ptp(points, axis=0))
    frame = fig.add_subplot(122)
    mask = np.zeros((32, 32), dtype=bool)
    mask[pixels[:, 0], pixels[:, 1]] = True
    values = np.broadcast_to(np.arange(32)[:, None], (32, 32))
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#d8dfe5")
    frame.imshow(np.ma.array(values, mask=~mask), cmap=cmap, interpolation="nearest")
    frame.set_xlabel("Firmware column")
    frame.set_ylabel("Firmware row")
    frame.set_title("Same taxel order in a 32 × 32 frame\nGrey = 286 unoccupied pixels")
    frame.set_xticks([0, 8, 16, 24, 31])
    frame.set_yticks([0, 8, 16, 24, 31])
    fig.tight_layout()
    fig.savefig(args.output / "taxel-mapping.svg", bbox_inches="tight", facecolor="white", metadata={"Date": None})
    plt.close(fig)
    for name in ("known-load.svg", "taxel-mapping.svg"):
        path = args.output / name
        path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    provenance = {
        "hand_report_sha256": hashlib.sha256(args.hand_report.read_bytes()).hexdigest(),
        "force_checker_sha256": hashlib.sha256(checker.read_bytes()).hexdigest(),
        "pattern_sha256": hashlib.sha256(pattern.read_bytes()).hexdigest(),
        "full_transient_peak_n": peak,
        "known_load_reference_n": expected,
        "plotting_environment": {
            "python": sys.version.split()[0],
            "numpy": version("numpy"),
            "matplotlib": version("matplotlib"),
        },
        "figures": {
            name: hashlib.sha256((args.output / name).read_bytes()).hexdigest()
            for name in ("known-load.svg", "taxel-mapping.svg")
        },
    }
    (args.output / "figures.json").write_text(json.dumps(provenance, indent=2) + "\n")


if __name__ == "__main__":
    main()
