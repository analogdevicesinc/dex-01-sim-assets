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

"""Build the pre-documentation prerelease from its immutable Git sources."""

import argparse
import re
import shutil
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--tag", default="v0.1.0-a.1")
parser.add_argument("--output", type=Path, default=Path("docs/_build/historical"))
args = parser.parse_args()
if not re.fullmatch(r"v[0-9][A-Za-z0-9_.-]*", args.tag):
    parser.error("Expected a version tag beginning with v and a digit")


def tagged_file(name):
    return subprocess.check_output(["git", "show", f"{args.tag}:{name}"], text=True)


source = args.output.parent / "historical-src"
source.mkdir(parents=True, exist_ok=True)
documentation = Path(__file__).resolve().parents[1] / "docs"
shutil.copytree(documentation / "_static", source / "_static", dirs_exist_ok=True)
shutil.copytree(documentation / "_templates", source / "_templates", dirs_exist_ok=True)
readme = tagged_file("README.md")
base = f"https://github.com/analogdevicesinc/dex-01-sim-assets/blob/{args.tag}/"
readme = re.sub(r"\]\((?!https?://|#)([^)]+)\)", lambda m: f"]({base}{m[1]})", readme)
(source / "index.md").write_text(
    f"# {args.tag} — historical prerelease\n\n"
    "```{warning}\n"
    "This prerelease predates the documentation site. The content below is its\n"
    "original tagged README, not the current main documentation.\n"
    "```\n\n"
    + readme.replace("# DEX-01 Tactile Sensor", "## DEX-01 Tactile Sensor", 1)
)
(source / "conf.py").write_text(
    "project = 'DEX-01 Tactile Sensor'\n"
    "repository = 'dex-01-sim-assets'\n"
    "author = 'Analog Devices, Inc.'\n"
    "copyright = '2026, Analog Devices, Inc.'\n"
    f"version = {args.tag!r}\n"
    "extensions = ['adi_doctools', 'myst_parser', 'sphinx.ext.githubpages']\n"
    "source_suffix = {'.md': 'markdown'}\n"
    "html_theme = 'harmonic'\n"
    f"html_baseurl = 'https://analogdevicesinc.github.io/dex-01-sim-assets/{args.tag}/'\n"
    "templates_path = ['_templates']\n"
    "html_static_path = ['_static']\n"
    "html_js_files = [('repository-metadata.js', {'priority': 200})]\n"
    f"html_context = {{'docs_source_ref': {args.tag!r}}}\n"
)
subprocess.run(["sphinx-build", "-b", "dirhtml", "-W", "--keep-going", str(source), str(args.output)], check=True)
for filename in ("LICENSE", "NOTICE", "LICENSE_Tesollo"):
    (args.output / filename).write_text(tagged_file(filename))
