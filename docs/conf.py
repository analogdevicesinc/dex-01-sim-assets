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

"""Lightweight ADI documentation configuration; never import the simulator."""

import os
import tomllib
from pathlib import Path

repository = "dex-01-sim-assets"
project = "DEX-01 Tactile Sensor"
author = "Analog Devices, Inc."
copyright = "2026, Analog Devices, Inc."
metadata = Path(__file__).parent.parent / "source/dex01_sim_asset/config/extension.toml"
release = tomllib.loads(metadata.read_text())["package"]["version"]
version = os.environ.get("ADOC_DOC_VERSION", "main")
extensions = ["adi_doctools", "myst_parser", "sphinx.ext.githubpages"]
needs_extensions = {"adi_doctools": "0.5.1"}
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
myst_enable_extensions = ["colon_fence"]
myst_heading_anchors = 3
exclude_patterns = ["_build", "README.md", "media/README.md"]
templates_path = ["_templates"]
html_static_path = ["_static"]
html_css_files = ["example-media.css"]
html_js_files = [("repository-metadata.js", {"priority": 200})]
html_context = {"docs_source_ref": os.environ.get("DOCS_SOURCE_REF", "main")}
html_theme = "harmonic"
html_title = "DEX-01 Tactile Sensor documentation"
html_theme_options = {"no_index": os.environ.get("DOCS_PREVIEW") == "true"}
html_baseurl = "https://analogdevicesinc.github.io/dex-01-sim-assets/"
html_extra_path = ["../LICENSE", "../NOTICE", "../LICENSE_Tesollo"]
linkcheck_timeout = 15
linkcheck_retries = 2
linkcheck_rate_limit_timeout = 30.0
linkcheck_anchors = False


def resolve_source_links(app, doctree, docname):
    """Keep reference and included contribution links aligned with the build."""
    from docutils import nodes

    source_ref = os.environ.get("DOCS_SOURCE_REF", "main")
    base = "https://github.com/analogdevicesinc/dex-01-sim-assets/blob/"
    for node in doctree.findall(nodes.reference):
        uri = node.get("refuri", "")
        if uri.startswith(base + "main/"):
            node["refuri"] = base + source_ref + "/" + uri[len(base + "main/") :]


def setup(app):
    app.connect("doctree-resolved", resolve_source_links)
