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
metadata = Path(__file__).parent.parent / "common/source/dex01_sim_common/pyproject.toml"
release = tomllib.loads(metadata.read_text())["project"]["version"]
version = os.environ.get("ADOC_DOC_VERSION", "main")
extensions = ["adi_doctools", "myst_parser", "sphinx.ext.githubpages"]
needs_extensions = {"adi_doctools": "0.5.1"}
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
myst_enable_extensions = ["colon_fence"]
myst_heading_anchors = 3
exclude_patterns = ["_build", "README.md", "media/README.md", "tests/**"]
templates_path = ["_templates"]
html_static_path = ["_static"]
html_css_files = ["example-media.css"]
html_js_files = [("repository-metadata.js", {"priority": 200}), ("environment-navigation.js", {"priority": 210})]
html_context = {"docs_source_ref": os.environ.get("DOCS_SOURCE_REF", "main"), "environment_navigation": True}
html_theme = "harmonic"
html_title = "DEX-01 Tactile Sensor documentation"
html_theme_options = {"no_index": os.environ.get("DOCS_PREVIEW") == "true"}
html_baseurl = f"https://analogdevicesinc.github.io/dex-01-sim-assets/{version}/"
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


def write_legacy_redirects(app, exception):
    """Preserve existing page URLs without keeping obsolete workflows in search."""
    if exception is not None or app.builder.name != "dirhtml":
        return
    from html import escape

    redirects = {
        "getting-started/installation": "../../physx/installation",
        "getting-started/quickstart": "../../physx/quickstart",
        "getting-started/newton": "../../newton",
        "guides/attachment": "../../physx/attachment",
        "guides/read-data": "../../reference/data",
        "guides/patterns": "../../concepts/patterns",
        "reference/numerical-settings": "../../physx/numerical-settings",
        "examples/flat-pad": "../../physx/examples/flat-pad",
        "examples/hand": "../../physx/examples/hand",
        "examples/stationary-press": "../../physx/examples/stationary-press",
    }
    for source, target in redirects.items():
        directory = Path(app.outdir) / source
        directory.mkdir(parents=True, exist_ok=True)
        url = escape(target.rstrip("/") + "/", quote=True)
        (directory / "index.html").write_text(
            '<!doctype html><html lang="en"><head><meta charset="utf-8">'
            f'<meta http-equiv="refresh" content="0; url={url}">'
            "<title>Documentation moved</title></head><body>"
            f'<a href="{url}">Continue to the environment documentation</a></body></html>'
        )


def scope_environment_navigation(app, pagename, templatename, context, doctree):
    """Keep the active environment's workflow together in the ADI sidebar."""
    environment = pagename.split("/", 1)[0]
    if environment not in ("physx", "newton"):
        return
    from adi_doctools.theme import navigation_tree
    from sphinx import addnodes
    from sphinx.environment.adapters.toctree import TocTree

    fragments = []
    for tree in app.env.get_doctree(environment + "/index").findall(addnodes.toctree):
        resolved = TocTree(app.env).resolve(
            pagename, app.builder, tree, titles_only=True, maxdepth=-1, collapse=False, includehidden=True
        )
        if resolved is not None:
            fragments.append(app.builder.render_partial(resolved)["fragment"])
    context["active_environment"] = environment
    context["environment_toc_tree"] = navigation_tree(app, "".join(fragments), context["content_root"], pagename)[0]


def setup(app):
    app.connect("doctree-resolved", resolve_source_links)
    app.connect("html-page-context", scope_environment_navigation, priority=800)
    app.connect("build-finished", write_legacy_redirects)
