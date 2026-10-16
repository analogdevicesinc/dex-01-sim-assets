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

"""Verify complete environment journeys and built navigation without a simulator."""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"


@pytest.mark.parametrize("environment", ["physx", "newton"])
def test_environment_journey_has_one_home_and_complete_topic_pages(environment):
    pages = (
        "installation",
        "quickstart",
        "attachment",
        "api",
        "numerical-settings",
        "validation",
        "recording",
        "troubleshooting",
    )
    home = (DOCS / environment / "index.md").read_text()
    for page in pages:
        assert (DOCS / environment / (page + ".md")).is_file()
        assert re.search(r"^" + re.escape(page) + r"$", home, re.MULTILINE)
    assert (DOCS / environment / "examples/index.md").is_file()


@pytest.mark.parametrize("environment", ["physx", "newton"])
def test_documented_backend_commands_exist_in_target_directory(environment):
    count = 0
    for path in (DOCS / environment).rglob("*.md"):
        for command in re.findall(r"(?:python|python3) ((?:\.\./)?scripts/[^\s\\]+\.py)", path.read_text()):
            assert (ROOT / environment / command).is_file(), (path, command)
            count += 1
    assert count >= 5


def test_environment_entry_pages_are_separate_from_shared_references():
    home = (DOCS / "index.md").read_text()
    assert "physx/index" in home and "newton/index" in home
    assert "getting-started/installation" not in home
    assert not (ROOT / "setup_env.sh").exists()
    assert not (ROOT / "setup_newton_env.sh").exists()
    assert "dex01_sim_common" in (DOCS / "reference/data.md").read_text()


@pytest.fixture(scope="module")
def built_site(tmp_path_factory):
    import io

    from sphinx.application import Sphinx

    destination = tmp_path_factory.mktemp("docs-site")
    warnings = io.StringIO()
    app = Sphinx(
        str(DOCS),
        str(DOCS),
        str(destination / "html"),
        str(destination / "doctrees"),
        "dirhtml",
        status=io.StringIO(),
        warning=warnings,
        warningiserror=True,
        freshenv=True,
    )
    app.build(force_all=True)
    assert app.statuscode == 0, warnings.getvalue()
    return destination / "html"


@pytest.mark.parametrize("environment,other", [("physx", "newton"), ("newton", "physx")])
def test_built_sidebar_keeps_the_selected_environment_journey(built_site, environment, other):
    from lxml import html

    document = html.fromstring((built_site / environment / "examples/hand/index.html").read_text())
    sidebar = document.xpath('//div[contains(concat(" ", normalize-space(@class), " "), " toc-tree ")]')[0]
    links = [a.get("href") for a in sidebar.xpath(".//a[@href]")]
    assert any("installation/" in link for link in links)
    assert any("validation/" in link for link in links)
    assert any("reference/data/" in link for link in links)
    assert not any(other + "/" in link for link in links)
    switch = document.xpath('//nav[@aria-label="Simulation environment"]')[0]
    assert switch.xpath('.//a[@aria-current="true"]/text()') == (["PhysX"] if environment == "physx" else ["Newton"])


def test_legacy_urls_redirect_without_duplicate_search_documents(built_site):
    from lxml import html

    redirect = html.fromstring((built_site / "getting-started/newton/index.html").read_text())
    assert redirect.xpath('//meta[@http-equiv="refresh"]')
    assert "../../newton/" in redirect.xpath("//a/@href")[0]
    sources = list((built_site / "_sources").rglob("*.txt"))
    assert not any("getting-started" in source.parts for source in sources)
