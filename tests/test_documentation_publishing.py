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

"""Exercise version retention, default selection, migration, and preview cleanup."""

import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "publish_site", Path(__file__).resolve().parents[1] / "scripts/docs/publish_site.py"
)
publisher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(publisher)


def artifact(tmp_path, version, content=None):
    root = tmp_path / ("artifact-" + version.replace("/", "-"))
    root.mkdir(exist_ok=True)
    (root / "index.html").write_text(f'<meta name="version" content="{version}">{content or version}')
    (root / "objects.inv").write_bytes(b"inventory")
    (root / "_static").mkdir(exist_ok=True)
    (root / "_static/theme.js").write_text(content or version)
    (root / ".nojekyll").touch()
    (root / "guides").mkdir(exist_ok=True)
    (root / "guides/index.html").write_text(version)
    return root


def tags(site):
    return json.loads((site / "tags.json").read_text())


def test_lifecycle_keeps_stable_default_and_all_release_bytes(tmp_path):
    site = tmp_path / "site"
    main = artifact(tmp_path, "main")
    assert publisher.assemble(site, main, "main") == "main"
    assert "url=main/" in (site / "index.html").read_text()
    release = artifact(tmp_path, "v0.2.0")
    assert publisher.assemble(site, release, "v0.2.0") == "v0.2.0"
    before = publisher.tree_hashes(site / "v0.2.0")
    (main / "_static/stale.js").write_text("stale")
    publisher.assemble(site, main, "main")
    (main / "_static/stale.js").unlink()
    publisher.assemble(site, main, "main")
    assert not (site / "main/_static/stale.js").exists()
    assert publisher.tree_hashes(site / "v0.2.0") == before
    publisher.assemble(site, artifact(tmp_path, "v0.10.0"), "v0.10.0")
    publisher.assemble(site, artifact(tmp_path, "v0.3.0"), "v0.3.0")
    publisher.assemble(site, artifact(tmp_path, "v1.0.0-rc.1"), "v1.0.0-rc.1")
    assert "url=v0.10.0/" in (site / "index.html").read_text()
    assert tags(site)["v0.10.0"] == ["v0.10.0", "latest stable"]
    assert tags(site)["main"] == ["main", "development"]
    assert tags(site)["v1.0.0-rc.1"] == ["v1.0.0-rc.1", "prerelease"]
    assert publisher.tree_hashes(site / "v0.2.0") == before
    publisher.assemble(site, artifact(tmp_path, "pull/2"), "pull/2", open_prs={2})
    assert tags(site)["pull/2"] == ["PR #2", "preview"]
    assert "pull/2" not in (site / "sitemap.xml").read_text()
    assert "url=v0.10.0/" in (site / "index.html").read_text()
    publisher.assemble(site, open_prs=set())
    assert not (site / "pull/2").exists()
    assert "pull/2" not in tags(site)
    # A build queued after closing must not resurrect the preview.
    publisher.assemble(site, artifact(tmp_path, "pull/2"), "pull/2", open_prs=set())
    assert not (site / "pull/2").exists()
    assert publisher.tree_hashes(site / "v0.2.0") == before


def test_release_rerun_does_not_overwrite_retained_release(tmp_path):
    site = tmp_path / "site"
    release = artifact(tmp_path, "v0.2.0")
    publisher.assemble(site, release, "v0.2.0")
    publisher.assemble(site, release, "v0.2.0")
    (release / "index.html").write_text("different")
    with pytest.raises(ValueError, match="Refusing to overwrite"):
        publisher.assemble(site, release, "v0.2.0")
    assert "v0.2.0" in (site / "v0.2.0/index.html").read_text()


@pytest.mark.parametrize("version", ["main", "v0.2.0", "v0.1.0-a.1"])
def test_migrates_previous_root_without_losing_releases_or_git(tmp_path, version):
    import shutil

    site = tmp_path / "site"
    shutil.copytree(artifact(tmp_path, version), site)
    (site / ".git").mkdir()
    (site / ".git/config").write_text("git settings")
    historical = artifact(tmp_path, "v0.1.0-a.1")
    if version != "v0.1.0-a.1":
        shutil.copytree(historical, site / "v0.1.0-a.1")
    publisher.assemble(site, historical=historical)
    assert (site / version / "objects.inv").exists()
    assert (site / "v0.1.0-a.1/index.html").exists()
    assert not (site / "objects.inv").exists()
    assert (site / ".git/config").read_text() == "git settings"
    assert "" not in tags(site)


def test_preview_only_site_and_historical_backfill(tmp_path):
    site = tmp_path / "site"
    assert publisher.assemble(site, artifact(tmp_path, "pull/2"), "pull/2", open_prs={2}) is None
    assert "No stable release" in (site / "index.html").read_text()
    assert "http-equiv" not in (site / "index.html").read_text()
    historical = artifact(tmp_path, "v0.1.0-a.1")
    publisher.assemble(site, artifact(tmp_path, "main"), "main", historical, {2})
    before = publisher.tree_hashes(site / "v0.1.0-a.1")
    (historical / "index.html").write_text("rebuilt theme")
    publisher.assemble(site, historical=historical, open_prs={2})
    assert publisher.tree_hashes(site / "v0.1.0-a.1") == before
    assert "url=main/" in (site / "index.html").read_text()


@pytest.mark.parametrize("version", ["../outside", "pull/0", "pull/2/../../outside", "v1/../../outside", ""])
def test_invalid_paths_are_rejected(tmp_path, version):
    with pytest.raises(ValueError, match="Invalid documentation version"):
        publisher.assemble(tmp_path / "site", artifact(tmp_path, "main"), version)


def test_rejects_incomplete_or_symlinked_artifacts(tmp_path):
    incoming = artifact(tmp_path, "main")
    (incoming / "objects.inv").unlink()
    with pytest.raises(ValueError, match="objects.inv"):
        publisher.assemble(tmp_path / "site", incoming, "main")
    (incoming / "objects.inv").write_bytes(b"inventory")
    (incoming / "unsafe").symlink_to(tmp_path)
    with pytest.raises(ValueError, match="symlinks"):
        publisher.assemble(tmp_path / "site", incoming, "main")
