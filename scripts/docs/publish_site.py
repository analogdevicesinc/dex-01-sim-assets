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

"""Assemble versioned ADI documentation without replacing retained releases."""

import argparse
import hashlib
import html
import json
import re
import shutil
from pathlib import Path

VERSION = re.compile(r"v[0-9][A-Za-z0-9_.+-]*")
STABLE = re.compile(r"v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(?:\+[A-Za-z0-9.-]+)?")
PREVIEW = re.compile(r"pull/([1-9][0-9]*)")


def stable_version(version):
    match = STABLE.fullmatch(version)
    return tuple(map(int, match.groups())) if match else None


def validate_version(version):
    if version != "main" and not VERSION.fullmatch(version) and not PREVIEW.fullmatch(version):
        raise ValueError(f"Invalid documentation version: {version!r}")


def tree_hashes(directory):
    """Compare every retained release asset, including dotfiles."""
    result = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"Documentation may not contain symlinks: {path}")
        if path.is_file():
            result[str(path.relative_to(directory))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def install_version(site, artifact, version):
    validate_version(version)
    if not (artifact / "index.html").is_file() or not (artifact / "objects.inv").is_file():
        raise ValueError("Documentation artifact must include index.html and objects.inv")
    incoming = tree_hashes(artifact)
    destination = site / version
    if destination.is_symlink():
        raise ValueError(f"Version directory may not be a symlink: {destination}")
    if VERSION.fullmatch(version) and destination.exists():
        if incoming != tree_hashes(destination):
            raise ValueError(f"Refusing to overwrite retained release {version}")
        return
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(artifact, destination)


def published_versions(site):
    versions = [
        path.name
        for path in site.iterdir()
        if path.is_dir()
        and (path.name == "main" or VERSION.fullmatch(path.name))
        and (path / "index.html").is_file()
        and (path / "objects.inv").is_file()
    ]
    pull = site / "pull"
    if pull.is_dir():
        versions.extend(
            f"pull/{path.name}"
            for path in pull.iterdir()
            if PREVIEW.fullmatch(f"pull/{path.name}")
            and (path / "index.html").is_file()
            and (path / "objects.inv").is_file()
        )
    return sorted(versions)


def migrate_root(site):
    """Preserve a site produced by the previous root-replacing publisher."""
    index = site / "index.html"
    if not index.is_file() or not (site / "objects.inv").is_file():
        return
    match = re.search(r'<meta name="version" content="([^\"]*)"', index.read_text())
    if not match:
        raise ValueError("Cannot identify the existing root documentation version")
    version = html.unescape(match[1])
    validate_version(version)
    retained = {version.split("/")[0], ".git", "pull", "main"}
    retained.update(path.name for path in site.iterdir() if VERSION.fullmatch(path.name))
    destination = site / version
    if not destination.exists():
        destination.mkdir(parents=True)
        for path in list(site.iterdir()):
            if path.name not in retained and path.name not in {"tags.json", "sitemap.xml", ".nojekyll"}:
                shutil.move(str(path), destination / path.name)
    else:
        for path in list(site.iterdir()):
            if path.name not in retained:
                if path.is_dir():
                    shutil.rmtree(path)
                else:
                    path.unlink()


def assemble(site, artifact=None, version=None, historical=None, open_prs=()):
    site.mkdir(parents=True, exist_ok=True)
    migrate_root(site)
    if artifact is not None:
        validate_version(version)
        if not version.startswith("pull/") or int(version.split("/")[1]) in open_prs:
            install_version(site, artifact, version)
    # The historical prerelease uses old sources and is backfilled only once.
    if historical is not None and not (site / "v0.1.0-a.1").exists():
        install_version(site, historical, "v0.1.0-a.1")
    for preview in (site / "pull").glob("*"):
        if preview.name.isdigit() and int(preview.name) not in open_prs:
            shutil.rmtree(preview)
    versions = published_versions(site)
    stable = sorted((v for v in versions if stable_version(v)), key=lambda v: (stable_version(v), v), reverse=True)
    prereleases = sorted((v for v in versions if VERSION.fullmatch(v) and not stable_version(v)), reverse=True)
    previews = sorted((v for v in versions if v.startswith("pull/")), key=lambda v: int(v.split("/")[1]))
    labels = {v: [v, "latest stable" if i == 0 else "release"] for i, v in enumerate(stable)}
    if "main" in versions:
        labels["main"] = ["main", "development"]
    labels.update({v: [v, "prerelease"] for v in prereleases})
    labels.update({v: [f"PR #{v.split('/')[1]}", "preview"] for v in previews})
    (site / "tags.json").write_text(json.dumps(labels, indent=2) + "\n")
    (site / ".nojekyll").touch()
    # Before the first stable release, main is the default. A preview-only first
    # publication gets a landing page and never becomes the default docs.
    target = stable[0] if stable else "main" if "main" in versions else None
    if target:
        link = html.escape(target + "/", quote=True)
        page = (
            '<!doctype html><html lang="en"><head><meta charset="utf-8">'
            "<title>DEX-01 documentation</title>"
            f'<meta http-equiv="refresh" content="0; url={link}"></head><body>'
            f'<p><a href="{link}">Open DEX-01 documentation</a></p>'
            "<script>location.replace("
            + json.dumps(target + "/")
            + "+location.search+location.hash)</script></body></html>\n"
        )
    else:
        links = "".join(f'<li><a href="{html.escape(v)}/">{html.escape(labels[v][0])}</a></li>' for v in versions)
        page = (
            '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="robots"'
            ' content="noindex"><title>DEX-01 documentation</title></head><body><h1>DEX-01 documentation</h1><p>No'
            f" stable release published yet.</p><ul>{links}</ul></body></html>\n"
        )
    (site / "index.html").write_text(page)
    # PR previews are deliberately excluded from the public sitemap.
    base = "https://analogdevicesinc.github.io/dex-01-sim-assets/"
    locations = [base]
    for v in versions:
        if v.startswith("pull/"):
            continue
        for path in sorted((site / v).rglob("index.html")):
            locations.append(base + str(path.parent.relative_to(site)) + "/")
    urls = "".join(f"<url><loc>{html.escape(url)}</loc></url>" for url in locations)
    (site / "sitemap.xml").write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">'
        + urls
        + "</urlset>\n"
    )
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--artifact", type=Path)
    parser.add_argument("--version")
    parser.add_argument("--historical", type=Path)
    parser.add_argument("--open-prs", type=Path, required=True)
    args = parser.parse_args()
    if bool(args.artifact) != bool(args.version):
        parser.error("--artifact and --version must be provided together")
    open_prs = set(json.loads(args.open_prs.read_text()))
    target = assemble(args.site, args.artifact, args.version, args.historical, open_prs)
    print(f"Default documentation: {target or 'landing page'}")


if __name__ == "__main__":
    main()
