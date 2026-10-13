# Maintain and publish documentation

This site follows [ADI Doctools](https://analogdevicesinc.github.io/doctools/),
its [authoring guidelines](https://analogdevicesinc.github.io/doctools/docs_guidelines/),
and its [shared publishing guide](https://analogdevicesinc.github.io/doctools/deploy/).
Harmonic is ADI's current theme name; Cosmic is its former compatible name.

## Local authoring

From the repository root, create a documentation-only environment:

```bash
python3.11 -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r docs/requirements.txt
make -C docs html
python -m http.server 8000 --bind 127.0.0.1 --directory docs/_build/html
```

Open `http://localhost:8000/`. For editing with live reload:

```bash
adoc serve -d docs
```

This requires no Isaac Sim, GPU, CUDA, or simulation LFS files. Generated files
are ignored by Git. Keep documentation dependency upgrades in reviewed PRs.
The pinned ADI release and shared-action commit are intentional; do not replace
them with moving `latest` or `action` references.

Author Markdown with MyST. Use meaningful section labels and toctrees, explicit
units, figures with captions and alt text, and small readable code examples.
Keep standard ADI styling. The HTML target uses Sphinx’s `dirhtml` builder
for directory URLs compatible with ADI’s search result links. Do not set the
theme’s `landing_page` option: it disables search for local documentation content.

Reference pages use curated interfaces and source excerpts rather than runtime
imports. Update them alongside interface changes. License and contribution
files at the repository root remain canonical.

## Media

The README and homepage share `docs/media/dex01-demo.webp`. Capture from the
current PhysX demo and retain its provenance in
`docs/media/README.md`. Keep images below 2 MB and ordinary Git files; the
existing simulation asset LFS rules do not cover documentation media.
Link larger recordings instead of shipping them with the site. Captions must
identify the force units and the fixture used.

## One-time repository setup

1. In **Settings → Pages**, choose **GitHub Actions** before the first deployment.
   ADI's shared action maintains versioned output in `gh-pages`; the official
   Pages actions explicitly upload and deploy it. A `GITHUB_TOKEN` branch push
   alone does not trigger a Pages rebuild.
2. Ensure organization policy permits the pinned ADI actions and their nested
   GitHub/ADI actions. Publishing uses `GITHUB_TOKEN` with `contents: write`,
   `pull-requests: read`, `pages: write`, and `id-token: write`; builds use read
   permissions. In **Settings → Environments → github-pages**, use **No
   restriction** for deployment branches and tags so merged main builds,
   version tags, and same-repository PR previews can deploy. The workflow
   itself limits deploy events and rejects forks; a main-only environment
   blocks previews. Apply any organization-required reviewer policy here.
3. Merge the documentation change into `main`. The Documentation workflow
   builds the current site and historical prerelease, creates `gh-pages`, and
   deploys the complete artifact. If setup happens after the first merge,
   rerun the workflow's failed deployment job after changing Pages settings.
4. Require **Build documentation** in the main branch protection/ruleset.
5. After the site is available, set the repository About website to
   `https://analogdevicesinc.github.io/dex-01-sim-assets/`.
6. Open the root and release version, search for “taxel,” switch light/dark
   themes, and check a narrow mobile window. Open a same-repository PR and
   verify its hosted preview; close it and verify preview removal. A fork PR
   must produce only the downloadable build artifact.

No registration in ADI's central documentation lookup table is required to
build or publish. A small metadata adapter supplies this repo’s search and
source-edit entry while retaining the standard ADI theme and organization
navigation. GitHub Pages is this repo’s canonical site; the adapter omits the
generic ADI mirror redirect prompt because no developer.analog.com mirror exists.
Registration can be requested separately for discoverability.

## Publishing behavior

| Event | Output |
|---|---|
| Push to `main` | Current root site; also backfills the historical prerelease |
| Push of `v` followed by a digit | Root site and snapshot under the tag path |
| Same-repository PR | Build artifact and hosted `pull/<number>/` preview |
| Fork PR | Read-only build and downloadable artifact; no publishing |
| Same-repository PR closes | Its hosted preview is removed |

Hosted PR previews carry `noindex` metadata. Retrieve downloadable previews
from the workflow run's `html` artifact; extract it and serve it with
`python -m http.server --bind 127.0.0.1`. The site URL is under
`https://analogdevicesinc.github.io/dex-01-sim-assets/pull/<number>/`.

ADI’s publisher maintains `.nojekyll`, `objects.inv`, and `tags.json`. Before
the Pages artifact upload, the workflow uses ADI’s supported fine-grained
metadata format to label prereleases explicitly and PR entries as previews.
Publishing and cleanup share a concurrency group and explicitly deploy the
complete prepared site. Main/tag deployments replace the root while preserving
version directories. A main push after a release therefore makes the root
main again; always consult the displayed version when reproducing a result.

A same-repository documentation PR can be the first publication: its preview
is available at `pull/<number>/` even before main has been built. The version
metadata does not invent a main entry until the root site exists.
The workflow checks the current PR state before publishing and before Pages
deployment, so a build queued behind preview cleanup cannot restore a closed PR.

The existing `v0.1.0-a.1` tag predates the site. The historical builder reads
its original README and licenses with `git show`, labels it as a historical
prerelease, and builds it with the pinned ADI theme. It does not change the tag
or its runtime code. Main builds include that snapshot inside the current
artifact and publish both in one deployment. New release tags include the full site.

## Reproduce the figures

Use the scene-capture utility in the simulation environment:

```bash
python -m pip install scipy rtree matplotlib
MPLBACKEND=Agg python scripts/docs/export_scene.py \
  --output /tmp/dex01-docs-state \
  --steps 240
python scripts/docs/plot_validation.py \
  --hand-report /tmp/dex01-hand.json \
  --output docs/media
```

The scene exporter runs the unchanged hand demo with a local 0.1 kg cube,
exports live PhysX body transforms, verifies five visible fingertip meshes,
checks every frame against its taxel values, and measures the world-space
distance of every taxel to its visible exterior (acceptance: 1 µm). Its manifest
records the source revision and output hashes. It does not advance a second physics engine.
The plotting utility runs the independent hand-report checker before creating
figures. It takes the report from the validation guide and the released NPZ
pattern to generate the force and mapping figures. `docs/media/figures.json`
records input and figure hashes plus the Python, NumPy, and Matplotlib versions.
Repeating generation with identical inputs and plotting versions produces
identical SVG files; axes include the complete impact transient.

## Render the captured scene

The capture can be rendered without advancing physics:

```bash
python scripts/docs/render_scene.py \
  --state /tmp/dex01-docs-state \
  --output /tmp/dex01-docs-render
```

Use the installed Isaac Sim 5.1 environment on its validated R580 driver,
or a separate renderer environment. The figures in this repository were
rendered with **Isaac Sim 6.0.1.0 / Python 3.12**, using the captured 5.1 physics
state and force arrays. To reproduce that renderer setup without changing the
simulation environment:

```bash
uv venv --python 3.12 --seed .venv-render
.venv-render/bin/python -m pip install \
  "isaacsim[all,extscache]==6.0.1.0" \
  --extra-index-url https://pypi.nvidia.com
.venv-render/bin/python scripts/docs/render_scene.py \
  --state /tmp/dex01-docs-state \
  --output /tmp/dex01-docs-render
```

The first renderer launch requires accepting NVIDIA's Omniverse license.
A supervising process checks the worker's exit, image dimensions, and output
hashes before accepting the render. The camera worker keeps the timeline
stopped and verifies every captured rigid-body transform. It produces a hand
view, a close-up of the loaded fingertip, the matching tactile composite, and a
render manifest. Camera, lighting, and neutral ground shading affect
presentation only; force values come from the captured 5.1 scene.

Keep large scene exports and recordings outside the repository. Once a camera
capture is checked, copy its optimized WebP into `docs/media/`, record physics
and renderer versions in the provenance, and inspect every fingertip, the
loaded object, and all five frame labels before accepting it.

## Checks and recovery

Run `make -C docs html` before review: Sphinx warnings fail the build. Check
internal navigation and GitHub README links as well as rendered pages. Run
`make -C docs linkcheck` separately; the scheduled/manual external-link workflow
reports failures without blocking every PR. Do not broadly ignore domains;
document a narrow exception only when a URL is confirmed valid but blocked.

After the first deployment, verify preview creation with a same-repository PR
and verify removal after it closes. A fork PR must never run the deployment job.
Confirm that a later main deployment retains release paths and `tags.json`.

If a build fails, the deployment job is skipped. If publishing fails, inspect
the Actions log and token/organization settings, then rerun the failed jobs.
To recover bad content, revert its source change and merge the revert into main.
To restore older published output, use a reviewed recovery change based on the known-good
source revision. Keep existing release directories when recovering the site.
