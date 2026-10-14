# Documentation CI and GitHub Pages

Maintainer configuration for the Documentation workflows. Local authoring and
media reproduction are covered in `DOCUMENTATION.md` at the repository root.
ADI's publisher follows its [shared publishing guide](https://analogdevicesinc.github.io/doctools/deploy/).

## One-time repository setup

1. In **Settings → Pages**, choose **GitHub Actions** before the first deployment.
   ADI's shared action maintains versioned output in `gh-pages`; the official
   Pages actions explicitly upload and deploy it. A `GITHUB_TOKEN` branch push
   alone does not trigger a Pages rebuild.
2. Ensure organization policy permits the pinned ADI actions and their nested
   GitHub/ADI actions. Publishing uses `GITHUB_TOKEN` with `contents: write`,
   `pull-requests: read`, `pages: write`, and `id-token: write`; builds use read
   permissions. In **Settings → Environments → github-pages**, select
   **Selected branches and tags**. Keep the `main` branch rule, add a branch
   rule for `refs/pull/*/merge`, and add a tag rule for `v[0-9]*`. PR workflows
   deploy from their merge ref, not the source branch; a main-only rule rejects
   every preview before any deployment step runs.
   The wildcard covers future PRs as well. The workflow rejects fork previews.
   Apply any organization-required reviewer policy here.
3. Rerun the failed PR deployment after adding its environment rule and open
   `https://analogdevicesinc.github.io/dex-01-sim-assets/pull/<number>/` for
   review. The successful publication job also links the preview in its summary.
   After review, merge the documentation change into `main`. The workflow
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
