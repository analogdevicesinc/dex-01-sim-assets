# Documentation CI and GitHub Pages

Maintainer configuration for the Documentation workflows. Local authoring and
media reproduction are covered in `DOCUMENTATION.md` at the repository root.
The site uses ADI's Harmonic theme and its `tags.json` version-menu format.
The repository's local publishing action assembles the complete site so that
development pushes cannot replace stable releases.

## One-time repository setup

1. In **Settings → Pages**, choose **GitHub Actions** before the first deployment.
   The local publishing action maintains versioned output in `gh-pages`; the official
   Pages actions explicitly upload and deploy it. A `GITHUB_TOKEN` branch push
   alone does not trigger a Pages rebuild.
2. Ensure organization policy permits the official GitHub actions used in the
   workflows. Publishing uses `GITHUB_TOKEN` with `contents: write`,
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
| Push to `main` | Replace `/main/`; backfill the historical prerelease once |
| New stable `vMAJOR.MINOR.PATCH` tag | Retain docs at the tag path; highest stable version becomes the default |
| New prerelease tag | Retain docs at the tag path; keep the stable default |
| Same-repository PR | Build artifact and hosted `pull/<number>/` preview |
| Fork PR | Read-only build and downloadable artifact; no publishing |
| Same-repository PR closes | Its hosted preview is removed |

Hosted PR previews carry `noindex` metadata. Retrieve downloadable previews
from the workflow run's `html` artifact; extract it and serve it with
`python -m http.server --bind 127.0.0.1`. The site URL is under
`https://analogdevicesinc.github.io/dex-01-sim-assets/pull/<number>/`.

The root `/` redirects to the highest published stable semantic version,
ordered numerically by major, minor, and patch. For example, `v0.10.0` is newer
than `v0.3.0`. Before any stable release exists, it redirects to `/main/`.
A site containing only previews or historical prereleases shows a landing page
instead of making a PR preview the default. This selection is based on the
version tag, not the order of deployments or the GitHub Release “latest” flag.

The publisher updates only the intended version directory. Main and PR builds
replace their directory completely, removing stale pages and assets. Release
directories are retained unchanged. A rerun with identical release output is
allowed; different output for an existing tag fails instead of overwriting it.
Publish a new tag for changed release documentation.

`tags.json` is generated from all published versions and read by the ADI theme
for the version dropdown. It labels the highest stable release “latest stable,”
`main` as “development,” prereleases, and PR previews. It has no root-version
entry, since the root is a redirect. Do not hand-edit the generated menu.
The sitemap contains development and release pages, and excludes PR previews.

Pages deployments use the assembled `gh-pages` commit as their build identity,
so main and tag publications from the same source commit remain distinct.
Artifact names include the run attempt so deployment reruns select one artifact.

Publishing and cleanup share a concurrency group. Every publication and cleanup
reads the current open PR list, removes stale previews, and deploys the complete
site. A preview build queued after closing cannot restore that PR's directory.
Files removed from the live site remain in `gh-pages` Git history.

The existing `v0.1.0-a.1` tag predates the site. The historical builder reads
its original README and licenses with `git show` and labels it as a historical
prerelease. Main builds upload it separately and the publisher installs it only
if the historical directory is absent. Later main builds leave its files intact.
This does not change the original tag or runtime code.

The first deployment also migrates output from the previous root-based layout:
root documentation is retained under its version name, existing release paths
are preserved, and the root becomes the default-version redirect.

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
