# Muon3D fork of Klipper: upstream remote and rebase process (KAN-104)

Muon3D_Klipper is a fork of `Klipper3d/klipper`. Moonraker (`Muon-3D/Moonraker`) is a fork of `Arksine/moonraker` and follows the same process.

## Remotes
```
git remote add upstream https://github.com/Klipper3d/klipper.git      # Moonraker: https://github.com/Arksine/moonraker.git
git fetch upstream
```
Never push to `upstream`.

## Where we are (measured 30 Oct 2026)
- Last common commit with upstream: `8a210d23` (12 Feb 2026).
- `master` has 413 commits upstream lacks; upstream has 284 we lack.
- Our changes since the merge base: 155 files, +23,639 / -271, mostly `klippy/extras` (48), `test/klippy` (32) and `core/M1` (24, the M1 config and macros). The few edits to upstream files (`klippy/chelper`, `test/serialqueue`) are where drift hurts first.
- Re-measure with:
```
mb=$(git merge-base origin/master upstream/master)
git rev-list --left-right --count origin/master...upstream/master
git diff --stat $mb origin/master
```

## Keeping the patch set visible
- Prefer new files (`klippy/extras/<name>.py`, `core/M1/...`) over edits to upstream files.
- Every edit to an upstream-owned file carries a `Muon:` comment with the KAN number.
- List edited upstream files with `git diff --name-status $mb origin/master | grep -v '^A'`; review this list at each rebase.

## Rebase process (merge-based, so history stays linear to review)
1. `git fetch upstream` and note the measurements above in the PR body.
2. Branch `chore/upstream-YYYY-MM-DD` from `origin/master`, then `git merge upstream/master`.
3. Resolve conflicts in favour of upstream, then re-apply the Muon change on top.
4. Run the host tests (`test/` suites run by CI on bigbox). Do not run anything that moves, heats or flashes a printer.
5. Open a PR named `KAN-n: update to upstream <date>`. A MuonOS pin bump follows only after C3 bench evidence.
6. Cadence: once a month, and immediately for an upstream security fix (cherry-pick it to `master` first, then do the full merge).

## Security backports
Watch upstream releases and advisories. A fix that cannot wait for the monthly merge is cherry-picked with `git cherry-pick -x <sha>` so the origin stays recorded, and gets its own PR.
