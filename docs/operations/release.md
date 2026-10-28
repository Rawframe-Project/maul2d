# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning; before 1.0.0 a breaking change bumps the minor version, a feature the minor and a fix the patch. Tags are `vX.Y.Z`.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul2d/base.h`, where CMake and `M2_ABI_VERSION` read it, updates `.release-please-manifest.json`, and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles.
2. Rewrite that entry in user language before merging: what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security. Leave out refactors, tests, CI and documentation. Say what a caller must change for a breaking change, and name every change of results or of the snapshot and tape formats, since games compare hashes across machines and a release refuses the snapshots and tapes of another ABI version.
3. Check that the `required` CI job passed on the release pull request on every cell.
4. Mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the changelog entry as its notes. The Pages workflow publishes the documentation and the testbed from `main`.
5. `.github/workflows/release.yml` then builds the source archive, writes `SHA256SUMS` and an SPDX SBOM, attests where they were built, and attaches them to the release. Like every Maul library, maul2d ships as source; packages for vcpkg and Conan are a family decision of their own. Check that the assets are there; a user verifies a download with `sha256sum -c --ignore-missing SHA256SUMS` and `gh attestation verify <file> -R Rawframe-Project/maul2d`.

## Notes

- A version that is not the next one by the commit titles is set with a `Release-As: X.Y.Z` footer in the commit that should produce it.
- The binary interface changes with every release before 1.0.0: `M2_ABI_VERSION` encodes the whole version, so a program must be built against the headers of the library it links.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- From 1.0.0, a patch release never changes results. Its `M2_ABI_VERSION` is the minor version's, so it reads the snapshots and tapes of the release before it, and a tape replays its calls on the reader's step code: a moved golden line would replay an old tape onto other bits instead of refusing it. A change that moves a line of `test/hashes.txt` is titled `feat`, so release-please makes the next release a minor one. `tools/check_patch_results.py` refuses a patch release whose `test/hashes.txt` differs from the last tag's; if one is proposed, a commit with a `Release-As: X.Y.0` footer makes it a minor release.
- Tags are never deleted or moved. A broken release is fixed by the next one.
