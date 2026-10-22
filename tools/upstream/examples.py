#!/usr/bin/env python3
"""Drift check of the ported examples (design/examples.md, design/PORT.md section 2).

The sketches in examples/ and their tests in test/examples are ported by hand from allwpilib's
wpilibcExamples, so tools/upstream/wpilib.py does not regenerate them. tools/upstream/examples.json
records, for each ported folder or file, the upstream folders or files it was ported from and the
git blob id of every upstream file at the pinned commit (BUILD.bazel and *.bzl excluded), plus the
blob ids of upstream's four example lists (examples.json, snippets.json, templates.json,
commands.json; a new upstream example changes one of them).

  --check  (default) compares the manifest with the upstream tree at the commit in
           tools/upstream/wpilib.lock and with the local tree:
           - the manifest's commit is the pinned commit;
           - every recorded upstream file still has the recorded blob id, and no file was added to
             or removed from a recorded upstream folder;
           - the four example lists are unchanged;
           - every ported path exists, every examples/* folder is ported or hand-written, and
             every path in test/examples is ported.
           Exit 1 on any difference, with the list of changed upstream files to review.
  --write  rewrites the blob ids and the commit from the upstream tree at the pinned commit (after
           a pin bump, once the listed changes have been carried into the ports).

Only git tree objects are read (`git ls-tree`), so the blob-filtered clone that wpilib.py keeps in
.upstream/allwpilib (or PROSDK_UPSTREAM) is enough.

Usage:
  examples.py [--check | --write] [--upstream PATH]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wpilib  # noqa: E402

ROOT = wpilib.ROOT
MANIFEST = wpilib.HERE / "examples.json"
EXAMPLES_SRC = "wpilibcExamples/src/main/cpp"
LISTS = [f"{EXAMPLES_SRC}/{k}/{k}.json" for k in ("examples", "snippets", "templates", "commands")]


def _skipped(path: str) -> bool:
    name = path.rsplit("/", 1)[-1]
    return name == "BUILD.bazel" or name.endswith(".bzl")


def upstream_blobs(up: Path, commit: str, paths: list[str]) -> dict[str, str]:
    """Blob id of every file under `paths` (folders or files) at `commit`."""
    out: dict[str, str] = {}
    listing = wpilib.git(up, "ls-tree", "-r", "--full-tree", commit, "--", *paths)
    for ln in listing.splitlines():
        meta, path = ln.split("\t", 1)
        mode, kind, sha = meta.split()
        if kind == "blob" and not _skipped(path):
            out[path] = sha
    return out


def local_problems(manifest: dict) -> list[str]:
    problems = []
    ported = manifest["ported"]
    for key in ported:
        if not (ROOT / key).exists():
            problems.append(f"{key}: in the manifest but missing from the tree")
    handwritten = set(manifest["handwritten"])
    for d in sorted((ROOT / "examples").iterdir()):
        if d.is_dir() and f"examples/{d.name}" not in ported and d.name not in handwritten:
            problems.append(f"examples/{d.name}: neither ported (manifest) nor hand-written")
    for p in sorted((ROOT / "test" / "examples").iterdir()):
        if f"test/examples/{p.name}" not in ported:
            problems.append(f"test/examples/{p.name}: not in the manifest")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description="Drift check of the ported WPILib examples.")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="compare (default)")
    mode.add_argument("--write", action="store_true", help="rewrite the blob ids and the commit")
    ap.add_argument("--upstream", type=Path,
                    default=Path(os.environ["PROSDK_UPSTREAM"]) if os.environ.get("PROSDK_UPSTREAM")
                    else ROOT / ".upstream" / "allwpilib",
                    help="allwpilib checkout at the pinned commit (env PROSDK_UPSTREAM)")
    args = ap.parse_args()

    lock = wpilib.read_lock()
    sparse = sorted({s for m in wpilib.load_modules().values() for s in m.sparse})
    up = wpilib.ensure_upstream(args.upstream.resolve(), lock, sparse)
    commit = lock["commit"]
    manifest = json.loads(MANIFEST.read_text())

    lists = upstream_blobs(up, commit, LISTS)
    current = {key: upstream_blobs(up, commit, entry["upstream"])
               for key, entry in manifest["ported"].items()}
    empty = [key for key, files in current.items() if not files]
    if empty:
        raise SystemExit("error: no upstream files at " + commit[:12] + " for: " + ", ".join(empty))

    if args.write:
        manifest["commit"] = commit
        manifest["lists"] = lists
        for key, entry in manifest["ported"].items():
            entry["files"] = current[key]
        MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"wrote {MANIFEST.relative_to(ROOT)}: {len(current)} ported paths at {commit[:12]}")
        return 0

    problems = local_problems(manifest)
    if manifest["commit"] != commit:
        problems.append(f"manifest commit {manifest['commit'][:12]} != pinned {commit[:12]}")
    for path in sorted(set(lists) | set(manifest["lists"])):
        if lists.get(path) != manifest["lists"].get(path):
            problems.append(f"{path}: upstream example list changed (new or removed examples)")
    nfiles = 0
    for key, entry in manifest["ported"].items():
        old, new = entry["files"], current[key]
        nfiles += len(new)
        for path in sorted(set(old) | set(new)):
            if path not in new:
                problems.append(f"{key}: upstream removed {path}")
            elif path not in old:
                problems.append(f"{key}: upstream added {path}")
            elif old[path] != new[path]:
                problems.append(f"{key}: upstream changed {path} ({old[path][:10]} -> "
                                f"{new[path][:10]})")
    if problems:
        print("examples drift:")
        for p in problems:
            print(f"  {p}")
        print("Carry the upstream changes into the ports, then run tools/upstream/examples.py "
              "--write.")
        return 1
    print(f"examples check OK: {len(manifest['ported'])} ported paths, {nfiles} upstream files "
          f"and {len(lists)} example lists match {commit[:12]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
