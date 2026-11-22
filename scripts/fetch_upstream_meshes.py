# SPDX-FileCopyrightText: 2026 Quack Robotics contributors
# SPDX-License-Identifier: Apache-2.0
"""Fetch the Open Duck Mini v2 STL meshes that robot.xml needs.

The meshes are not committed (GOVERNANCE.md: no binary blobs). They are
downloaded from upstream at the pinned commit and checked against
assets/open_duck_mini_v2/SHA256SUMS. Any mismatch or download error exits
nonzero; nothing is silently skipped.
"""
import hashlib
import pathlib
import sys
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / "assets" / "open_duck_mini_v2"
SUMS_FILE = MODEL_DIR / "SHA256SUMS"
UPSTREAM_COMMIT = "b23317a485b3cec7d8417f352478778b3475173c"
UPSTREAM_BASE = (
    "https://raw.githubusercontent.com/apirrone/Open_Duck_Mini/"
    f"{UPSTREAM_COMMIT}/mini_bdx/robots/open_duck_mini_v2/"
)


def read_pinned_sums() -> dict[str, str]:
    pinned = {}
    for line in SUMS_FILE.read_text().splitlines():
        digest, filename = line.split(maxsplit=1)
        pinned[filename] = digest
    return pinned


def sha256_of(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch(filename: str, expected_digest: str) -> None:
    target = MODEL_DIR / filename
    if target.exists() and sha256_of(target) == expected_digest:
        return
    with urllib.request.urlopen(UPSTREAM_BASE + filename, timeout=60) as response:
        payload = response.read()
    actual_digest = hashlib.sha256(payload).hexdigest()
    if actual_digest != expected_digest:
        raise SystemExit(f"{filename}: sha256 {actual_digest} != pinned {expected_digest}")
    target.write_bytes(payload)


def main() -> int:
    pinned = read_pinned_sums()
    meshes = {name: digest for name, digest in pinned.items() if name.endswith(".stl")}
    for filename, digest in sorted(meshes.items()):
        fetch(filename, digest)
    print(f"{len(meshes)} meshes present and verified @ {UPSTREAM_COMMIT[:7]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
