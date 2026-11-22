<!--
SPDX-FileCopyrightText: 2026 Quack Robotics contributors
SPDX-License-Identifier: CC-BY-SA-4.0
-->
# Open Duck Mini v2 MJCF (vendored)

- Source: https://github.com/apirrone/Open_Duck_Mini
- Path: `mini_bdx/robots/open_duck_mini_v2/`
- Commit: `b23317a485b3cec7d8417f352478778b3475173c`
- License: Apache-2.0 (upstream)

`robot.xml` is vendored unmodified. Its sha256 is pinned in `SHA256SUMS`, in
`tests/test_actuator_map.py`, and in the shared CI sim-gate.

The 45 STL meshes it references are **not committed** (GOVERNANCE.md §3: no
binary blobs). Fetch and verify them against `SHA256SUMS`:

```
python scripts/fetch_upstream_meshes.py
```

To re-pin to a new upstream commit, update the commit here and in the fetch
script, regenerate `SHA256SUMS`, and rerun `pytest tests/`.
