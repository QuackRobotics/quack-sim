# Projector clearance tooling (Atlas)

Clearance between the left-shoulder projector tube (Woz quack-hardware #8 90bde49: Ø12×50, lens face (46, −45, 115.9) mm
in trunk_assembly, 8° down, enlarged to r 7.2 mm) and every moving neck/head/antenna mesh vertex of upstream
Open Duck Mini v2 robot.xml b23317a. Run from this folder with `<robot_dir>` = the b23317a `open_duck_mini_v2` robot dir.

- `projclear.py`, `stlvol.py`: Woz's FK and STL loader (quack-hardware 510e662), unchanged.
- `table.py`: 5° grid over full ranges; stage 3 gives margin m. Rule: target allowed iff min over the 16 enclosing nodes ≥ 4 + m.
- `permesh.py`: per-mesh breakdown of the negative nodes.
- `fk_sign.py`: sign of each head joint (+5° → front vertex motion).
- `rollbox.py`, `rollbox.log`: HAL head box (neck ±15, pitch ±15, yaw ±50, roll −15..15, 5° grid, fine on coarse < 25 mm); worst node per roll set and per roll node.
- **m is empirical**: m = 1.73 mm is the worst cellmin−true seen over 3,967 random in-cell samples (table.py stage 3), not a proven bound.
- `table.log`, `permesh.log`: results of the 2026-11-24 run (m = 1.73 mm → threshold 5.73 mm).
Outputs (`table_b23317a.npz`, `permesh.npz`, `table_validate.npy`) are in gs://lj495412-bot-handoff-20261108/quack/atlas/projector_table_b23317a_20261124.tgz.
