#!/bin/bash
# Exact args for the roll-box cases. Run from ~/quack-core/atlas/rollbox/ on razer.
# Other-joint boxes (absolute deg, default 0):
#   A = interim box from table.py (neck 0..65, pitch -22.5..45, yaw full -160..160)
#   B = ruled HAL clamp around default (#8: neck +-15 checked, pitch +-13.5, yaw +-48)
# Roll cases: -8/+5, +-8, policy p1 -5.95 plus 4.5 / 9.0 overlay (-10.45 / -14.95), and symmetric +-10.45 / +-14.95.
set -e
cd "$(dirname "$0")"
P=../.venv/bin/python
for B in "0 65 -22.5 45 -160 160" "-15 15 -13.5 13.5 -48 48"; do
  echo "== other joints (neck_lo neck_hi pitch_lo pitch_hi yaw_lo yaw_hi): $B"
  $P rollbox.py $B -8 5 roll_-8/+5
  $P rollbox.py $B -8 8 roll_+-8
  $P rollbox.py $B -10.45 -10.45 p1-5.95+ovl4.5
  $P rollbox.py $B -14.95 -14.95 p1-5.95+ovl9.0
  $P rollbox.py $B -10.45 10.45 roll_+-10.45
  $P rollbox.py $B -14.95 14.95 roll_+-14.95
done
