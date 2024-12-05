#!/bin/bash

# Default number of planes to launch
DEFAULT_PLANES=2
NUM_PLANES=${1:-$DEFAULT_PLANES}

# Path to PX4-Autopilot
PX4_PATH="~/Developer/PX4-Autopilot"
PX4_BIN="$PX4_PATH/build/px4_sitl_default/bin/px4"
WORLD="lawn"

# Check if num of plane arg is at least 1
if [ "$NUM_PLANES" -lt 1 ]; then
  echo "Number of planes must be at least 1."
  exit 1
fi

# Launch fpv plane
echo "Launching plane 1 (rc_cessna_fpv) with pose 0"
gnome-terminal -- bash -c "
  cd $PX4_PATH;
  PX4_SYS_AUTOSTART=4016 PX4_SIM_MODEL=gz_rc_cessna_fpv PX4_GZ_WORLD=$WORLD $PX4_BIN -i 1;
  exec bash
"

# Launch the remaining planes
for ((i=2; i<=NUM_PLANES; i++)); do
  MODEL_POSE="-$((i-1))"
  echo "Launching plane $i (rc_cessna) with pose $MODEL_POSE"
  gnome-terminal -- bash -c "
    cd $PX4_PATH;
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4003 PX4_GZ_MODEL_POSE=\"$MODEL_POSE\" PX4_SIM_MODEL=gz_rc_cessna PX4_GZ_WORLD=$WORLD $PX4_BIN -i $i;
    exec bash
  "
done

echo "All $NUM_PLANES planes launched!"
