#!/bin/bash

PARAM="/data/params/d_tmp/CameraOffset"
INCREMENT=0.02

# read current value (default 0.00)
CURRENT=$(cat "$PARAM" 2>/dev/null)
CURRENT=${CURRENT:-0.00}

# add (float-safe)
NEW=$(awk "BEGIN { printf \"%.2f\", $CURRENT + $INCREMENT }")

# write back
echo "$NEW" > "$PARAM"

echo "CameraOffset: $CURRENT -> $NEW"
