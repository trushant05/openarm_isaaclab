#!/bin/bash

/workspace/isaaclab/isaaclab.sh -p convert_urdf.py \
  /workspace/openarm_playground/URDF/openarm.urdf \
  /workspace/openarm_playground/USD/unmerged/openarm/openarm.usd \
  --fix-base \
  --root-link-name "openarm_body_link0" \
  --joint-stiffness 100.0 \
  --joint-damping 1.0 \
  --joint-target-type "position" \
  --collision-approximation "convexDecomposition" --verbose
