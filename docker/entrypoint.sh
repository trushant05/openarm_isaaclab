#!/bin/bash

set -e

# Inside your container or dev environment
for dir in "$DOCKER_REPO_PATH"/source/*; do
    if [ -d "$dir" ]; then
        if [ -f "$dir/setup.py" ] || [ -f "$dir/pyproject.toml" ]; then
            echo "Installing $dir..."
            /workspace/isaaclab/_isaac_sim/python.sh -m pip install -e "$dir"
        fi
    fi
done

# Start an interactive bash shell
exec bash
