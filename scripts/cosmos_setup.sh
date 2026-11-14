#!/usr/bin/env bash
# Fetches the pinned OpenC3 COSMOS project for OrthoStream (ADR-021) into
# ground/cosmos/runtime/orthostream-cosmos and configures it as a separate
# instance: own compose project, volumes and ports (ground/cosmos/
# compose.override.yaml). The COSMOS project is upstream's runnable
# template; it is not copied into this repository.
#
# Usage: scripts/cosmos_setup.sh
#
# Then:
#   ground/cosmos/runtime/orthostream-cosmos/openc3.sh run    start (http://localhost:2910)
#   ground/cosmos/runtime/orthostream-cosmos/openc3.sh stop   stop, keep the volumes
#   scripts/cosmos_plugin.sh                                  build and install the plugin
set -euo pipefail

# The pinned COSMOS release (ADR-021 section 1).
COSMOS_PROJECT_TAG="v7.4.1"
COSMOS_PROJECT_URL="https://github.com/OpenC3/cosmos-project.git"
COMPOSE_PROJECT="orthostream-cosmos"

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
project_dir="$repo_root/ground/cosmos/runtime/orthostream-cosmos"

if [[ -d "$project_dir/.git" ]]; then
    current_tag="$(git -C "$project_dir" describe --tags --exact-match 2>/dev/null || echo unknown)"
    if [[ "$current_tag" != "$COSMOS_PROJECT_TAG" ]]; then
        echo "error: $project_dir is at $current_tag, expected $COSMOS_PROJECT_TAG." >&2
        echo "Stop COSMOS, remove that directory and run this script again." >&2
        exit 1
    fi
    echo "COSMOS project $COSMOS_PROJECT_TAG is already at $project_dir"
else
    mkdir -p "$(dirname -- "$project_dir")"
    git -c advice.detachedHead=false clone --quiet --depth 1 \
        --branch "$COSMOS_PROJECT_TAG" "$COSMOS_PROJECT_URL" "$project_dir"
    echo "COSMOS project $COSMOS_PROJECT_TAG fetched into $project_dir"
fi

ln -sfn ../../compose.override.yaml "$project_dir/compose.override.yaml"

# Instance settings. .env.local is read after the upstream .env; it is not
# tracked (the runtime directory is ignored by git).
if [[ ! -f "$project_dir/.env.local" ]]; then
    cat > "$project_dir/.env.local" <<ENV
# OrthoStream instance (written by scripts/cosmos_setup.sh)
COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT
OPENC3_EXTERNAL_URL=http://localhost:2910
OPENC3_DEMO=0
SECRET_KEY_BASE=$(head -c 64 /dev/urandom | od -An -tx1 | tr -d ' \n')
ENV
    echo "Instance settings written to $project_dir/.env.local"
fi
echo "Start with: $project_dir/openc3.sh run"
