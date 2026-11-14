#!/usr/bin/env bash
# Generates, builds and installs the OrthoStream COSMOS plugin (ADR-021) into
# the instance of scripts/cosmos_setup.sh, which must be running.
#
#   1. tools/ground/generate_cosmos_plugin.py: build/cosmos -> build/cosmos_plugin
#   2. openc3cli rake build: the plugin gem
#   3. openc3cli load: install it, or upgrade the installed version
#
# Run mission/build.sh first; the packet definitions come from its EDS build.
# Usage: scripts/cosmos_plugin.sh [version]   (default: 0.1.<timestamp>)
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cosmos_dir="$repo_root/ground/cosmos/runtime/orthostream-cosmos"
plugin_dir="$repo_root/build/cosmos_plugin/openc3-cosmos-orthostream"
version="${1:-0.1.$(date +%Y%m%d%H%M%S)}"

if [[ ! -x "$cosmos_dir/openc3.sh" ]]; then
    echo "COSMOS instance missing: run scripts/cosmos_setup.sh" >&2
    exit 1
fi
python3 "$repo_root/tools/ground/generate_cosmos_plugin.py"

# The same container command as "openc3.sh cli", without a terminal.
cd "$cosmos_dir"
compose=(docker compose --env-file .env --env-file .env.local -f compose.yaml -f compose.override.yaml)
export OPENC3_USER_ID="$(id -u)" OPENC3_GROUP_ID="$(id -g)"
cli() {
    "${compose[@]}" run --rm -T -v "$plugin_dir:/openc3/local:z" -w /openc3/local --no-deps \
        openc3-cosmos-cmd-tlm-api ruby /openc3/bin/openc3cli "$@"
}

cli rake build "VERSION=$version"
gem="openc3-cosmos-orthostream-$version.gem"
cli load "$gem"
echo "Installed $gem"
