#!/usr/bin/env bash
# Run the ROAM test world with the same sensors and viewers as the main launcher.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/scripts/launch_roam.sh" roam_test "$@"
