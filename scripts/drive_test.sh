#!/usr/bin/env bash
# Compatibility entry point; implementation is grouped by responsibility.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/control/drive_test.sh" "$@"
