#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypersolve
test -d /tmp/hypersolve-native-constant-sign-coverage.t7dwlv
env CARGO_TARGET_DIR=/tmp/hypersolve-native-constant-sign-coverage.t7dwlv scripts/coverage.sh > /tmp/hypersolve-root-evidence-coverage.log 2>&1
echo 'Hypersolve root-evidence coverage complete'
