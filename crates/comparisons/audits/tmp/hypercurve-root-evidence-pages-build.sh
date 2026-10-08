#!/usr/bin/env bash
set -euo pipefail
cd /home/tim/Documents/GitHub/workspace/hypercurve/examples/hypercurve_ui
env -u NO_COLOR trunk --skip-version-check build index.html --release --locked --dist /tmp/hypercurve-pages-root-evidence.xrVrhY --public-url /hypercurve/ > /tmp/hypercurve-root-evidence-trunk.log 2>&1
echo 'Hypercurve root-evidence Pages build complete'
