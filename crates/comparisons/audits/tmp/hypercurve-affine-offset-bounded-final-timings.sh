#!/usr/bin/env bash
set -euo pipefail
bash /tmp/hyperreal-affine-offset-bounded-micro-matched.sh
bash /tmp/hypercurve-affine-offset-bounded-matched.sh
bash /tmp/hypercurve-affine-offset-bounded-rational-extended.sh
echo 'All bounded affine-offset matched timing controls complete'
