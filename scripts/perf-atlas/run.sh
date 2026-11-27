#!/usr/bin/env bash
# Build selected perf_atlas examples sequentially, then render one HTML report.
#
#   scripts/perf-atlas/run.sh [--quick] [--out DIR] [--crates "a b c"] [--filter TEXT]
#
# Results land in DIR (default target/perf-atlas): one JSON document per crate and
# index.html. The sweeps time wall-clock calls; avoid other heavy work while they run.
# The default crate list is explicit, not discovered. Missing examples are skipped;
# failed commands stop the run and existing output files are not cleared. An empty
# selected set reaches the renderer's missing-input error. --seed is available only
# through direct example invocation, not this wrapper. Missing option values fail
# under nounset; unknown options exit with status 2. Relative output paths are based
# on the repository root after cd. No timeout encloses an individual local run.
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=target/perf-atlas
CRATES="hyperreal hyperinterval hyperlimit hyperlattice hyperlattice-interval hypersolve hyperdynamics hypercurve hypertri"
ARGS=()
while (($#)); do
  case "$1" in
    --quick) ARGS+=(--quick) ;;
    --out) OUT="$2"; shift ;;
    --crates) CRATES="$2"; shift ;;
    --filter) ARGS+=(--filter "$2"); shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
mkdir -p "$OUT"
DOCS=()
for crate in $CRATES; do
  if [ ! -f "crates/$crate/examples/perf_atlas.rs" ]; then
    echo "skipping $crate: no examples/perf_atlas.rs" >&2
    continue
  fi
  echo "== $crate" >&2
  cargo run --locked --release -q -p "$crate" --all-features --example perf_atlas -- \
    --out "$OUT/$crate.json" "${ARGS[@]}"
  DOCS+=("$OUT/$crate.json")
done
python3 scripts/perf-atlas/render.py "$OUT" "${DOCS[@]}"
