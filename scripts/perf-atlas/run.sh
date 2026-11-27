#!/usr/bin/env bash
# Build and run every crate's perf_atlas example, then render one HTML report.
#
#   scripts/perf-atlas/run.sh [--quick] [--resources [--counters]] [--out DIR] [--crates "a b c"] [--filter TEXT]
#
# Results land in DIR (default target/perf-atlas, or target/resource-atlas with
# --resources): one JSON document per crate and index.html. Timing sweeps measure
# wall-clock calls; avoid other heavy work while they run. --resources records heap,
# stack, result-shape and counter use per call instead (memory and depth atlases).
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=
CRATES="hyperreal hyperinterval hyperlimit hyperlattice hyperlattice-interval hypersolve hyperdynamics hypercurve hypertri"
ARGS=()
RESOURCES=
while (($#)); do
  case "$1" in
    --quick) ARGS+=(--quick) ;;
    --resources) ARGS+=(--resources); RESOURCES=1 ;;
    --counters) ARGS+=(--counters) ;;
    --out) OUT="$2"; shift ;;
    --crates) CRATES="$2"; shift ;;
    --filter) ARGS+=(--filter "$2"); shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
if [ -z "$OUT" ]; then
  if [ -n "${RESOURCES:-}" ]; then OUT=target/resource-atlas; else OUT=target/perf-atlas; fi
fi
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
