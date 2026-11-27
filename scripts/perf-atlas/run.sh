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
CRATES="hyperreal hyperinterval hyperlimit hyperlattice hyperlattice-interval hypersolve hyperdynamics hypercurve hypertri hypermesh hyperbrep hyperbrep-io hyperparts hyperpath hypersdf hypervoxel hyperspatial hyperphysics hyperevolution hypergraphics hypercircuit hyperdrc hyperpack csgrs"
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
  # The root package lives at the top level; hypervoxel is a nested workspace
  # with its own lockfile, so it is built from its own directory.
  if [ "$crate" = csgrs ]; then dir=.; else dir="crates/$crate"; fi
  if [ ! -f "$dir/examples/perf_atlas.rs" ]; then
    echo "skipping $crate: no examples/perf_atlas.rs" >&2
    continue
  fi
  echo "== $crate" >&2
  out="$(cd "$OUT" && pwd)/$crate.json"
  if [ "$crate" = hypervoxel ]; then
    # Its default members are the voxelis crates, so name the package.
    (cd "$dir" && cargo run --locked --release -q -p hypervoxel --all-features --example perf_atlas -- \
      --out "$out" "${ARGS[@]}")
  else
    cargo run --locked --release -q -p "$crate" --all-features --example perf_atlas -- \
      --out "$out" "${ARGS[@]}"
  fi
  DOCS+=("$out")
done
python3 scripts/perf-atlas/render.py "$OUT" "${DOCS[@]}"
