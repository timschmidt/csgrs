#!/usr/bin/env bash
# Regenerates the tracked benchmark reports of the hyper crates.
#
# Each crate's report writers read Criterion's results, so every crate gets
# its own CRITERION_HOME: in the workspace, Criterion otherwise writes all
# crates' results to one target/criterion and each report would catalogue
# the others' benches. Reports are written into crates/<crate>/.
#
# Usage: scripts/bench-reports.sh [crate ...]
#   With no arguments, regenerates hyperreal, hyperlattice, hyperlimit,
#   hypersolve and hypertri. Set BENCH_REPORTS_SKIP_CGAL=1 to skip
#   hypersolve's CGAL comparison.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

target_dir="${CARGO_TARGET_DIR:-$root/target}"
if [[ "$target_dir" != /* ]]; then
  target_dir="$root/$target_dir"
fi

bench() {
  local crate="$1"
  shift
  echo "==> $crate: cargo bench $*" >&2
  (
    cd "crates/$crate"
    CRITERION_HOME="$target_dir/criterion-$crate" cargo bench --locked -p "$crate" "$@"
  )
}

hyperreal() {
  bench hyperreal --features simple
  bench hyperreal --bench dispatch_trace --features dispatch-trace
}

hyperlattice() {
  # mathbench rewrites benchmarks.md whole, so it runs before the benches
  # that add sections to it.
  bench hyperlattice --bench mathbench
  bench hyperlattice --bench regression_sentinels
  bench hyperlattice --bench retained_fuzz
  bench hyperlattice --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
  bench hyperlattice --bench api_dispatch_trace --features hyperreal-dispatch-trace
}

hyperlimit() {
  bench hyperlimit --features parallel
  bench hyperlimit --all-features --bench predicates -- --write-dispatch-trace-md
}

hypersolve() {
  bench hypersolve
  bench hypersolve --bench dispatch_trace --features dispatch-trace
  if [[ "${BENCH_REPORTS_SKIP_CGAL:-}" != 1 ]]; then
    echo "==> hypersolve: CGAL quadratic comparison" >&2
    (cd crates/hypersolve && benches/competitors/run_cgal_quadratic.sh)
  fi
}

hypertri() {
  bench hypertri --features all-algorithms,runtime-select,f64-interop
  bench hypertri --bench dispatch_trace --features all-algorithms,runtime-select,dispatch-trace
}

crates=("$@")
if ((${#crates[@]} == 0)); then
  crates=(hyperreal hyperlattice hyperlimit hypersolve hypertri)
fi
for crate in "${crates[@]}"; do
  case "$crate" in
    hyperreal | hyperlattice | hyperlimit | hypersolve | hypertri) "$crate" ;;
    *)
      echo "no tracked benchmark reports for $crate" >&2
      exit 2
      ;;
  esac
done
