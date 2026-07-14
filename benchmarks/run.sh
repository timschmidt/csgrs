#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="$root/benchmarks/results/$(date -u +%Y%m%dT%H%M%SZ)"
csgrs_only=0
quick=0

while (($#)); do
  case "$1" in
    --csgrs-only) csgrs_only=1 ;;
    --quick) quick=1 ;;
    --output)
      shift
      output="${1:?--output requires a directory}"
      ;;
    *)
      echo "usage: $0 [--quick] [--csgrs-only] [--output DIRECTORY]" >&2
      exit 2
      ;;
  esac
  shift
done

if ((quick)); then
  export CSGRS_BENCH_SAMPLES=1
  export CSGRS_BENCH_WARMUP=0
fi

mkdir -p "$output"
cd "$root"

cargo bench --bench kernel_comparison >"$output/csgrs-kernel.csv"
cargo bench --bench feature_pipeline --features offset,bevymesh >"$output/csgrs-feature.csv"

csv_files=("$output/csgrs-kernel.csv" "$output/csgrs-feature.csv")
if ((!csgrs_only)); then
  cmake -S benchmarks/native -B target/native-benchmarks \
    -DCMAKE_BUILD_TYPE=Release -DCSGRS_BENCH_REQUIRE_ALL=ON
  cmake --build target/native-benchmarks --parallel
  target/native-benchmarks/csgrs_bench_cgal >"$output/cgal-epeck.csv"
  target/native-benchmarks/csgrs_bench_occt >"$output/opencascade-tight.csv"
  csv_files+=("$output/cgal-epeck.csv" "$output/opencascade-tight.csv")
fi

python3 benchmarks/summarize.py "${csv_files[@]}" --output "$output/summary.md"
echo "benchmark report: $output/summary.md"
