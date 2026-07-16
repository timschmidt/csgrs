#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="$root/benchmarks/results/$(date -u +%Y%m%dT%H%M%SZ)"
csgrs_only=0
quick=0
cold_warm=0

while (($#)); do
  case "$1" in
    --csgrs-only) csgrs_only=1 ;;
    --quick) quick=1 ;;
    --cold-warm) cold_warm=1 ;;
    --output)
      shift
      output="${1:?--output requires a directory}"
      ;;
    *)
      echo "usage: $0 [--quick] [--cold-warm] [--csgrs-only] [--output DIRECTORY]" >&2
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

run_cold_samples() {
  local destination="$1"
  shift
  local samples="${CSGRS_BENCH_COLD_SAMPLES:-5}"
  if ((quick)); then
    samples=1
  fi
  local temporary="$output/.cold-sample.csv"
  : >"$destination"
  for ((sample = 0; sample < samples; sample++)); do
    CSGRS_BENCH_TEMPERATURE=cold \
    CSGRS_BENCH_SAMPLES=1 \
    CSGRS_BENCH_WARMUP=0 \
    CSGRS_BENCH_ITERATIONS=1 \
    CSGRS_BENCH_SAMPLE_OFFSET="$sample" \
      "$@" >"$temporary"
    if ((sample == 0)); then
      head -n 1 "$temporary" >"$destination"
    fi
    sed -n '2,$p' "$temporary" >>"$destination"
  done
  rm -f "$temporary"
}

if ((cold_warm)); then
  if ((csgrs_only)); then
    echo "--cold-warm requires all cross-kernel engines" >&2
    exit 2
  fi
  cmake -S benchmarks/native -B target/native-benchmarks \
    -DCMAKE_BUILD_TYPE=Release -DCSGRS_BENCH_REQUIRE_ALL=ON
  CCACHE_DISABLE=1 cmake --build target/native-benchmarks --parallel

  CSGRS_BENCH_TEMPERATURE=warm \
    cargo bench --bench kernel_comparison >"$output/csgrs-kernel-warm.csv"
  CSGRS_BENCH_TEMPERATURE=warm \
    target/native-benchmarks/csgrs_bench_cgal >"$output/cgal-epeck-warm.csv"
  CSGRS_BENCH_TEMPERATURE=warm \
    target/native-benchmarks/csgrs_bench_occt >"$output/opencascade-tight-warm.csv"

  run_cold_samples "$output/csgrs-kernel-cold.csv" \
    cargo bench --bench kernel_comparison
  run_cold_samples "$output/cgal-epeck-cold.csv" \
    target/native-benchmarks/csgrs_bench_cgal
  run_cold_samples "$output/opencascade-tight-cold.csv" \
    target/native-benchmarks/csgrs_bench_occt

  python3 benchmarks/summarize.py \
    "$output/csgrs-kernel-cold.csv" \
    "$output/csgrs-kernel-warm.csv" \
    "$output/cgal-epeck-cold.csv" \
    "$output/cgal-epeck-warm.csv" \
    "$output/opencascade-tight-cold.csv" \
    "$output/opencascade-tight-warm.csv" \
    --output "$output/summary.md" \
    --require-engine-parity
  echo "benchmark report: $output/summary.md"
  exit 0
fi

export CSGRS_BENCH_TEMPERATURE="${CSGRS_BENCH_TEMPERATURE:-warm}"
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

summary_args=("${csv_files[@]}" --output "$output/summary.md")
if ((!csgrs_only)); then
  summary_args+=(--require-engine-parity)
fi
python3 benchmarks/summarize.py "${summary_args[@]}"
echo "benchmark report: $output/summary.md"
