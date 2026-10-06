#!/usr/bin/env bash
set -euo pipefail

compiler="${CXX:-g++}"
binary="${TMPDIR:-/tmp}/hypersolve-cgal-quadratic"
competitor_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "$competitor_dir/../.." && pwd)"
report_path="$repository_root/cgal_quadratic_benchmarks.md"

"${compiler}" \
  -O3 \
  -DNDEBUG \
  -std=c++17 \
  "$competitor_dir/cgal_quadratic.cpp" \
  -o "${binary}" \
  -lgmpxx \
  -lgmp

result="$("${binary}")"
printf '%s\n' "$result"

benchmark_id="${result%% *}"
cgal_version=""
median_ns=""
p05_ns=""
p95_ns=""
checksum=""
for field in $result; do
  case "$field" in
    cgal=*) cgal_version="${field#cgal=}" ;;
    median_ns=*) median_ns="${field#median_ns=}" ;;
    p05_ns=*) p05_ns="${field#p05_ns=}" ;;
    p95_ns=*) p95_ns="${field#p95_ns=}" ;;
    checksum=*) checksum="${field#checksum=}" ;;
  esac
done

if [[ -z "$cgal_version" || -z "$median_ns" || -z "$p05_ns" || -z "$p95_ns" || -z "$checksum" ]]; then
  echo "failed to parse CGAL benchmark output" >&2
  exit 1
fi

temporary_report="$(mktemp "$repository_root/.cgal-quadratic-report.XXXXXX")"
trap 'rm -f -- "$temporary_report"' EXIT
{
  printf '# Hypersolve CGAL Exact Quadratic Comparison\n\n'
  printf 'Generated automatically by `benches/competitors/run_cgal_quadratic.sh`. '
  printf 'The harness constructs both exact irrational roots of `x^2 - 2` with CGAL/Gmpq.\n\n'
  printf '| Benchmark | CGAL | Median | p05 | p95 | Checksum |\n'
  printf '| --- | --- | ---: | ---: | ---: | ---: |\n'
  printf '| `%s` | `%s` | %s ns | %s ns | %s ns | %s |\n' \
    "$benchmark_id" "$cgal_version" "$median_ns" "$p05_ns" "$p95_ns" "$checksum"
} > "$temporary_report"
mv -- "$temporary_report" "$report_path"
trap - EXIT
printf 'updated %s\n' "$report_path"
