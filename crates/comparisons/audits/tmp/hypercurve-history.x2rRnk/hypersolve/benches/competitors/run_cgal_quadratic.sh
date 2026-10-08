#!/usr/bin/env bash
set -euo pipefail

compiler="${CXX:-g++}"
binary="${TMPDIR:-/tmp}/hypersolve-cgal-quadratic"

"${compiler}" \
  -O3 \
  -DNDEBUG \
  -std=c++17 \
  "$(dirname "$0")/cgal_quadratic.cpp" \
  -o "${binary}" \
  -lgmpxx \
  -lgmp

"${binary}"
