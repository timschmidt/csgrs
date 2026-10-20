#!/usr/bin/env bash
# Enforce this repository's side of the evidence retention budget
# (docs/admission.md, "Retention budget"). Generated evidence belongs in the
# evidence repository described in docs/evidence-archive.md.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

max_file_bytes=$((4 * 1024 * 1024))
max_total_bytes=$((256 * 1024 * 1024))
archived_roots=(
  docs/evidence
  benchmarks/baselines
  'crates/*/docs/evidence'
  crates/comparisons/audits
  crates/comparisons/integration
  crates/hyperspatial/verification
  crates/hyperreal/benchmarks/checkpoints
)
# Complete PCB board inputs for hyperdrc workloads.
allowed_large=(
  "crates/hyperdrc/docs/HVP109A.zip"
)

status=0
tracked=$(git ls-files -- "${archived_roots[@]}")
if [[ -n "${tracked}" ]]; then
  echo "error: generated evidence is tracked; commit it to the evidence repository:"
  head -n 20 <<<"${tracked}"
  status=1
fi

sizes=$(git ls-files --format='%(objectname) %(path)' |
  git cat-file --batch-check='%(objectsize) %(rest)')
total=$(awk '{sum += $1} END {print sum + 0}' <<<"${sizes}")
if ((total > max_total_bytes)); then
  echo "error: tracked files total ${total} bytes; the budget is ${max_total_bytes}"
  status=1
fi
while read -r size path; do
  if ((size > max_file_bytes)); then
    allowed=false
    for candidate in "${allowed_large[@]}"; do
      [[ "${path}" == "${candidate}" ]] && allowed=true
    done
    if [[ "${allowed}" == false ]]; then
      echo "error: ${path} has ${size} bytes; the per-file budget is ${max_file_bytes}"
      status=1
    fi
  fi
done <<<"${sizes}"

if ((status == 0)); then
  echo "repository budget: $(wc -l <<<"${sizes}") files, ${total} bytes"
fi
exit "${status}"
