#!/usr/bin/env bash
set -euo pipefail
for spec in '1 1' '3 3' '8 3' '20 20'; do
  read -r degree order <<< "$spec"
  for variant in before after fast fast after before before after fast fast after before; do
    timeout 30s taskset -c 6 /home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555/release/endpoint_fast "$variant" "$degree" "$order"
  done
done
