#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
stack_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)

repositories=(
  hyperreal
  hyperlattice
  hyperlimit
  hypersolve
  hypercurve
  hypertri
  hypermesh
  hyperphysics
  hypersdf
  hyperbrep
  hyperpath
  hyperdrc
  hypervoxel
  hyperpack
  hyperparts
  hyperevolution
  hypergraphics
  hypercircuit
  csgrs
  csgrs-ffi
  openscad-rs
  synaps-cad
)

printf '%s\n' '# Stack Exactness-Policy File Audit'
printf '\n'
printf '%s\n' \
  'Generated from the live worktree by `scripts/generate_stack_exactness_file_audit.sh`.' \
  'The audit universe is every Rust source, Cargo manifest, and build script in each' \
  'crate that implements or consumes the Hyper exact-arithmetic stack. Generated' \
  'artifacts, vendored dependencies, binary assets, and prose-only files are excluded.' \
  'Tests, examples, benches, and fuzz targets are included because they can encode or' \
  'conceal incorrect predicate assumptions.'
printf '\n'
printf '%s\n' \
  '`hypervoxel`, `hyperbrep`, and Hypersdf'\''s optional Hypervoxel adapter are' \
  'intentionally deferred from this pass at the user'\''s request. Their entries remain unchecked' \
  'and do not count toward this pass'\''s completion.'
printf '\n'
printf '%s\n' '## Status legend'
printf '\n'
printf '%s\n' \
  '- `[ ]`: not yet manually reviewed.' \
  '- `[x]`: reviewed against the centralized Hyperlimit policy and predicate boundary.' \
  '- Signal tags are search aids, not findings. `policy` means direct Hyperlimit policy' \
  '  or predicate use; `scalar` means local sign/order/equality or approximation code;' \
  '  `none` means no lexical signal and still requires a manual applicability check.'

for repository in "${repositories[@]}"; do
  repository_root="$stack_root/$repository"
  printf '\n'
  printf '## `%s`\n\n' "$repository"

  while IFS= read -r relative_path; do
    absolute_path="$repository_root/$relative_path"
    if [[ ! -f "$absolute_path" ]]; then
      continue
    fi
    signals=()

    if rg -q \
      'hyperlimit|PredicatePolicy|PredicateOutcome|classify_real_sign|compare_reals|orient[23d]?|incircle|insphere' \
      "$absolute_path"; then
      signals+=(policy)
    fi

    if rg -q \
      'to_f(32|64)|lossy|approx|epsilon|EPSILON|partial_cmp|total_cmp|signum|is_sign_|is_zero|known_sign|refine_(sign|zero)|certified_dyadic_interval|[<>=!]=?[[:space:]]*0([._[:alnum:]]*)?' \
      "$absolute_path"; then
      signals+=(scalar)
    fi

    if ((${#signals[@]} == 0)); then
      signals=(none)
    fi

    signal_text=$(IFS=,; printf '%s' "${signals[*]}")
    if [[ "$repository" == hypervoxel || "$repository" == hyperbrep ]]; then
      disposition='deferred at user request'
    else
      disposition='pending'
    fi
    printf -- '- [ ] `%s/%s` — signals: `%s`; disposition: %s\n' \
      "$repository" "$relative_path" "$signal_text" "$disposition"
  done < <(
    git -C "$repository_root" ls-files --cached --others --exclude-standard |
      rg '(^|/)(Cargo\.toml|build\.rs)$|\.rs$' |
      sort -u
  )
done
