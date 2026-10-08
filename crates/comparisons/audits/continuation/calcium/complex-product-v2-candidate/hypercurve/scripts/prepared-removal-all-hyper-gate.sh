#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: scripts/prepared-removal-all-hyper-gate.sh \
    --immediate-evidence PATH --artifacts DIR [--correctness-only|--performance-only]

Run the second-tier cross-stack gate for removing prepared machinery below
Hypercurve. PATH must be a nonempty report from the already-passed immediate
API gate, including its correctness, executed timing, Callgrind, Heaptrack,
binary-size, and static-callgraph results.

The default runs all-feature correctness tests and every benchmark target for
every hyper crate. Logs and a source-state manifest are written beneath DIR.
Performance logs are evidence for review; a successful process exit alone is
not an assertion that timings did not regress.
EOF
}

immediate_evidence=
artifact_dir=
run_correctness=true
run_performance=true

while (($# > 0)); do
    case "$1" in
        --immediate-evidence)
            immediate_evidence=${2-}
            shift 2
            ;;
        --artifacts)
            artifact_dir=${2-}
            shift 2
            ;;
        --correctness-only)
            run_performance=false
            shift
            ;;
        --performance-only)
            run_correctness=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "$immediate_evidence" || ! -s "$immediate_evidence" ]]; then
    echo "error: --immediate-evidence must name a nonempty first-tier gate report" >&2
    exit 2
fi
if [[ -z "$artifact_dir" ]]; then
    echo "error: --artifacts is required" >&2
    exit 2
fi

immediate_evidence=$(realpath "$immediate_evidence")
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace_root=$(cd "$script_dir/../.." && pwd)
mkdir -p "$artifact_dir"
artifact_dir=$(cd "$artifact_dir" && pwd)

crates=(
    hyperbrep
    hypercircuit
    hypercurve
    hyperdrc
    hyperevolution
    hypergraphics
    hyperlattice
    hyperlimit
    hypermesh
    hyperpack
    hyperparts
    hyperpath
    hyperphysics
    hyperreal
    hypersdf
    hypersolve
    hypertri
    hypervoxel
)

manifest="$artifact_dir/source-state.txt"
{
    echo "immediate_evidence=$immediate_evidence"
    sha256sum "$immediate_evidence"
    for crate in "${crates[@]}"; do
        crate_dir="$workspace_root/$crate"
        if [[ ! -f "$crate_dir/Cargo.toml" ]]; then
            echo "error: missing $crate_dir/Cargo.toml" >&2
            exit 2
        fi
        printf '%s ' "$crate"
        git -C "$crate_dir" rev-parse HEAD
        git -C "$crate_dir" status --short
    done
} >"$manifest"

if [[ "$run_correctness" == true ]]; then
    for crate in "${crates[@]}"; do
        echo "== correctness: $crate =="
        (
            cd "$workspace_root/$crate"
            cargo test --workspace --all-features --all-targets
        ) 2>&1 | tee "$artifact_dir/$crate-correctness.log"
    done
fi

if [[ "$run_performance" == true ]]; then
    for crate in "${crates[@]}"; do
        echo "== performance: every $crate benchmark =="
        (
            cd "$workspace_root/$crate"
            cargo bench --workspace --all-features
        ) 2>&1 | tee "$artifact_dir/$crate-all-benchmarks.log"
    done
fi

{
    for crate in "${crates[@]}"; do
        printf '%s ' "$crate"
        git -C "$workspace_root/$crate" rev-parse HEAD
        git -C "$workspace_root/$crate" status --short
    done
} >"$artifact_dir/source-state-after.txt"

echo "Cross-stack commands completed. Review every performance log before accepting the removal."
