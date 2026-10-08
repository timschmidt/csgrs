#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${CARGO_TARGET_DIR:-${repo_dir}/target/coverage}"
if [[ "${target_dir}" != /* ]]; then
    target_dir="${repo_dir}/${target_dir}"
fi
profile_dir="${target_dir}/profraw"
profile_data="${target_dir}/hyperreal.profdata"
report_dir="${target_dir}/html"
object_manifest="${target_dir}/test-objects.txt"
target_triple="$(rustc -vV | sed -n 's/^host: //p')"
llvm_bin="$(rustc --print sysroot)/lib/rustlib/${target_triple}/bin"
llvm_cov="${llvm_bin}/llvm-cov"
llvm_profdata="${llvm_bin}/llvm-profdata"

if [[ ! -x "${llvm_cov}" || ! -x "${llvm_profdata}" ]]; then
    echo "coverage requires rustup component add llvm-tools-preview" >&2
    exit 1
fi
if ! command -v jq >/dev/null 2>&1; then
    echo "coverage requires jq to read Cargo's test-artifact manifest" >&2
    exit 1
fi

mkdir -p "${profile_dir}" "${report_dir}"
rm -f "${profile_dir}"/*.profraw "${profile_data}" "${object_manifest}"
find "${report_dir}" -mindepth 1 -delete

cd "${repo_dir}"
export CARGO_TARGET_DIR="${target_dir}"
coverage_rustflags="${RUSTFLAGS:+${RUSTFLAGS} }-C instrument-coverage"
export LLVM_PROFILE_FILE="${profile_dir}/hyperreal-%p-%m.profraw"

run_configuration() {
    local label="$1"
    local metadata="$2"
    shift 2
    local cargo_args=("$@")

    echo "Coverage configuration: ${label}"
    # Separate symbol namespaces avoid profile-hash collisions when the same
    # source function has different feature-gated bodies.
    export RUSTFLAGS="${coverage_rustflags} -C metadata=hyperreal_coverage_${metadata}"
    cargo test "${cargo_args[@]}" --lib --tests --no-run --message-format=json |
        jq -r '
            select(.reason == "compiler-artifact")
            | select(
                .profile.test == true
                or (.target.kind | index("bin") != null)
            )
            | select(.executable != null)
            | .executable
        ' >>"${object_manifest}"
    cargo test "${cargo_args[@]}" --lib --tests --quiet
}

# These configurations cover every compile-time primitive-cache state as well
# as both sides of the serde, Simple, and dispatch-trace feature boundaries.
run_configuration "no features" no_features --no-default-features
run_configuration "binary32 cache" binary32_cache --no-default-features \
    --features cached-f32-approx
run_configuration "binary64 cache" binary64_cache --no-default-features \
    --features cached-f64-approx
run_configuration "both primitive caches" both_primitive_caches --no-default-features \
    --features cached-f32-approx,cached-f64-approx
run_configuration "serde" serde --no-default-features --features serde
run_configuration "Simple expression language" simple --no-default-features --features simple
run_configuration "all features" all_features --all-features

# Criterion test mode executes benchmark fixtures once, with their assertions,
# and examples/binaries are compiled as test targets. Include that validation
# surface once rather than multiplying its very-wide arithmetic cases across
# every feature configuration. The environment flag keeps generated benchmark
# ledgers byte-for-byte stable during a coverage run.
echo "Coverage configuration: all-feature targets and benchmark fixtures"
cargo test --all-features --all-targets --no-run --message-format=json |
    jq -r '
        select(.reason == "compiler-artifact")
        | select(
            .profile.test == true
            or (.target.kind | index("bin") != null)
        )
        | select(.executable != null)
        | .executable
    ' >>"${object_manifest}"
HYPERREAL_SKIP_BENCHMARK_REPORTS=1 \
    cargo test --all-features --all-targets --quiet

mapfile -t test_objects < <(sort -u "${object_manifest}")
if [[ ${#test_objects[@]} -eq 0 ]]; then
    echo "Cargo did not report any test executables" >&2
    exit 1
fi

mapfile -t raw_profiles < <(find "${profile_dir}" -maxdepth 1 -type f -name '*.profraw' -print)
if [[ ${#raw_profiles[@]} -eq 0 ]]; then
    echo "the instrumented tests did not produce any coverage profiles" >&2
    exit 1
fi
"${llvm_profdata}" merge -sparse "${raw_profiles[@]}" -o "${profile_data}"

primary_object="${test_objects[0]}"
object_args=()
for object in "${test_objects[@]:1}"; do
    object_args+=(--object "${object}")
done

ignore_regex='/(\.cargo/registry|\.rustup|rustc|target|tests|benches|examples|fuzz)/'

echo "Instrumented Rust source (inline #[cfg(test)] code included):"
"${llvm_cov}" report \
    "${primary_object}" \
    "${object_args[@]}" \
    --instr-profile="${profile_data}" \
    --ignore-filename-regex="${ignore_regex}"

"${llvm_cov}" show \
    "${primary_object}" \
    "${object_args[@]}" \
    --instr-profile="${profile_data}" \
    --ignore-filename-regex="${ignore_regex}" \
    --format=html \
    --output-dir="${report_dir}" \
    --show-instantiations=false \
    --show-line-counts-or-regions

# LLVM's file summary combines production code with inline unit-test modules.
# Derive a second physical executable-line view from the annotated report,
# excluding dedicated test-only source files and trailing `mod tests { ... }`
# blocks. Executions from those tests still count toward production lines.
echo
echo "Production executable lines (test-only source excluded):"
"${llvm_cov}" show \
    "${primary_object}" \
    "${object_args[@]}" \
    --instr-profile="${profile_data}" \
    --ignore-filename-regex="${ignore_regex}" \
    --format=text \
    --show-instantiations=false \
    --show-line-counts-or-regions |
    awk -F'|' -v prefix="${repo_dir}/" '
        /^\/.*\.rs:$/ {
            file = $0
            sub(/:$/, "", file)
            relative = substr(file, length(prefix) + 1)
            boundary = 999999

            if (relative ~ /(^|\/)tests\.rs$/ || relative == "src/real/normal_reference.rs") {
                boundary = 1
            } else {
                source_line_number = 0
                pending_cfg_test = 0
                while ((getline source_line < file) > 0) {
                    source_line_number++
                    if (source_line ~ /^[[:space:]]*#\[cfg\(test\)\][[:space:]]*$/) {
                        pending_cfg_test = source_line_number
                        continue
                    }
                    if (pending_cfg_test != 0 && source_line ~ /^[[:space:]]*$/) {
                        continue
                    }
                    if (pending_cfg_test != 0 && source_line ~ /^[[:space:]]*mod[[:space:]]+tests[[:space:]]*\{/) {
                        boundary = pending_cfg_test
                        break
                    }
                    if (pending_cfg_test != 0) {
                        pending_cfg_test = 0
                    }
                }
                close(file)
            }

            files[file] = 1
            boundaries[file] = boundary
            next
        }
        file != "" && $1 ~ /^[[:space:]]*[0-9]+$/ {
            line = $1 + 0
            count = $2
            gsub(/[[:space:]]/, "", count)
            if (line < boundaries[file] && count != "") {
                total[file]++
                if (count != "0") {
                    hit[file]++
                }
            }
        }
        END {
            for (file in files) {
                order[++file_count] = file
            }
            for (left = 1; left <= file_count; left++) {
                for (right = left + 1; right <= file_count; right++) {
                    if (order[left] > order[right]) {
                        temporary = order[left]
                        order[left] = order[right]
                        order[right] = temporary
                    }
                }
            }
            printf "%-58s %8s %8s %8s %9s\n", "Source", "Lines", "Hit", "Missed", "Coverage"
            for (row = 1; row <= file_count; row++) {
                file = order[row]
                relative = substr(file, length(prefix) + 1)
                missed = total[file] - hit[file]
                coverage = total[file] ? 100 * hit[file] / total[file] : 100
                printf "%-58s %8d %8d %8d %8.2f%%\n", relative, total[file], hit[file], missed, coverage
                sum += total[file]
                sum_hit += hit[file]
            }
            printf "%-58s %8d %8d %8d %8.2f%%\n", "TOTAL", sum, sum_hit, sum - sum_hit, 100 * sum_hit / sum
        }
    '

echo "HTML report: ${report_dir}/index.html"
