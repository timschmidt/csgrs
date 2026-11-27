#!/usr/bin/env bash
set -euo pipefail

# Build and report LLVM execution coverage for selected Cargo configurations.
# This is not Rustdoc coverage or a correctness/completeness gate: no percentage
# threshold is enforced. Compiler/LLVM/jq tools and Cargo environment are inherited.
# Relative CARGO_TARGET_DIR values are interpreted from this crate directory.
# Before builds, delete this target's prior profraw/profdata/text/object-list output
# and every entry under its html directory; do not point it at unrelated reports.
# Full tests/targets can be expensive; this script supplies no total resource bound.

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${CARGO_TARGET_DIR:-${repo_dir}/target/coverage}"
if [[ "${target_dir}" != /* ]]; then
    target_dir="${repo_dir}/${target_dir}"
fi
profile_dir="${target_dir}/profraw"
profile_data="${target_dir}/hyperlattice.profdata"
report_dir="${target_dir}/html"
text_report="${target_dir}/coverage.txt"
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
rm -f "${profile_dir}"/*.profraw "${profile_data}" "${text_report}" "${object_manifest}"
find "${report_dir}" -mindepth 1 -delete

cd "${repo_dir}"
export CARGO_TARGET_DIR="${target_dir}"
coverage_rustflags="${RUSTFLAGS:+${RUSTFLAGS} }-C instrument-coverage"
export LLVM_PROFILE_FILE="${profile_dir}/hyperlattice-%p-%m.profraw"

# Append test-profile or bin artifacts having an executable path. This can retain
# more than one feature build; no source ownership or coverage validation occurs.
record_test_objects() {
    jq -r '
        select(.reason == "compiler-artifact")
        | select(
            .profile.test == true
            or (.target.kind | index("bin") != null)
        )
        | select(.executable != null)
        | .executable
    ' >>"${object_manifest}"
}

# Collect no-run artifacts, then execute lib/integration tests with a distinct
# metadata suffix. This separates symbol namespaces, not all environment/cache state.
run_configuration() {
    local label="$1"
    local metadata="$2"
    shift 2
    local cargo_args=("$@")

    echo "Coverage configuration: ${label}"
    # The no-feature and all-feature builds contain different bodies for the
    # dispatch-trace facade. Isolated symbol namespaces keep both profiles
    # valid when LLVM merges them.
    export RUSTFLAGS="${coverage_rustflags} -C metadata=hyperlattice_coverage_${metadata}"
    cargo test "${cargo_args[@]}" --lib --tests --no-run --message-format=json |
        record_test_objects
    cargo test "${cargo_args[@]}" --lib --tests --quiet
}

# Select the no-default and all-feature endpoints. These do not exhaust all
# feature combinations or dependency-only cache configurations. Compiling a branch
# does not establish that execution covered it or that its result was correct.
run_configuration "no features" no_features --no-default-features
run_configuration "all features" all_features --all-features

# Include all-feature targets in Cargo test mode. Actual execution depends on each
# target's harness: compiling/testing an example need not call its main function.
# Request benchmark-report suppression through the named environment flag; targets
# that do not honor it may still perform their own output writes.
echo "Coverage configuration: all-feature targets and benchmark fixtures"
cargo test --all-features --all-targets --no-run --message-format=json |
    record_test_objects
HYPERLATTICE_SKIP_BENCHMARK_REPORTS=1 \
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

# This path-pattern filter is not a positive restriction to this crate's src tree;
# other local workspace dependencies may remain in LLVM's report.
ignore_regex='/(\.cargo/registry|\.rustup|rustc|hyperreal|target|tests|benches|examples|fuzz)/'

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

# Derive a heuristic displayed-line summary from LLVM's text report. Recognize
# only a standalone cfg(test) attribute followed by blank lines and `mod tests {`;
# intervening docs/attributes or other test-module forms are not recognized.
# Everything at/after the first recognized boundary is excluded, including any
# later production items. Without a boundary, the sentinel is line 999999.
# Nonempty displayed count fields are counted; any text other than literal "0"
# counts as hit. This is not unique-line/branch/function coverage or a Rust parser.
# File headings must be absolute .rs paths. Relative labels assume the crate prefix,
# and the final total has no zero-denominator guard. No threshold is enforced.
echo
echo "Production executable lines (trailing inline test modules excluded):"
"${llvm_cov}" show \
    "${primary_object}" \
    "${object_args[@]}" \
    --instr-profile="${profile_data}" \
    --ignore-filename-regex="${ignore_regex}" \
    --format=text \
    --show-instantiations=false \
    --show-line-counts-or-regions >"${text_report}"
awk -F'|' -v prefix="${repo_dir}/" '
        /^\/.*\.rs:$/ {
            file = $0
            sub(/:$/, "", file)
            boundary = 999999
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
            printf "%-42s %8s %8s %8s %9s\n", "Source", "Lines", "Hit", "Missed", "Coverage"
            for (row = 1; row <= file_count; row++) {
                file = order[row]
                relative = substr(file, length(prefix) + 1)
                missed = total[file] - hit[file]
                coverage = total[file] ? 100 * hit[file] / total[file] : 100
                printf "%-42s %8d %8d %8d %8.2f%%\n", relative, total[file], hit[file], missed, coverage
                sum += total[file]
                sum_hit += hit[file]
            }
            printf "%-42s %8d %8d %8d %8.2f%%\n", "TOTAL", sum, sum_hit, sum - sum_hit, 100 * sum_hit / sum
        }
    ' "${text_report}"

echo "HTML report: ${report_dir}/index.html"
echo "Annotated text report: ${text_report}"
