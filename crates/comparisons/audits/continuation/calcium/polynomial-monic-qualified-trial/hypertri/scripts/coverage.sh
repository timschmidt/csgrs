#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${CARGO_TARGET_DIR:-${repo_dir}/target/coverage}"
if [[ "${target_dir}" != /* ]]; then
    target_dir="${repo_dir}/${target_dir}"
fi
profile_dir="${target_dir}/profraw"
profile_data="${target_dir}/hypertri.profdata"
report_dir="${target_dir}/html"
text_report="${target_dir}/coverage.txt"
object_manifest="${target_dir}/test-objects.txt"
minimum_production_line_coverage="${HYPERTRI_MIN_PRODUCTION_LINE_COVERAGE:-95}"
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
export LLVM_PROFILE_FILE="${profile_dir}/hypertri-%p-%m.profraw"

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

run_configuration() {
    local label="$1"
    local metadata="$2"
    shift 2
    local cargo_args=("$@")

    echo "Coverage configuration: ${label}"
    # Isolated symbol namespaces allow LLVM to merge bodies compiled under
    # different feature gates without treating them as profile mismatches.
    export RUSTFLAGS="${coverage_rustflags} -C metadata=hypertri_coverage_${metadata}"
    cargo test "${cargo_args[@]}" --lib --tests --no-run --message-format=json |
        record_test_objects
    cargo test "${cargo_args[@]}" --lib --tests --quiet
}

# Cover the published minimal surface, each single planar algorithm's runtime
# selector branches, and the complete native/serde/trace/f64 surface.
run_configuration "no features" no_features --no-default-features
run_configuration "runtime selector without executor" runtime_only \
    --no-default-features --features runtime-select
run_configuration "earcut runtime" earcut_runtime \
    --no-default-features --features earcut,runtime-select
run_configuration "CDT runtime" cdt_runtime \
    --no-default-features --features cdt,runtime-select
run_configuration "all features" all_features --all-features

# Criterion test mode executes each benchmark fixture once. Examples and the
# harness-free dispatch tracer are included as build/run targets as well.
echo "Coverage configuration: all-feature targets and benchmark fixtures"
export RUSTFLAGS="${coverage_rustflags} -C metadata=hypertri_coverage_all_targets"
cargo test --all-features --all-targets --no-run --message-format=json |
    record_test_objects
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

ignore_regex='/([.]cargo/registry|[.]rustup|rustc|hyperreal|hyperlattice|hyperlimit|delaunator|earcutr|target|tests|benches|examples|fuzz)/'

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

# LLVM's file table combines production code with trailing inline unit tests.
# Stop at an explicit test-gated `mod tests` boundary (including multiline
# `#[cfg(all(test, ...))]` attributes) while retaining the production-line hit
# counts caused by all integration and unit executions.
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
awk -F'|' -v prefix="${repo_dir}/" -v threshold="${minimum_production_line_coverage}" '
        /^\/.*[.]rs:$/ {
            file = $0
            sub(/:$/, "", file)
            boundary = 999999
            source_line_number = 0
            pending_cfg_test = 0
            in_cfg_attribute = 0
            cfg_attribute_start = 0
            cfg_attribute_has_test = 0
            while ((getline source_line < file) > 0) {
                source_line_number++
                if (in_cfg_attribute != 0) {
                    if (source_line ~ /(^|[^[:alnum:]_])test([^[:alnum:]_]|$)/) {
                        cfg_attribute_has_test = 1
                    }
                    if (source_line ~ /\)\][[:space:]]*$/) {
                        in_cfg_attribute = 0
                        if (cfg_attribute_has_test != 0) {
                            pending_cfg_test = cfg_attribute_start
                        }
                    }
                    continue
                }
                if (source_line ~ /^[[:space:]]*#\[cfg\(/) {
                    cfg_attribute_start = source_line_number
                    cfg_attribute_has_test = source_line ~ /(^|[^[:alnum:]_])test([^[:alnum:]_]|$)/
                    if (source_line ~ /\)\][[:space:]]*$/) {
                        if (cfg_attribute_has_test != 0) {
                            pending_cfg_test = cfg_attribute_start
                        }
                    } else {
                        in_cfg_attribute = 1
                    }
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
            coverage = 100 * sum_hit / sum
            printf "%-42s %8d %8d %8d %8.2f%%\n", "TOTAL", sum, sum_hit, sum - sum_hit, coverage
            if (coverage < threshold) {
                printf "production line coverage %.4f%% is below required %.4f%%\n", coverage, threshold > "/dev/stderr"
                exit 2
            }
        }
    ' "${text_report}"

echo "Production coverage threshold: ${minimum_production_line_coverage}%"
echo "HTML report: ${report_dir}/index.html"
echo "Annotated text report: ${text_report}"
