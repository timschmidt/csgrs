#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${CARGO_TARGET_DIR:-${repo_dir}/target/coverage}"
if [[ "${target_dir}" != /* ]]; then
    target_dir="${repo_dir}/${target_dir}"
fi
profile_dir="${target_dir}/profraw"
profile_data="${target_dir}/hyperlimit.profdata"
report_dir="${target_dir}/html"
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
rm -f "${profile_dir}"/*.profraw "${profile_data}"
find "${report_dir}" -mindepth 1 -delete

cd "${repo_dir}"
export CARGO_TARGET_DIR="${target_dir}"
# The metadata salt prevents Cargo from reusing an instrumented proc-macro or
# build artifact that embedded an older, relative LLVM profile destination.
export RUSTFLAGS="${RUSTFLAGS:+${RUSTFLAGS} }-C instrument-coverage -C metadata=hyperlimit_coverage_v1"
export LLVM_PROFILE_FILE="${profile_dir}/hyperlimit-%p-%m.profraw"

mapfile -t test_objects < <(
    cargo test --all-features --no-run --message-format=json |
        jq -r '
            select(.reason == "compiler-artifact")
            | select(.profile.test == true)
            | select(.executable != null)
            | .executable
        '
)

if [[ ${#test_objects[@]} -eq 0 ]]; then
    echo "Cargo did not report any test executables" >&2
    exit 1
fi

cargo test --all-features --quiet

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

ignore_regex='/(\.cargo/registry|\.rustup|rustc|hyperreal|hyperlattice|target|tests|benches|examples|fuzz)/'

echo "Instrumented Rust source (inline #[cfg(test)] modules included):"
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

# Every inline unit-test module in this crate is a top-level, trailing
# `#[cfg(test)] mod tests`. LLVM's file summary cannot distinguish those lines
# from production lines because both map to the same source path. Derive a
# second physical-line summary from llvm-cov's non-instantiated annotation and
# stop each file at that explicit module boundary. Unit-test executions still
# contribute counts to production lines; only the test harness definitions are
# excluded from the denominator.
echo
echo "Production executable lines (trailing inline test modules excluded):"
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
            boundary = 999999
            source_line_number = 0
            while ((getline source_line < file) > 0) {
                source_line_number++
                if (source_line ~ /^[[:space:]]*#\[cfg\(test\)\][[:space:]]*$/) {
                    boundary = source_line_number
                    break
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
    '

echo "HTML report: ${report_dir}/index.html"
