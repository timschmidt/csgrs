#!/usr/bin/env bash
# Release check for the packaged stack (docs/admission.md, "The shipped user
# path"). It packages every released crate, verifies each package builds from
# its own contents, then builds a clean consumer project outside the
# repository against the packaged sources only. The consumer runs the csgrs
# user path, every README quick start and the packaged curve-range example.
#
# Usage: scripts/release-check.sh [--no-verify] [--allow-dirty] [--toolchain <name>]
#
# Before publication, crates.io cannot serve these versions, so the consumer
# patches each released crate name to its unpacked .crate contents. Those
# contents are what `cargo publish` uploads; no workspace path is visible to
# the consumer. Publication itself remains outside this check.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${root}"
source "${root}/scripts/release-package-cache.sh"

verify=1
dirty=()
toolchain=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-verify) verify=0 ;;
    --allow-dirty) dirty=(--allow-dirty) ;;
    --toolchain) toolchain="+$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

cargo_command=(cargo)
[[ -z "${toolchain}" ]] || cargo_command+=("${toolchain}")

# Released crates in dependency order. hypervoxel, hyperbrep, hyperbrep-io
# and hyperspatial are not part of this release.
crates=(
  hyperreal hyperinterval hyperlattice-interval hyperlattice hyperlimit hypertri
  hypersolve hyperdynamics hypercurve hypermesh
  hyperphysics hypersdf csgrs csgrs-ffi hypergraphics hyperevolution hyperpack
  hyperparts hyperpath hyperdrc hypercircuit
)
# Consumer quick starts and packaged examples that need non-default features.
declare -A quickstart_features=(
  [hypertri]="earcut"
  [hyperinterval]="fixed"
  [hyperlattice-interval]="fixed"
  [hyperdynamics]="field"
  [hypercurve]="interval-ranges"
)

target="${CARGO_TARGET_DIR:-${root}/target}/release-check"
# Unpacked packages and the consumer live outside the repository, so Cargo
# cannot attach them to its workspace; build outputs stay under target/.
work="$(mktemp -d "${TMPDIR:-/tmp}/csgrs-release-check.XXXXXX")"
trap 'rm -rf "${work}"' EXIT
mkdir -p "${work}/packages"
rm -rf "${target}/package"

# Fixed package timestamps can retain both old builds and old synthetic-registry
# sources. Clear owned builds and isolate only package verification's Cargo home;
# ordinary crates.io caches stay shared and the temporary home follows work's trap.
clear_release_package_builds "${target}" "${crates[@]}"
prepare_release_package_home "${work}/cargo-home"
bash "${root}/scripts/test-release-package-cache.sh" "${toolchain}"

package_args=()
for crate in "${crates[@]}"; do package_args+=(-p "${crate}"); done
# hypersdf's optional hypervoxel adapter needs hypervoxel to resolve while
# packaging. hypervoxel is not released yet: crates.io must have it before
# hypersdf can be published (see docs/RELEASE.md).
overlay=(--config 'patch.crates-io.hypervoxel.path="crates/hypervoxel"')
verify_args=()
[[ ${verify} -eq 1 ]] || verify_args=(--no-verify)
echo "== packaging ${#crates[@]} crates"
CARGO_HOME="${work}/cargo-home" CARGO_TARGET_DIR="${target}" "${cargo_command[@]}" package --locked \
  "${overlay[@]}" "${verify_args[@]}" "${dirty[@]}" "${package_args[@]}"

echo "== documenting local docs.rs feature and rustdoc configurations"
python3 "${root}/scripts/test-release-docs.py" "${toolchain}"
python3 "${root}/scripts/release-docs.py" --toolchain="${toolchain}" \
  --target-dir "${target}" "${overlay[@]}" "${crates[@]}"

# Unpack exactly what would be uploaded.
declare -A version
for crate in "${crates[@]}"; do
  archives=("${target}/package/${crate}"-[0-9]*.crate)
  if [[ ${#archives[@]} -ne 1 || ! -f "${archives[0]}" ]]; then
    echo "expected exactly one verified archive for ${crate}" >&2
    exit 1
  fi
  archive="${archives[0]}"
  tar -xzf "${archive}" -C "${work}/packages"
  base=$(basename "${archive}" .crate)
  version[${crate}]="${base#"${crate}"-}"
done

consumer="${work}/consumer"
mkdir -p "${consumer}/src/bin"
{
  echo '[package]'
  echo 'name = "release-consumer"'
  echo 'version = "0.0.0"'
  echo 'edition = "2024"'
  echo 'publish = false'
  echo
  echo '[dependencies]'
  for crate in "${crates[@]}"; do
    features="${quickstart_features[${crate}]:-}"
    if [[ -n "${features}" ]]; then
      echo "${crate} = { version = \"=${version[${crate}]}\", features = [\"${features}\"] }"
    else
      echo "${crate} = \"=${version[${crate}]}\""
    fi
  done
  echo
  echo '[patch.crates-io]'
  for crate in "${crates[@]}"; do
    echo "${crate} = { path = \"../packages/${crate}-${version[${crate}]}\" }"
  done
  echo
  echo '[workspace]'
} > "${consumer}/Cargo.toml"

cp scripts/release-consumer/user_path.rs "${consumer}/src/bin/user_path.rs"
echo 'fn main() {}' > "${consumer}/src/main.rs"
for crate in "${crates[@]}"; do
  [[ ${crate} == csgrs-ffi ]] && continue
  readme="${work}/packages/${crate}-${version[${crate}]}/README.md"
  [[ ${crate} == csgrs ]] && readme="${work}/packages/csgrs-${version[csgrs]}/readme.md"
  python3 - "${readme}" "${consumer}/src/bin/quickstart_${crate//-/_}.rs" <<'EOF'
import re, sys
text = open(sys.argv[1]).read()
block = re.search(r'<!-- quickstart:start -->\s*```rust\n(.*?)```\s*<!-- quickstart:end -->', text, re.S)
if block is None:
    sys.exit(f"{sys.argv[1]}: no Rust quick start between quickstart markers")
open(sys.argv[2], "w").write(block.group(1))
EOF
done

# Exercise the shipped optional API with the example from the actual archive.
cp "${work}/packages/hypercurve-${version[hypercurve]}/examples/interval_ranges.rs" \
  "${consumer}/src/bin/curve_interval_ranges.rs"

echo "== building the consumer against packaged sources"
(cd "${consumer}" && CARGO_TARGET_DIR="${target}/consumer" "${cargo_command[@]}" build --bins)

# Every released crate must resolve from the unpacked packages, never from
# the repository.
python3 - "${consumer}" "${root}" "${toolchain}" "${crates[@]}" <<'EOF'
import json, os, subprocess, sys
consumer, root, toolchain = sys.argv[1], os.path.realpath(sys.argv[2]), sys.argv[3]
crates = set(sys.argv[4:])
cargo = ["cargo"] + ([toolchain] if toolchain else [])
# The lock file the build just wrote fixes the resolution. Metadata reads
# every locked package's manifest, including packages for other platforms
# that the build never downloaded, so it may fetch them.
meta = json.loads(subprocess.check_output(
    cargo + ["metadata", "--format-version", "1", "--locked"], cwd=consumer))
packages = os.path.realpath(os.path.join(consumer, "..", "packages"))
for package in meta["packages"]:
    path = os.path.realpath(package["manifest_path"])
    if package["name"] in crates and not path.startswith(packages + os.sep):
        sys.exit(f"{package['name']} resolved outside the packages: {path}")
    if package["source"] is None and package["name"] != "release-consumer" \
            and not path.startswith(packages + os.sep):
        sys.exit(f"{package['name']} is a path dependency: {path}")
print(f"resolution: {len(crates)} released crates from packages, nothing from {root}")
EOF

echo "== building C and C++ consumers of the packaged csgrs-ffi"
ffi="${work}/packages/csgrs-ffi-${version[csgrs-ffi]}"
patches=()
for crate in "${crates[@]}"; do
  patches+=(--config "patch.crates-io.${crate}.path=\"${work}/packages/${crate}-${version[${crate}]}\"")
done
(cd "${ffi}" && CARGO_TARGET_DIR="${target}/ffi" "${cargo_command[@]}" build "${patches[@]}" --lib)
cc -std=c11 -Wall -Wextra -Werror -I "${ffi}/include" scripts/release-consumer/ffi_smoke.c \
  "${target}/ffi/debug/libcsgrs_ffi.a" -lm -lpthread -ldl -o "${work}/ffi_smoke"

c++ -std=c++17 -Wall -Wextra -Werror -I "${ffi}/include" -I bindings/cpp \
  scripts/release-consumer/bindings_smoke.cpp "${target}/ffi/debug/libcsgrs_ffi.a" \
  -lm -lpthread -ldl -o "${work}/cpp_smoke"

echo "== running the user path, README quick starts, curve ranges and C and C++ consumers"
run_dir="${work}/run"
mkdir -p "${run_dir}"
run() {
  local name="$1"
  shift
  printf '%-30s ' "${name}"
  (cd "${run_dir}" && "$@" > "${run_dir}/${name}.log" 2>&1) || {
    echo "FAILED"
    cat "${run_dir}/${name}.log"
    exit 1
  }
  echo "ok"
}
for binary in "${target}/consumer/debug/user_path" "${target}"/consumer/debug/quickstart_* \
    "${target}/consumer/debug/curve_interval_ranges" \
    "${work}/ffi_smoke" "${work}/cpp_smoke"; do
  [[ -x ${binary} && -f ${binary} ]] || continue
  run "$(basename "${binary}")" "${binary}"
done

# The other language bindings wrap the same packaged library and header.
echo "== running the Python, Go and TypeScript bindings"
library="${target}/ffi/debug/libcsgrs_ffi.so"
run python_bindings env PYTHONPATH="${root}/bindings/python" \
  python3 "${root}/scripts/release-consumer/bindings_smoke.py" "${library}"
go_module="${work}/go"
mkdir -p "${go_module}"
cp bindings/go/*.go "${go_module}/"
printf 'module csgrs\n\ngo 1.22\n' > "${go_module}/go.mod"
run go_bindings env -C "${go_module}" CGO_CFLAGS="-I${ffi}/include" \
  CGO_LDFLAGS="-L${target}/ffi/debug -lcsgrs_ffi" LD_LIBRARY_PATH="${target}/ffi/debug" \
  go test ./...
# The TypeScript compiler the bindings are checked with locally.
tsc=(tsc)
command -v tsc > /dev/null || tsc=(npx --yes --package=typescript@5.7.3 tsc)
run typescript_bindings "${tsc[@]}" --noEmit --target ES2020 --strict "${root}/bindings/js/csgrs.ts"
echo "release check passed"
