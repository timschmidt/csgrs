#!/usr/bin/env bash
# Repeated package verification must consume a changed dependency's archive,
# even when its version and the compiler target directory are unchanged.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${root}/scripts/release-package-cache.sh"
cargo_command=(cargo)
if [[ -n "${1:-}" ]]; then cargo_command+=("$1"); fi
work="$(mktemp -d "${TMPDIR:-/tmp}/csgrs-package-cache-test.XXXXXX")"
mkdir -p "${CARGO_TARGET_DIR:-${root}/target}"
build="$(mktemp -d "${CARGO_TARGET_DIR:-${root}/target}/package-cache-test.XXXXXX")"
trap 'rm -rf "${work}" "${build}"' EXIT
mkdir -p "${work}/child/src" "${work}/parent/src" "${work}/baseline-home"
cat > "${work}/Cargo.toml" <<'TOML'
[workspace]
members = ["child", "parent"]
resolver = "3"
TOML
for package in child parent; do
  cat > "${work}/${package}/Cargo.toml" <<TOML
[package]
name = "csgrs-release-cache-${package}-fixture"
version = "0.1.0"
edition = "2024"
license = "MIT"
description = "Local package-cache acceptance fixture"
repository = "https://github.com/timschmidt/csgrs"
TOML
done
cat >> "${work}/parent/Cargo.toml" <<'TOML'
[dependencies]
child = { package = "csgrs-release-cache-child-fixture", version = "0.1.0", path = "../child" }
TOML
echo 'pub fn original() -> u32 { 41 }' > "${work}/child/src/lib.rs"
echo 'pub fn value() -> u32 { child::original() }' > "${work}/parent/src/lib.rs"
package_args=(package --locked -p csgrs-release-cache-child-fixture -p csgrs-release-cache-parent-fixture)
fixture_crates=(csgrs-release-cache-child-fixture csgrs-release-cache-parent-fixture)
(
  cd "${work}"
  CARGO_HOME="${work}/baseline-home" "${cargo_command[@]}" generate-lockfile --offline
  CARGO_HOME="${work}/baseline-home" CARGO_TARGET_DIR="${build}" "${cargo_command[@]}" "${package_args[@]}"
)

# This new symbol independently witnesses consumption of the new archive.
echo 'pub fn original() -> u32 { 41 } pub fn newly_added() -> u32 { 43 }' > "${work}/child/src/lib.rs"
echo 'pub fn value() -> u32 { child::newly_added() }' > "${work}/parent/src/lib.rs"
clear_release_package_builds "${build}" "${fixture_crates[@]}"
if (
  cd "${work}"
  CARGO_HOME="${work}/baseline-home" CARGO_TARGET_DIR="${build}" "${cargo_command[@]}" "${package_args[@]}"
) > "${work}/stale.log" 2>&1; then
  echo 'negative control unexpectedly consumed the changed archive; update the Cargo cache witness' >&2
  exit 1
fi
if ! python3 - "${work}/stale.log" <<'PY'
from pathlib import Path
import sys
text = Path(sys.argv[1]).read_text()
assert 'E0425' in text and 'newly_added' in text, text
PY
then
  cat "${work}/stale.log" >&2
  exit 1
fi
echo 'negative control: cleared compiler artifacts still use stale synthetic-registry source'
clear_release_package_builds "${build}" "${fixture_crates[@]}"
CARGO_HOME="${work}/baseline-home" prepare_release_package_home "${work}/fresh-home"
(
  cd "${work}"
  CARGO_HOME="${work}/fresh-home" CARGO_TARGET_DIR="${build}" "${cargo_command[@]}" "${package_args[@]}"
)
echo 'package-cache acceptance passed'
