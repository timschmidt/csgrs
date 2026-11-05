#!/usr/bin/env bash
# Shared cache boundaries for package verification and its acceptance test.

clear_release_package_builds() {
  local release_target="$1"
  shift
  local build_path build_profile crate_name
  for build_path in "${release_target}" "${release_target}/consumer" "${release_target}/ffi"; do
    for build_profile in debug release; do
      for crate_name in "$@"; do
        rm -rf "${build_path}/${build_profile}/.fingerprint/${crate_name}-"* \
          "${build_path}/${build_profile}/build/${crate_name}-"* \
          "${build_path}/${build_profile}/deps/lib${crate_name//-/_}-"* \
          "${build_path}/${build_profile}/deps/${crate_name//-/_}-"*
      done
    done
  done
}

prepare_release_package_home() {
  local destination="$1"
  local shared_cargo_home="${CARGO_HOME:-${HOME}/.cargo}"
  local cache_kind cached
  if [[ -e "${destination}" ]]; then
    echo "package verification requires a fresh Cargo home: ${destination}" >&2
    return 1
  fi
  if [[ -d "${shared_cargo_home}" ]]; then
    shared_cargo_home="$(cd "${shared_cargo_home}" && pwd)"
  fi
  for cache_kind in index cache src; do
    mkdir -p "${destination}/registry/${cache_kind}"
    # Share public crates.io caches only. Cargo's synthetic package registry
    # uses other directory names; its same-version sources must start fresh.
    for cached in "${shared_cargo_home}/registry/${cache_kind}/"index.crates.io-* \
      "${shared_cargo_home}/registry/${cache_kind}/github.com-1ecc6299db9ec823"; do
      [[ -d "${cached}" ]] || continue
      ln -s "${cached}" "${destination}/registry/${cache_kind}/${cached##*/}"
    done
  done
}
