#!/usr/bin/env bash
set -euo pipefail

size_harness_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
size_feature_set="${1:-default}"

case "$size_feature_set" in
    default)
        size_feature_args=()
        ;;
    all)
        size_feature_args=(--features hypermesh-all)
        ;;
    *)
        echo "usage: $0 [default|all]" >&2
        exit 2
        ;;
esac

size_target_root="${HYPERMESH_SIZE_TARGET_DIR:-$size_harness_dir/target/$size_feature_set}"
size_manifest="$size_harness_dir/Cargo.toml"
size_wasm_target="wasm32-unknown-unknown"

report_artifact() {
    local size_label="$1"
    local size_artifact="$2"
    local size_kind="$3"

    printf '%s\n' "$size_label"
    stat -c 'file_bytes=%s' "$size_artifact"
    sha256sum "$size_artifact"
    printf 'gzip_9_bytes='
    gzip -9 -c "$size_artifact" | wc -c
    if command -v brotli >/dev/null 2>&1; then
        printf 'brotli_11_bytes='
        brotli -q 11 -c "$size_artifact" | wc -c
    fi

    if [[ "$size_kind" == native ]] && command -v size >/dev/null 2>&1; then
        size "$size_artifact"
    fi

    if [[ "$size_kind" == wasm ]] && command -v wasm-opt >/dev/null 2>&1; then
        local size_optimized="${size_artifact%.wasm}.opt.wasm"
        wasm-opt --all-features -Oz "$size_artifact" -o "$size_optimized"
        stat -c 'wasm_opt_oz_bytes=%s' "$size_optimized"
        printf 'wasm_opt_oz_gzip_9_bytes='
        gzip -9 -c "$size_optimized" | wc -c
        if command -v brotli >/dev/null 2>&1; then
            printf 'wasm_opt_oz_brotli_11_bytes='
            brotli -q 11 -c "$size_optimized" | wc -c
        fi
    fi
}

rustc -Vv
printf 'feature_set=%s\n' "$size_feature_set"
printf 'target_dir=%s\n' "$size_target_root"

for size_profile in release size; do
    for size_binary in hypermesh-size-harness immediate; do
        cargo build \
            --locked \
            --manifest-path "$size_manifest" \
            --profile "$size_profile" \
            --bin "$size_binary" \
            --target-dir "$size_target_root" \
            "${size_feature_args[@]}"

        size_native_artifact="$size_target_root/$size_profile/$size_binary"
        report_artifact \
            "native/$size_profile/$size_feature_set/$size_binary" \
            "$size_native_artifact" \
            native

        cargo build \
            --locked \
            --manifest-path "$size_manifest" \
            --profile "$size_profile" \
            --bin "$size_binary" \
            --target "$size_wasm_target" \
            --target-dir "$size_target_root" \
            "${size_feature_args[@]}"

        size_wasm_artifact="$size_target_root/$size_wasm_target/$size_profile/$size_binary.wasm"
        report_artifact \
            "wasm/$size_profile/$size_feature_set/$size_binary" \
            "$size_wasm_artifact" \
            wasm
    done
done
