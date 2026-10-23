#!/usr/bin/env bash
# Reproducible artifact-size attribution for root CSGRS consumers, the C ABI,
# and the WASM bindings. Every artifact is reported with its file bytes, its
# code and read-only data bytes separately from its debug sections, its
# deployed stripped size, compressed sizes, and a content hash.
set -euo pipefail

size_harness_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
size_repo_root="$(cd -- "${size_harness_dir}/../.." && pwd)"
size_scope="${1:-all}"

case "$size_scope" in
    primitive | cad | raster | abi | wasm | wasm-documented | attribute | perf | all) ;;
    *)
        echo "usage: $0 [primitive|cad|raster|abi|wasm|wasm-documented|attribute|perf|all]" >&2
        exit 2
        ;;
esac

size_target_root="${CSGRS_SIZE_TARGET_DIR:-$size_harness_dir/target}"
size_wasm_target="wasm32-unknown-unknown"
# Embedded-profile optimization levels to compare. The repository ships "z".
read -r -a size_opt_levels <<<"${CSGRS_SIZE_OPT_LEVELS:-z s}"
# Timed iterations per run and number of runs in the `perf` scope.
size_perf_repeats="${CSGRS_SIZE_PERF_REPEATS:-200}"
size_perf_runs="${CSGRS_SIZE_PERF_RUNS:-5}"

section_report() {
    local section_artifact="$1"
    local section_kind="$2"

    if [[ "$section_kind" == native ]]; then
        command -v readelf >/dev/null 2>&1 || return 0
        readelf -S -W "$section_artifact" | python3 -I -c '
import re
import sys

text = rodata = alloc = debug = symbols = 0
pattern = re.compile(r"^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)")
for line in sys.stdin:
    match = pattern.match(line)
    if not match:
        continue
    name, kind, addr, _off, size = match.groups()
    size = int(size, 16)
    if kind == "NOBITS":
        continue
    if name.startswith(".debug"):
        debug += size
        continue
    if name in {".symtab", ".strtab", ".comment"}:
        symbols += size
        continue
    if int(addr, 16) != 0:
        alloc += size
        if name == ".text":
            text += size
        elif name.startswith(".rodata") or name in {".rela.dyn", ".eh_frame"}:
            rodata += size
print(f"text_bytes={text}")
print(f"rodata_and_unwind_bytes={rodata}")
print(f"allocated_bytes={alloc}")
print(f"symbol_bytes={symbols}")
print(f"debug_bytes={debug}")
'
        if command -v strip >/dev/null 2>&1; then
            local section_stripped="${section_artifact}.stripped"
            cp -- "$section_artifact" "$section_stripped"
            strip -s -- "$section_stripped"
            stat -c 'stripped_file_bytes=%s' "$section_stripped"
            printf 'stripped_gzip_9_bytes='
            gzip -9 -c -- "$section_stripped" | wc -c
            rm -f -- "$section_stripped"
        fi
        return 0
    fi

    python3 -I -c '
import sys

WASM_SECTIONS = {
    0: "custom", 1: "type", 2: "import", 3: "function", 4: "table",
    5: "memory", 6: "global", 7: "export", 8: "start", 9: "element",
    10: "code", 11: "data", 12: "data_count",
}

blob = open(sys.argv[1], "rb").read()
offset = 8
code = data = name_custom = debug_custom = other = 0
while offset < len(blob):
    section_id = blob[offset]
    offset += 1
    size = 0
    shift = 0
    while True:
        byte = blob[offset]
        offset += 1
        size |= (byte & 0x7F) << shift
        shift += 7
        if not byte & 0x80:
            break
    body = blob[offset : offset + size]
    offset += size
    kind = WASM_SECTIONS.get(section_id, "unknown")
    if kind == "custom":
        name_len = 0
        shift = 0
        cursor = 0
        while True:
            byte = body[cursor]
            cursor += 1
            name_len |= (byte & 0x7F) << shift
            shift += 7
            if not byte & 0x80:
                break
        custom_name = body[cursor : cursor + name_len].decode("utf-8", "replace")
        if custom_name.startswith(".debug") or custom_name == "name":
            if custom_name == "name":
                name_custom += size
            else:
                debug_custom += size
        else:
            other += size
    elif kind == "code":
        code += size
    elif kind == "data":
        data += size
    else:
        other += size
print(f"wasm_code_bytes={code}")
print(f"wasm_data_bytes={data}")
print(f"wasm_name_section_bytes={name_custom}")
print(f"wasm_debug_section_bytes={debug_custom}")
print(f"wasm_other_section_bytes={other}")
' "$section_artifact"
}

report_artifact() {
    local size_label="$1"
    local size_artifact="$2"
    local size_kind="$3"

    printf '%s\n' "$size_label"
    stat -c 'file_bytes=%s' "$size_artifact"
    sha256sum "$size_artifact"
    printf 'gzip_9_bytes='
    gzip -9 -c -- "$size_artifact" | wc -c
    if command -v brotli >/dev/null 2>&1; then
        printf 'brotli_11_bytes='
        brotli -q 11 -c -- "$size_artifact" | wc -c
    fi
    section_report "$size_artifact" "$size_kind"

    if [[ "$size_kind" == wasm ]] && command -v wasm-opt >/dev/null 2>&1; then
        local size_optimized="${size_artifact%.wasm}.opt.wasm"
        wasm-opt --all-features -Oz "$size_artifact" -o "$size_optimized"
        stat -c 'wasm_opt_oz_bytes=%s' "$size_optimized"
        printf 'wasm_opt_oz_gzip_9_bytes='
        gzip -9 -c -- "$size_optimized" | wc -c
        if command -v brotli >/dev/null 2>&1; then
            printf 'wasm_opt_oz_brotli_11_bytes='
            brotli -q 11 -c -- "$size_optimized" | wc -c
        fi
    fi
    printf '\n'
}

# Build one harness binary and report the native and WASM artifacts.
harness_build() {
    local harness_bin="$1"
    local harness_features="$2"
    local harness_profile="$3"
    local harness_opt="$4"
    local harness_label="$5"

    local harness_args=(
        --locked
        --manifest-path "$size_harness_dir/Cargo.toml"
        --profile "$harness_profile"
        --bin "$harness_bin"
        --target-dir "$size_target_root/harness"
    )
    if [[ -n "$harness_features" ]]; then
        harness_args+=(--features "$harness_features")
    fi

    local harness_env=()
    if [[ -n "$harness_opt" ]]; then
        harness_env=("CARGO_PROFILE_$(printf '%s' "$harness_profile" | tr '[:lower:]' '[:upper:]')_OPT_LEVEL=$harness_opt")
    fi

    env "${harness_env[@]}" cargo build "${harness_args[@]}"
    report_artifact \
        "native/$harness_label/$harness_bin" \
        "$size_target_root/harness/$harness_profile/$harness_bin" \
        native

    env "${harness_env[@]}" cargo build "${harness_args[@]}" --target "$size_wasm_target"
    report_artifact \
        "wasm/$harness_label/$harness_bin" \
        "$size_target_root/harness/$size_wasm_target/$harness_profile/$harness_bin.wasm" \
        wasm
}

# Build the C ABI library and a real C consumer linked against the static form.
abi_build() {
    local abi_profile="$1"
    local abi_opt="$2"
    local abi_label="$3"

    local abi_env=()
    if [[ -n "$abi_opt" ]]; then
        abi_env=("CARGO_PROFILE_$(printf '%s' "$abi_profile" | tr '[:lower:]' '[:upper:]')_OPT_LEVEL=$abi_opt")
    fi

    env "${abi_env[@]}" cargo build \
        --locked \
        --manifest-path "$size_repo_root/Cargo.toml" \
        --package csgrs-ffi \
        --lib \
        --profile "$abi_profile" \
        --target-dir "$size_target_root/root"

    local abi_dir="$size_target_root/root/$abi_profile"
    report_artifact "abi/$abi_label/cdylib" "$abi_dir/libcsgrs_ffi.so" native

    if command -v cc >/dev/null 2>&1; then
        local abi_consumer="$abi_dir/abi_consumer"
        cc -O2 -o "$abi_consumer" \
            -I"$size_repo_root/crates/csgrs-ffi/include" \
            "$size_harness_dir/consumer/abi_consumer.c" \
            "$abi_dir/libcsgrs_ffi.a" \
            -lm -lpthread -ldl
        "$abi_consumer" 12 1
        report_artifact "abi/$abi_label/c_consumer" "$abi_consumer" native

        # The same consumer with unreferenced sections discarded at link time.
        local abi_consumer_gc="$abi_dir/abi_consumer_gc"
        cc -O2 -Wl,--gc-sections -o "$abi_consumer_gc" \
            -I"$size_repo_root/crates/csgrs-ffi/include" \
            "$size_harness_dir/consumer/abi_consumer.c" \
            "$abi_dir/libcsgrs_ffi.a" \
            -lm -lpthread -ldl
        "$abi_consumer_gc" 12 1
        report_artifact "abi/$abi_label/c_consumer_gc_sections" "$abi_consumer_gc" native
    fi
}

# Build the WASM bindings. `binding` matches scripts/test-wasm-bindings.sh
# (`--no-default-features --features wasm`); `documented` matches the README
# and wasm-pack command (`--features wasm` with the default features).
wasm_bindings_build() {
    local bindings_profile="$1"
    local bindings_opt="$2"
    local bindings_label="$3"
    local bindings_mode="${4:-binding}"

    local bindings_features=(--no-default-features --features wasm)
    if [[ "$bindings_mode" == documented ]]; then
        bindings_features=(--features wasm)
        bindings_label="$bindings_label-default-features"
    fi

    local bindings_env=()
    if [[ -n "$bindings_opt" ]]; then
        bindings_env=("CARGO_PROFILE_$(printf '%s' "$bindings_profile" | tr '[:lower:]' '[:upper:]')_OPT_LEVEL=$bindings_opt")
    fi

    env "${bindings_env[@]}" cargo build \
        --locked \
        --manifest-path "$size_repo_root/Cargo.toml" \
        --package csgrs \
        --lib \
        "${bindings_features[@]}" \
        --profile "$bindings_profile" \
        --target "$size_wasm_target" \
        --target-dir "$size_target_root/root"

    local bindings_raw="$size_target_root/root/$size_wasm_target/$bindings_profile/csgrs.wasm"
    report_artifact "wasm-bindings/$bindings_label/raw" "$bindings_raw" wasm

    local bindgen_bin="${WASM_BINDGEN:-wasm-bindgen}"
    if ! command -v "$bindgen_bin" >/dev/null 2>&1; then
        printf 'wasm_bindgen=absent\n\n'
        return 0
    fi
    local locked_version
    locked_version="$(python3 -I -c '
import sys, tomllib
with open(sys.argv[1], "rb") as lock:
    packages = tomllib.load(lock)["package"]
print(next(p["version"] for p in packages if p["name"] == "wasm-bindgen"))
' "$size_repo_root/Cargo.lock")"
    if [[ "$("$bindgen_bin" --version)" != "wasm-bindgen ${locked_version}" ]]; then
        printf 'wasm_bindgen=version_mismatch expected=%s found=%s\n\n' \
            "$locked_version" "$("$bindgen_bin" --version)"
        return 0
    fi
    local bindings_out="$size_target_root/root/wasm-bindings-$bindings_label"
    "$bindgen_bin" --target web --out-name csgrs --out-dir "$bindings_out" "$bindings_raw"
    report_artifact "wasm-bindings/$bindings_label/bindgen" "$bindings_out/csgrs_bg.wasm" wasm
    stat -c 'bindgen_js_bytes=%s' "$bindings_out/csgrs.js"
    stat -c 'bindgen_dts_bytes=%s' "$bindings_out/csgrs.d.ts"
    printf '\n'
}

# Attribute retained code to crates for each embedded consumer.
attribute_runs() {
    if ! cargo bloat --version >/dev/null 2>&1; then
        printf 'cargo_bloat=absent\n'
        return 0
    fi
    local attribute_entry
    for attribute_entry in primitive: composed:cad raster:raster; do
        local attribute_bin="${attribute_entry%%:*}"
        local attribute_features="${attribute_entry#*:}"
        local attribute_args=(
            --locked
            --manifest-path "$size_harness_dir/Cargo.toml"
            --profile embedded
            --bin "$attribute_bin"
            --target-dir "$size_target_root/harness"
            --crates
            -n 25
        )
        if [[ -n "$attribute_features" ]]; then
            attribute_args+=(--features "$attribute_features")
        fi
        printf 'attribute/embedded-opt-z/%s\n' "$attribute_bin"
        env CARGO_PROFILE_EMBEDDED_OPT_LEVEL=z cargo bloat "${attribute_args[@]}"
        printf '\n'
    done
}

# Compare completed-workload latency across embedded optimization levels.
# Every level is built first and copied aside; runs then alternate between
# levels so host drift affects each level alike.
perf_runs() {
    local perf_bin="$1"
    local perf_features="$2"
    local perf_copies=()

    for perf_opt in "${size_opt_levels[@]}"; do
        local perf_args=(
            --locked
            --manifest-path "$size_harness_dir/Cargo.toml"
            --profile embedded
            --bin "$perf_bin"
            --target-dir "$size_target_root/harness"
        )
        if [[ -n "$perf_features" ]]; then
            perf_args+=(--features "$perf_features")
        fi
        env "CARGO_PROFILE_EMBEDDED_OPT_LEVEL=$perf_opt" cargo build "${perf_args[@]}"
        local perf_copy="$size_target_root/harness/embedded/$perf_bin-opt-$perf_opt"
        cp -- "$size_target_root/harness/embedded/$perf_bin" "$perf_copy"
        perf_copies+=("$perf_opt:$perf_copy")
    done

    for ((perf_run = 1; perf_run <= size_perf_runs; perf_run += 1)); do
        local perf_entry
        for perf_entry in "${perf_copies[@]}"; do
            printf 'perf/embedded-opt-%s/%s/run-%s ' "${perf_entry%%:*}" "$perf_bin" "$perf_run"
            "${perf_entry#*:}" 12 "$size_perf_repeats" | tr '\n' ' '
            printf '\n'
        done
    done
}

rustc -Vv
printf 'scope=%s\n' "$size_scope"
printf 'target_dir=%s\n' "$size_target_root"
printf 'opt_levels=%s\n' "${size_opt_levels[*]}"
printf '\n'

if [[ "$size_scope" == attribute ]]; then
    attribute_runs
    exit 0
fi

if [[ "$size_scope" == perf ]]; then
    printf 'perf_repeats=%s perf_runs=%s\n' "$size_perf_repeats" "$size_perf_runs"
    perf_runs primitive ""
    perf_runs composed cad
    exit 0
fi

if [[ "$size_scope" == primitive || "$size_scope" == all ]]; then
    harness_build primitive "" release "" "release"
    for size_opt in "${size_opt_levels[@]}"; do
        harness_build primitive "" embedded "$size_opt" "embedded-opt-$size_opt"
    done
fi

if [[ "$size_scope" == cad || "$size_scope" == all ]]; then
    harness_build composed cad release "" "release"
    for size_opt in "${size_opt_levels[@]}"; do
        harness_build composed cad embedded "$size_opt" "embedded-opt-$size_opt"
    done
fi

if [[ "$size_scope" == raster || "$size_scope" == all ]]; then
    harness_build raster raster release "" "release"
    for size_opt in "${size_opt_levels[@]}"; do
        harness_build raster raster embedded "$size_opt" "embedded-opt-$size_opt"
    done
fi

if [[ "$size_scope" == abi || "$size_scope" == all ]]; then
    abi_build release "" "release"
    for size_opt in "${size_opt_levels[@]}"; do
        abi_build embedded "$size_opt" "embedded-opt-$size_opt"
    done
fi

if [[ "$size_scope" == wasm || "$size_scope" == all ]]; then
    wasm_bindings_build wasm "" "wasm"
    for size_opt in "${size_opt_levels[@]}"; do
        wasm_bindings_build embedded "$size_opt" "embedded-opt-$size_opt"
    done
fi

if [[ "$size_scope" == wasm-documented || "$size_scope" == all ]]; then
    wasm_bindings_build wasm "" "wasm" documented
fi
