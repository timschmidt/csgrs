# Strict IEEE WebAssembly qualification

This standalone `no_std` consumer links Hyperinterval's `float-enclosure`
feature without an allocator or another dependency. It passes full binary32/64
endpoint bits across the actual WASM ABI, including subnormals and signed zeros.
No JavaScript number conversion supplies an expected real value.

From the repository root, with the selected Rust WASM target, Python, Node
with WebAssembly BigInt support, and Binaryen's `wasm-dis` installed:

```sh
python3 scripts/check_interval_wasm.py --toolchain 1.88.0 --output /tmp/interval-wasm-msrv
node crates/hyperinterval/conformance/wasm-float/browser.mjs \
  /tmp/interval-wasm-msrv/interval_float.wasm /tmp/interval-wasm-msrv/corpus.json
```

The output directory must be new. The optional browser command needs Chromium
(`CHROMIUM` may select its executable); it creates and removes its own profile
and loopback server. It runs the identical verifier in a browser page.

`oracle.py` derives expected results with Python integers and `Fraction`.
Binary search through positive IEEE encodings establishes adjacent directed
bounds; square roots compare exact squares. It covers every finite normal
exponent class, subnormal leading-bit positions, adversarial endpoint classes,
seeded random intervals, typed failures, empty/invalid combinations, clipped
root domains, fused cancellation and output-to-input arithmetic chains. Higher
integer powers use exact small-integer fixtures; general higher-power rounding
is not claimed tight. No native interval implementation produces the oracle.
Separate bit-mask queries verify set relations, signs, membership and exact width
ordering, including widths that cannot be projected to a finite endpoint value.
The downstream lattice module checks native and separate-array chains through
dimension 1024 against exact integer expectations, preserving fixed-point checks.

`query32`/`query64` selectors 0, 1, 2 return status, lower bits and upper bits.
Status 0 is nonempty complete, 1 empty complete, and 2 nonempty partial-domain.
Errors 16–21 are nonfinite input, reversed endpoints, overflow, zero-containing
denominator, empty domain, and unqualified target. Endpoint queries on errors
or empty outputs trap. Flags 1/2 explicitly select empty first/second inputs;
their unused endpoint payloads are ignored. Each query recomputes the bounded
operation, so there is no mutable result cache or stale-output channel.

The driver records exact results, tool versions, source-independent corpus and
module hashes, dependency closure, and generated-code inspection. The whole
module must have no imports, `memory.grow`, or floating arithmetic/conversion;
bit reinterpretation is allowed. This is finite platform qualification, not
exhaustive binary64 proof, a timing benchmark, or evidence for other targets.
The root progress ledger records the actual compiler/runtime versions and scope.
