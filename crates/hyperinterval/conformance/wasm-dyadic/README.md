# WASM dyadic-to-IEEE conversion

This separate allocating consumer executes the exact 56-case corpus shared with
native tests. It covers both widths, signs, subnormal boundaries, huge mantissas,
extreme exponents and finite overflow. Assertions trap on failure. The existing
float-only consumer remains allocation-free. Run through
`scripts/check_interval_wasm.py`; `check.mjs` and `browser.mjs` accept `--egress`
for this module. No JavaScript floating oracle or native rounding mode is used.
