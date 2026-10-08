# Checkpoint 80 — Hyper inverse-angle capability comparison

This is a read-only public capability check, not an optimization or speed
comparison. Bind the unchanged 957-file retained source map, a small audit-only
crate, its lock and driver before compilation. Reuse the existing shared target,
two build jobs, disabled incrementality and disabled dev debug information.

Execute identical debug/release programs, then compare ordered output and full
certificates. Preserve every `Unknown`; never interpret it as inequality.
Every `Equal`/`NotEqual` must agree with the independently derived identity or
nonidentity. The Node checker derives fractions independently from the same exact
field/cubic oracle used to validate the donor inputs.

- 1,728 angle rows: asin/acos at pi/12 and atan/derived-acot at pi/24, full two
  periods, stored/radical/2^1025-period forms, -64/-256 predicate floors, identity
  and +1/1024 unequal target. Poles are asserted and emitted without invalid values.
- 32 golden quadratic rows; 48 cyclotomic cubic rows. Cubic Hyper input is a
  `cos_pi` expression, not a generic imported algebraic minimal polynomial.
- 16 near-input rows, perturbed by +/-2^-100 against the original inverse angle.
  Monotonicity proves nonidentity; -64 is allowed to remain unknown.
- 16 positive/negative denominator-3360 rows, stored/huge-period, both predicate
  floors and identity/unequal target. These retain a compact angle certificate;
  they are not a matched representation or cost comparison with FLINT degree 384.
- Eight explicit rational asin/acos domain-error rows. Total 1,848 + terminal.

The acot comparison is explicitly derived as atan(1/x), with +pi/2 at zero,
matching FLINT's (-pi/2,pi/2] branch. There is no claim that Hyper exposes an acot
API. Complex logarithm and all general algebraic inputs are outside this probe.
No production files are edited; full workspace regression/benchmark/size gates
are not claimed for this audit-only executable. Fmt, Clippy, locked offline
metadata, both profiles, independent output checking and corruption controls are
required before publishing capability results.
