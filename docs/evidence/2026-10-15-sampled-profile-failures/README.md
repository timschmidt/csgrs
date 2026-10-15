# Sampled-profile failures and composition

This slice applies `properties.md` and `docs/admission.md`, chiefly P1, P4,
P6, P8–P9, P17–P20, P25–P30 and P35–P39. CSGRS owns sampled-profile parameter
validation and construction facts; Hypercurve owns topology. The public Rust
constructors, teardrop solid composition, WASM, examples, benchmarks and fuzz
target share the same implementation. The previous goal turn made progress in
`d41fc3601`, `573bdcb8f` and `2271ad6c5`. The full plan remains active.

The migration covers arrow, trapezoid, star, teardrop, egg, squircle, pie_slice,
heart and supershape. Preserve existing positive dimensions and range/sample
restrictions, the full README and benchmark resolutions, and successful exact
sample rows. Primary APIs return `Result<CurveRegion2, ValidationError>`;
context variants retain the consumed certainty. Do not publish scalar, predicate,
capacity or native failure as empty success. Teardrop-cylinder composition must
propagate construction errors and join its certainty with extrusion certainty.
The four WASM constructors retain their signatures and expose typed error codes.

Equal-width positive arrows have duplicate shoulder endpoints in the old sampler;
clamp-free equality evidence permits omitting only those duplicates. The intended
shaft-plus-head area must survive. Other successful sample formulas remain
unchanged. These are sampled sets, with no analytic-fidelity or end-to-end source
provenance claim. Precision, total work, scalar growth, heap bytes and cancellation
remain different contracts: only primary buffer capacity/reservation is checked
here. Large trials use external wall/address-space containment.

## Preserved evidence

- `baseline-v1`, based on `2271ad6c5`: nine full README fixtures pass and fifteen
  assertions fail. Fourteen are actual error/completeness defects. One oracle
  incorrectly demanded an error from the valid collinear one-chord semicircle.
- `candidate-v1`: 23 of 24 tests pass. Investigation establishes that Hypercurve
  successfully regularizes that collinear set to empty; it is not an error
  swallowed by a wrapper. Its assertion is corrected to require Certified
  empty geometry and downstream union/extrusion composition. No production
  behavior is weakened to hide a native failure; the original failed oracle
  and trial remain preserved.
- `candidate-v2`: all 30 expanded tests and the existing all-feature library
  tests pass. Independent rational/algebraic areas cover every migrated family.
  Exact volume 9 and a separate manifold check cover an arrow with a hole and
  teardrop extrusion; the dyadic arrow survives OBJ value-for-value. Approximate
  arrow shoulder equality remains qualified even when a strict identical ring
  is cached. Every input-controlled primary buffer rejects capacity overflow.
- All nine original README fixtures preserve their exact serialized coordinate
  rows byte-for-byte. The serializers retain exact scalar expressions and omit
  accelerator caches. This comparison supplements independent semantic oracles;
  matching rows/checksums alone would not establish the intended set.

`compiler-consolidation-v1` shares 19,367,511,727 bytes of identical non-executable
compiler artifacts in terminal trials, retaining every path and byte and checking
all 2,485 protected artifacts before and after. Executables, unique outputs,
source snapshots and logs remain untouched. These closed targets must never be
resumed with Cargo; future trials copy seeds with separate inodes.

Final native/WASM qualification and proportionate cost/allocation/size evidence
are pending. The original 35 pending native files remain byte-identical and await
separate admission. Full CI/pre-push, external-package/release, directed fuzz,
complete image-corpus and all remaining plan requirements stay open. No push.
