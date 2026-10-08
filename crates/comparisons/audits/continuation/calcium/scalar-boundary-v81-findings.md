# Checkpoint 81 — reciprocal inverses and scalar boundaries

Forty-two more donor files are fully read and independently compared with live
Hyper. All observed float imports and certified integer results are correct.
Reciprocal inverse compositions expose missing Hyper equality proofs, not false
inequalities. No production change is retained; seven continuation improvements
remain in place. This does not complete the ecosystem audit.

## Completed source reads

Both Calcium and current FLINT pins are read independently, first to last line.
Use `qqbar/` for Calcium and `src/qqbar/` for FLINT.

| Files | Calcium lines | FLINT lines |
| --- | ---: | ---: |
| `asec_pi.c`, `acsc_pi.c` and both tests | 202 | 186 |
| `floor.c`, `ceil.c` and both tests | 334 | 314 |
| `set_d.c`, `set_re_im_d.c` and both tests | 239 | 223 |
| `get_fmpq.c`, `get_fmpz.c` | 55 | 53 |
| `numerator.c`, `denominator.c` | 49 | 47 |
| `height.c`, `height_bits.c` | 39 | 41 |
| `swap.c`, `inlines.c`, `phi.c` | 61 | 59 |
| Total: 21 files per pin | 979 | 923 |

New coverage: 42 complete files / 1,902 unique lines. Cumulative continuation
coverage: 1,508 complete files, 20 partial files, 189,257 uniquely read lines.
The already-covered manual ranges and recorded Hyper excerpts are rereads, not
new donor credit. The inline-emission shim does not credit its included header
or callees. [Read records](scalar-boundary-read-records-v81.json) and
[origin](scalar-boundary-origin-v81.json) bind exact ranges and per-file hashes.

The source pins remain those in the inventory: Calcium
`8dbb16fc4fe92eaf3ebbc7478d629e994d39f944`, FLINT
`e269d38061d7a42070ddcffe6eb114466ed4aa7e`. No donor file was changed.

## Architecture and transfer conclusions

1. Reciprocal inverse wrappers guard zero, invert, then delegate to asin/acos.
   Their principal branches are inherited. A failed recognition is not negative
   proof; these wrappers also inherit their recognizer's completeness limitations.
   Failed p/q outputs and null pointers are not used by the audit.
2. Donor floor/ceiling operate on the real part, even for complex values. They
   use exact rational division, a cached interval filter, magnitude-aware
   refinement, then a half-offset integer proposal and exact sign correction.
   The selected integer is never accepted on approximate proximity alone.
3. Hyper already exposes total multivalued `near_integer`, plus bounded certified
   floor/ceiling with explicit Exhausted. Its generic floor searches nine nearby
   candidates. Existing near_integer could restrict that proposal to two candidates,
   but this is **unimplemented and unmeasured**. Proof/cache availability, regression,
   cost and consumer behavior must be checked before retaining such a change.
   Algebraic total directional rounding must not be transplanted as a total
   decision procedure for general computable reals.
4. Hyper already decodes IEEE inputs directly into reduced dyadic rationals,
   avoiding a decimal round trip and general gcd. Its exact class and cached
   primitive view preserve the relevant scalar/matrix boundary. Replacing this
   with the donor's ARF-to-rational sequence has no demonstrated benefit.
5. Failed complex floating imports need not leave the destination unchanged.
   The implementation may import the imaginary component before discovering a
   non-finite real component. Consume only the explicit success result.
6. The donor denominator is the minimal polynomial's leading coefficient, not
   necessarily the smallest multiplier making a value an algebraic integer.
   Its numerator, coefficient height and bit height must be interpreted on that
   contract, not as Hyper's reduced rational numerator/denominator.
7. Swap must move polynomial ownership and its selected-root enclosure together.
   The archive swaps those fields; current FLINT swaps the complete structure.
   Hyper's immutable shared scalar ownership already supplies the corresponding
   value/certificate association; no new manual swap mechanism is justified.

The full upstream test files cover random exact round trips, principal branches,
real-part rounding inequalities and special-float imports. They were read, not
newly executed. The archived acsc test's progress label says asec; that is cosmetic,
not a numeric defect. No unverified source-cleanup patch is kept.

## Native independent qualification

One small collector uses existing pinned FLINT/GMP/MPFR libraries. It emits
19,923 rows plus terminal, with 163,328 independent mathematical checks passing.
Eighteen deliberate corruptions are rejected.

| Family | Observations |
| --- | --- |
| Scalar floats | 18,432 inputs: every binary32/binary64 exponent, both signs, four sampled significands; 18,416 exact finite successes and 16 explicit failures |
| Complex floats | Full 16-by-16 boundary cross product: 121 successes, 135 failures |
| Reciprocal inverse angles | 576 rows: 528 exact recognized values, 48 poles; all pi/12 residues, three periods, factors 1 and 3, two cache states |
| Reciprocal inverse controls | Twelve rows: four recognized +/-1 cases, eight zero/domain/imaginary failures |
| Floor/ceiling | 644 rows across seven origins through +/-2^256, 23 rational/quadratic offsets, real/complex inputs and two cache states |
| Other helpers | Three selected phi/swap/extraction records |

IEEE bit fields are decoded independently into exact integer ratios. The
trigonometric oracle uses Q(sqrt(2),sqrt(3)) signs and Galois products. Rounding
uses exact Q(sqrt(2)) inequalities and independently derived complex minimal
polynomials; inputs include offsets down to +/-2^-1024 and half-boundary offsets.
The corpus checks every input polynomial and both enclosure components. Denominator
and height are checked against the full polynomial. Numerator **polynomials** are
checked through exact invertible scaling, in separate/in-place modes; this is not
an independent numerator selected-root enclosure check.

Native and Memcheck streams match byte for byte: 9,799,577 bytes. Memcheck reports
zero errors and zero live blocks, with 1,682,521 allocations/frees and
2,220,444,799 cumulative requested bytes. Those are entire-collector totals,
not peak memory or per-operation costs. The native command has 1 GiB address-space,
90 CPU-second and 120 wall-second caps; the instrumented run has a separate
180-second wall cap. Both commands complete successfully. Only current FLINT is
executed; the archive is source evidence.

Sixteen serialized component intervals use exponents below -8192, down to -16383.
These are valid unusually tight intervals, not failed arithmetic. The initial
checker inherited v80's too-small resource guard. A new bounded decoder permits
exponent magnitude 32768 and at most 12000 characters per integer, retaining
the same exact containment, ordering and <=2^-96 width checks. Thirty shared-range
comparisons and malformed/over-limit controls pass. The original failing checker
and capture remain unchanged. No mathematical acceptance tolerance was loosened.

## Hyper qualification

The unchanged 957-file live map and 184-file isolated previous candidate are
rechecked. A small audit-only crate produces 19,340 rows plus terminal in both
debug and release. Complete streams match, including error variants and
certificates: 4,819,943 bytes per profile. Twelve corruptions are rejected.

- All 18,416 finite float cases have the independently expected canonical exact
  numerator/denominator and cached f64 export bits. Signed-zero views are checked
  separately from the mathematical zero. Four infinities and twelve NaNs return
  their distinct errors.
- All 322 floor results and all 322 ceilings are certified and correct. All 322
  near_integer results are mathematically adjacent integers. Repeating on the
  same retained expression preserves every result. This phase is not the donor's
  explicit high-precision enclosure-polishing phase.
- Of 528 valid reciprocal inverse angle rows, 336 prove equality and 192 remain
  Unknown. The latter represent sixteen op/residue inputs repeated over periods,
  unreduced factors and predicate floors—not 192 independent defects. None is
  incorrectly declared unequal.
- Forty-eight forward reciprocal poles and two zero inverse controls return
  DivideByZero; four out-of-domain controls return NotANumber; four +/-1 inverse
  controls prove equality. Both forward and inverse reciprocal functions here
  are explicit compositions, not claims that Hyper exposes sec_pi/csc_pi or
  asec/acsc APIs.

Formatting, warning-denied Clippy, locked offline metadata and both profiles pass.
The complete resolved graph has 21 packages/nodes with Hyperreal as the sole live
Hyper dependency. This is bounded public scalar capability, not a fresh whole-stack
regression, WASM runtime, product-size comparison or benchmark.

## Preserved development evidence and storage

Four failed captures remain explicit: native checker exponent guard; audit Rust
build using three absent API names; its enclosing driver; and the first Hyper
checker conflating direct division's DivideByZero with a trig API's NotANumber.
The Rust corrections are exactly the borrowed export name and two explicit
reciprocal constructions. Both original and corrected source are retained.
The error-classification correction changes only the checker, not any library
result. Two earlier successful formatter/metadata captures are development gates.

Combined evidence passes 2026-09-12T02:55:02.191Z: one nonempty result record,
empty stderr, code zero. Sealing and sealed verification are separate gates.

Reuse the native libraries and existing shared Rust target. One C executable
adds 22,880 bytes in `/tmp/calcium-scalar-boundary-v81.nLhDoT`; unique shared-target
debug/release audit executables total 5,628,112 bytes. The three bound executables
total 5,650,992 bytes; this is storage accounting, not representative product size.
The post-build `/tmp` snapshot has 13,904,842,752 bytes available. No broad source
copy, library rebuild or cleanup is performed. Both tracked donor trees remain
clean and scoped Git whitespace checks pass.

```sh
node verify-scalar-boundary-v81.mjs
```

Continue the remaining quadratic/expression/complex support files, formal,
symbolic and historical references, and full inventory reconciliation. Qualify
the two-candidate rounding idea separately if it merits implementation. No new
production transfer, external report, commit or push occurs in this checkpoint.
