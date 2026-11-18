# Solver interval-box proof scope

## Certified radius domain

Phase 7 belongs to Hypersolve. The public interval and Krawczyk certifiers accept
a candidate and a list of exact real radii. Each supplied radius must be
nonnegative before it can bound an enclosure. The witness concerns P1, P6,
P8, P9 and P19. It exercises the distinction between a policy-authorized
approximate decision and proof of an input precondition.

On `97fe4e072`, the sine addition identity makes the following radius exactly
negative, even though its terminal comparison with zero returns approximate
equality.

```text
sin(e + 1) - sin(e) cos(1) - cos(e) sin(1) - 2^-2048
```

All three residual packages certified the constant equality on that input.
The affine Krawczyk certifier for `x = 0` and both quadratic Krawczyk certifiers
for `x² + x = 0` returned `CertifiedUniqueRoot` at zero. The supplied interval
was invalid. Commit `e45bb6b62` restricts the four radius comparisons to
certified decisions. Both policies now return the existing
`UnknownVariableRadiusSign` diagnostic for the hidden negative. The public
box facade reports `InvalidInput` with no certificate. An ordinary negative
radius still receives its proved-negative diagnostic. Zero and positive
radii can be used on the same analysis and context after rejection.
[Regression tests](../../crates/hypersolve/tests/interval_box_radius.rs)

## Completed checks

| Check | Observed result |
| --- | --- |
| Rust 1.95.0 Hypersolve, all features | 1,523 tests and doctests passed, including the two-root and unequal-radius Krawczyk controls |
| Hypersolve, no default features | 1,388 tests and doctests passed |
| Hyperdynamics reachability | All 11 downstream integration tests passed |
| Static checks | Workspace formatting, owner all-feature/all-target Clippy with warnings denied, owner all-feature rustdoc with warnings denied, repository budget passed |
| Native release client | All 84 ordinary box replay records had identical verdicts, full evidence, dispatch traces and allocation observations |
| WASM release client | All 240 ordered calls per revision passed in two fresh instances with zero imports. Fixed baseline, previous primary and repaired versions reached 1,310,720 linear-memory bytes from 1,245,184 |

The native client covered two policies, three packages, two centers and seven
constraint and radius configurations. The separate native and WASM domain fixtures exercised
exact zero, one, negative one, a hidden negative, and its positive shift.
The native Criterion comparison exercised nine owner workloads, including
coupled Krawczyk systems through dimension eight. An exploratory timing run
during concurrent builds produced an outlier. In the four-block comparison,
affine Krawczyk appeared 2.54% slower. An independent eight-block pinned rerun
measured candidate medians of 3.842–3.919 µs and control medians of
3.869–3.872 µs. The larger affine slowdown did not repeat. The raw samples
are retained. No speed improvement is claimed.

## Bounded stack and linked size

The Linux qualification client ran one complete construction, proof attempt
and result inspection per call. Across 60 workloads, two temperatures, two
sentinel bytes and 24 fresh processes per revision, the largest **per-workload**
increment in observed stack was 32 bytes. The largest cumulative charge was
48 bytes. This includes observed increases between fixed baseline `c9f1f9ac5`
and the previous primary. After the pilot, the confirmation run used caps of
128 bytes and 1% per change, 256 bytes and 2% cumulative, separately for each
workload and temperature. The minimum remaining thread headroom across all
three builds was 2,072,776 bytes. Caller frames and the guard are accounted
for in headroom. The measured call includes nested solver work and cleanup.
The per-workload ledger debits increases even where process-to-process
variation contributed to a difference between maxima.
[Stack accounts and limits](interval-memory-accounts.json)

Peak requested heap across the campaign was 15,036 bytes on all three
revisions. Each pair of completed calls returned to zero live requested
and allocator-usable bytes. Maximum observed glibc usable peaks were 16,536,
16,504 and 16,504 bytes respectively. Requested bytes, usable bytes, allocation
traffic, process RSS and wasm linear memory are separate observations. The
qualification covers these serial host calls. It establishes no firmware or
arbitrary-input memory bound.

| Final linked image | Previous primary | Retained | Change |
| --- | ---: | ---: | ---: |
| Native owner certification benchmark | 16,928,328 | 16,928,288 | −40 bytes |
| Native replay observer | 3,790,984 | 3,791,000 | +16 bytes |
| WASM interval and Krawczyk client | 1,765,964 | 1,766,030 | +66 bytes |
| WASM code section | 1,569,561 | 1,569,586 | +25 bytes |

Every measured native allocated section and the union of defined function
ranges held its size. The native replay observer carries 56 bytes of positive
file growth from the fixed program revision. The WASM file and code charges
remain separate. The linked clients fit the 64 MiB host image allowance with
at least 16 MiB free. [Target size accounts](interval-size-accounts.json)

## Unresolved whole-box certificates

The public facade also aggregates row results that apply only at the center.
For example, `x³ = 0` holds at zero and fails at one. Both lie in `[-1,1]`
and `[0,2]`. On the previous primary the corresponding centers yielded
`Certified` and `Violation` respectively. Neither aggregate verdict describes
the whole box. A separate fixture reproduced a certified aggregate from an
approximate ball receipt. Both issues remain in the retained implementation.

The review branch `interval-box-replay-coverage-review` at `245d4bb43`
preserves a broader candidate and its six regressions. It passed 1,527
all-feature owner tests and doctests. Across the 84 native rows it corrected
24 aggregate verdicts while preserving all inner evidence, dispatch and heap
records. Its linked client's allocated RELRO padding grew from 1,376 to
1,992 bytes and a sparse alternative grew it by 800 bytes. Both exceeded the
existing 1% per-section cap. The smaller file and function union cannot
pay for another section's growth.

The archived commands, fixed/control/candidate executables, failed fixtures,
per-workload observations and raw timing samples are indexed under
`2026-11-18-interval-box-radius`. [Qualification archive](../evidence-archive.tsv)
