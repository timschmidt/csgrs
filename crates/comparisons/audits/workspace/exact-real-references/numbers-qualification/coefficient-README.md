# Inverse-series coefficient width repair

Status: scalar repair qualified and retained uncommitted; the broad ecosystem
audit remains open. The two reported Hypercurve failures reproduce identically
without the coefficient repair; see the checkpoint below, not an inference of
blanket green CI.

## Cause and repair

The frozen `series-limit-README.md` describes the original failure checkpoint.
At n=23,171, `(2*n-1)^2 = 2,147,488,281` exceeds i32::MAX. At192,000 requested
bits, exact input1/16 legitimately reaches this coefficient. Before the repair,
all four rational/generic asin/asinh paths panic in debug; in release all four
fail their directed-MPFR enclosure checks. These are in-domain requests, not
misuse of the earlier series scheduling hint. No donor code was copied.

One inline helper now computes positive numerator and denominator in u64,
widening before multiplication. For every1<=n<=i32::MAX,2*n<=u32::MAX-1;
therefore both `(2*n-1)^2` and `(2*n)*(2*n+1)` fit u64. The eight boundary tests
compare with independently formed u128 products, including i32::MAX. asinh
negates the resulting owned BigInt term after division by the positive
denominator. Truncation toward zero commutes with this negation, including a
term that becomes zero. The existing error budget, stopping rule, lazy graph,
cache representation and operation domain remain unchanged.

This repairs the coefficient-width defect, not every possible extreme i32
precision/counter expression in Hyperreal. No arbitrary precision cap, API,
node variant, object field or dependency was introduced. A new whole-range
precision-arithmetic claim is explicitly out of scope for this checkpoint.

## Frozen evidence

- `series_limit.rs` remains byte-identical to the original probe; the before
  binaries/logs are untouched. Requestedbits+256 directed MPFR, exact dyadic
  input verification, and disjointness checks distinguish wrong answers from
  an insufficient oracle. `analyze-series-limit.mjs` validates that baseline.
- `coefficient-regression-before-debug.log`: four specific overflow failures.
  `coefficient-regression-before-release.log`: four non-enclosure assertions.
- Four new regression functions test both signs in each rational/generic
  path at192,000bits. Together with the coefficient boundary test these add
  five tests. The raw generic nodes ensure public rational dispatch cannot
  accidentally leave two of the repaired recurrences untested.
- `coefficient-{debug,release}-runs.json`: eight independent public probes per
  build, asin/asinh at184000,192000,256000,524288bits. All16pass, no60secondcap.
- `coefficient-hyperreal-debug-final.log`:746passes including19doctests.
  `coefficient-hyperreal-release-all-features-final.log`:853passes including
  24doctests. Both commands exit0.
- `coefficient-clippy.log`: allfeatures/alltargets-Dwarnings, exit0.
  `coefficient-fmt.log`: fmtcheck, exit0. `coefficient-wasm32-check.log`:
  wasm32-unknown-unknown --lib compile check, exit0. This is compilation
  coverage, not an executed32-bit numerical benchmark.
- `coefficient-memcheck.log`: all four high-precision regression functions,
  eight signed requests, releaseallfeatures,119.04s, zeroMemcheck errors,
  exit0. Leak checking was disabled; no zero-leak claim is made.
- Downstream --lib: Hyperlattice19, Hyperlimit242, Hypertri3, Hypersolve432,
  allpassed/exit0,696passes. Hypercurve full run is separately tracked below.

Initial benchmark harness compilation used a private shift helper; the
external harness was corrected to use public multiplication before freezing
or measuring a baseline. Its failure log is retained. Initial debug and
release full suites, and escalated retries, passed numerical tests but failed
to link doctests because /tmp reported Disk quota exceeded. Final complete
runs used an isolated workspace TMPDIR `.audit-coefficient-tmp.XBlc5e` and
passed. No shared temporary files were deleted to work around the quota.

## Performance, allocation and size

`coefficient_controls.rs`, before/after binaries,198fresh-process observations
and `analyze-coefficient-bench.mjs` are frozen. Nine alternating rounds,
eightpostwarmup paired samples per family, CPU6. Ordinary families check256
preconstructed inputs at32/128bits; high controls check one1/16 at184000bits.
Timed work is clone+function construction+approximation. Every output is
independently checked against outward MPFR outside timing (4096/184256bits).
41,508checked outputs across all198observations; all checksums match.

No speed ratio uses the incorrect192000bit baseline. Timings are mixed:

- asin-high paired before/after CPU ratio1.17321, descriptive bootstrap95%
  interval1.04452–1.23960; asinh-high1.01235, interval0.98525–1.11714.
- Ordinary paired medians range0.96817–1.02229. Some intervals, notably
  asinh-radical0.60050–0.98435, are broad. These are not a universal speedup
  or a proof of no ordinary-path regression; concurrent host work was not
  excluded. The complete per-family observations and intervals are retained.
- Both184000bit controls drop68995→46001allocation calls (33.33%) and
  1323590176→1058953032cumulative allocated bytes (19.99%). These are
  allocation traffic, not live/peak memory. Ordinary tiny asin/asinh and
  radical asin also allocate less; several unchanged paths allocate equally.
- The installed num-bigint0.4.6 source uses different division ownership:
  BigUint DivAssign<u32> clones a borrowed dividend, whereas DivAssign<u64>
  takes ownership via mem::replace. This supports retaining unsigned-wide
  division; exact measured counts, rather than a generic type-width slogan,
  establish this workload's memory-traffic change.
- Identical benchmark build flags: file2216984→2218048bytes(+1064);
  text1671619→1672595(+976),data218920unchanged,bss2848→1856(−992),
  loadedtotal1893387→1893371(−16). Layout changes are workload-specific.

Retention follows the user's priority: remove a reproducible exactness and
completeness failure, with a small implementation and binary cost and lower
measured allocation traffic. No general speed or cross-language claim.

## Artifact hashes

| Artifact under workspace root | SHA256 |
| --- | --- |
| exact-real-references/numbers-qualification/coefficient_controls.rs | 961561139b341c6b805e4e4e708ee8673a2c1f0fb93a4256b83f65368d427a64 |
| .audit-numbers.rjcbha/coefficient-controls-before | 921104ff0c82ec7a36c581b4fef13e1070d7bccf2e2d4b13e6ca574b2052cdfd |
| .audit-numbers.rjcbha/coefficient-controls-after | d61fd5051597fde9c1b457b671e1e19bcd9654a2ff433ffb8b3520cfff27136c |
| .audit-numbers.rjcbha/coefficient-series-after-debug | 6f3e856c756430c952247a2d051a0fad31e86bfe482c7d3a3dc4edb704fef26b |
| .audit-numbers.rjcbha/coefficient-series-after-release | 4524504b35969406dbee70f89bae8a4e88f74b3f6573e4f6dfd3ae6d51fb2847 |
| .audit-numbers.rjcbha/coefficient-retained-inverse-trig.rs | a4ee2646773a3cc1f7a934a7cf6dbf8cf6ef614e102123abb6d8595160f5bed1 |
| .audit-numbers.rjcbha/coefficient-retained-inverse-hyperbolic.rs | dd28f60617c20bbf0a35ba9d2261726f349e52035f30b7d011a3a26858c25e43 |
| .audit-numbers.rjcbha/coefficient-retained-tests.rs | bafee0865639f035a4b481ba6cc1570c004267b2118b809da1b2581e0655363a |

## Hypercurve checkpoint

The full current-worktree run started on HEAD149bea52c9c16797001fee2a95cf1dc739fe82e5
with user edits to bezier_offset.rs,bezier_region.rs,policy.rs. Those source
hashes are in `coefficient-hypercurve-launch.sha256`. It reported two failures:
`nonrepresented_center_nonrational_chamfer_inverts_with_retained_authority`
(cache-entry size assertion) and
`nonrepresented_center_transverse_chamfer_inverts_by_correlated_point`
(Uncertain(Predicate) versus Decided(Greater)). A focused after run reproduces
both. A separate six-crate source mirror `.audit-coefficient-control.UPsrn7`
preserves the same Hypercurve source and restores ONLY the coefficient repair
in its private Hyperreal copy to the verified prior hashes; its focused
before control finishedexit101, with the same two assertions, same source
lines and same Uncertain(Predicate)/Decided(Greater) values as after. Both
focused runs execute exactly those two tests,0pass/2fail. The pre-repair
inverse-trig and inverse-hyperbolic hashes match the original baseline exactly.
Thus these two failures are not introduced by the coefficient change. The
full run finished714.96s,898passed/2failed/1ignored,exit101. The launch hashes
matched repeatedly through the control-copy build and until shortly before
completion; a later concurrent edit to bezier_offset.rs changed its hash at
the final check. Qualification applies to the compiled/frozen snapshot, not
that subsequent edit. The user's actual crates have not been reverted or
repaired as part of this control.
