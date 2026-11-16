# Firmware FOC and servo interval ownership

This Phase 0/9 inventory audit reviews the 27 original `alumina-foc` search
matches at firmware `2d68e82b7341db7b747b20e4ad1455a4c1113a00`. Fifteen are
production carriers/helpers, eleven are tests, a test oracle or a layout gate,
and one helper has already been removed. Seven omitted point/component carriers
are added. The other 23 original firmware candidates remain lexical/unreviewed.
This is a source ownership audit, not completion of firmware migration or the
interval plan. No production code, test, policy or fixture changes accompany it.

The missing acceptance witness was a body-reviewed disposition for these rows:
their old search labels did not establish which arithmetic was delegated, which
retained domain evidence, or even whether a symbol still existed. The relevant
contracts are P1/P3/P6 exact enclosures and honest failures, P19 preserved evidence,
and P31–P33 machine boundaries, together with plan sections 7/12 and Phase 9.
Current locators and full-file hashes are recorded in the audit evidence.

## Carriers and authority

`alumina-foc` is `no_std`; its normal dependencies are the portable protocol and
Hyperinterval with `fixed`. The reviewed production representations and operations
use fixed primitive storage and no heap allocation. This source observation does
not replace an allocation measurement or whole-firmware resource qualification.

| Carrier | Meaning and ownership |
| --- | --- |
| `Q30` | One exact `i32 / 2^30` lattice point. Point multiply/divide explicitly round to nearest, ties to even. |
| `Q30Interval` | Nonempty ordered closed real enclosure with Q30 endpoints. `FixedInterval<i32,30>` owns ratio construction, arithmetic, square and floor midpoint. Const construction, membership and width remain in the public wrapper. |
| `Phase3`, `AlphaBeta`, `DqInterval` | Component enclosures in phase, stationary and rotating coordinates. They retain coordinate meaning; the fields do not encode arbitrary cross-component correlations. |
| `Rotation` | Private sine, cosine and squared-norm intervals. Construction checks component range, containment of unit norm and the declared maximum norm error. |
| `Duty3`, `ModulationResult` | Duty enclosures and retained common-mode/centered-phase results. A freely constructed duty carrier alone does not establish PWM admissibility. |
| `ServoPosition` | One exact `i64 / 2^32` point in configuration-defined units. The type alone does not declare turns, radians or linear units. |
| `ServoPositionInterval` | Nonempty Q31.32 enclosure; Hyperinterval owns full-width midpoint and width. Const endpoint construction remains supported. |
| `ServoPositionErrorInterval` | Target-minus-observation endpoints widened to `i128`, still with denominator `2^32`. The maximum difference needs the full `u64` magnitude. |
| `ServoQ30ErrorInterval` | Target-minus-observation endpoints widened to `i64`, still with denominator `2^30`; subtraction delegates to `FixedInterval<i64,30>`. |
| `ServoPositionIntervalError` | Distinguishes invalid endpoint order from arithmetic failure; it is not a numerical carrier. |

The generic fixed carrier currently admits only `i32` and `i64` endpoint types.
Replacing the wide position-error carrier with either would lose supported
differences. Its remaining generic ownership work must preserve the width and
unit contract; a narrower interval is not an equivalent implementation.

The wrappers have no direct serde/native replay implementation. That does not
mean their values never cross a wire: the configuration codec embeds Q30 lower
and upper numerators, and alumina-config's `RECORD_KIND_FOC_CURRENT_CHANNEL`
decode arm constructs the gain through checked `Q30Interval::new`. This review covers that selected
decode ingress, not every configuration field or the complete codec. Copying
endpoint bits does not reconstruct calibration, phase or hardware evidence.

## Operations and remaining migration

| Path | Current arithmetic / retained domain work |
| --- | --- |
| `scale_wide_interval` | Clarke's wide source bounds use `FixedInterval<i64,30>` multiply, then checked conversion to `i32`; outward rounding precedes narrowing. |
| `interval_min`, `interval_max` | Manual endpointwise images remain. They are generic migration candidates, not completed delegation or mere domain wrappers. |
| `expand_unit_interval` | Widen by a nonnegative angle error, then intersect with the independently justified sine/cosine range `[-1,1]`. The analytic justification remains in FOC. |
| `widen_interval`, `interval_within_limit` | Current uncertainty expansion rejects negative error/overflow; admission checks the entire enclosure against a physical current limit. Generic expansion remains a candidate. |
| Current-channel mapping | Signed ADC-count difference scales both gain endpoints in widened integers, reorders for negative sign, adds error and checks narrowing/width/range. Generic signed scalar-image arithmetic remains manual. |
| Encoder position/velocity | Checked `i128` products and directed quotients map count uncertainty to configured position or normalized velocity. Quotient/scalar-image delegation remains open. Units, wrap selection, cadence and model-error policy remain with the encoder. |
| `clamp_i64_to_i32_bounds` | Explicit selected-point PI saturation and anti-windup behavior. It is not an interval intersection or fallback for an unknown enclosure. |
| `ratio_bounds` | Absent at this revision; removed by `953ed4b7e8059288abe5146cfd68400f0961587e` when Q30 arithmetic delegated to Hyperinterval. The retained inventory row has line zero to denote absence. |

Other inspected immediate arithmetic, including Clarke's alpha quotient,
phase-to-duty mapping and outer-loop position-to-velocity scaling, still uses
domain-local directed division. This audit does not mark those paths migrated.
Hyperinterval already provides `integer_quotient_bounds`; any replacement must
preserve the admitted input domain, error order, outputs and completed-workload
cost gates. No source inspection result here establishes a reachable overflow
bug in those private helpers.

Space-vector modulation computes the min/max common mode, centered phases and
duties, then rejects out-of-range results. It does not silently clip an invalid
modulation into success. Encoder observation prepares a copied next state and
commits it only on success; failure latches its diagnostic while retaining the
previous accepted count/sequence. Angle reconstruction retains reduction,
quadrant transport, polynomial remainder and unit-range evidence above the
generic Q30 arithmetic. These selected callers were reviewed, not the entire
FOC actor or every transitive hardware consumer.

## Acceptance evidence and limits

At the same firmware revision, the preceding clock qualification ran Rust 1.88
`cargo test --locked`: 602 firmware tests/doctests passed in 55 suites, including
86 FOC unit tests and two `q30_contract` integration tests. Strict all-target
Clippy and workspace formatting passed. This audit reuses those unchanged-source
results; it does not present them as a new benchmark or a newly executed test run.
The archived logs are in evidence commit
`c390ef8e50da1f274ce5b1217f83a3e0bf4bdcac`, directory
`docs/evidence/2026-11-15-interval-firmware-interval-inventory`.

Reviewed witnesses include independent integer certificates for constants;
negative, non-power-of-two encoder scaling; forward wrapping; failed encoder
range/precision admission before count/sequence advance; Q30 zero-divisor and
range failures; and PWM range/pulse/quantization rejection. The Q30 contract
tests feed a generated ratio through square, reconstruction, identity operations
and double negation, and check that intermediate overflow cannot be excused by
later cancellation. Servo grid tests independently verify full-width midpoint,
width and target-difference relations. Const API and layout assertions remain
unchanged. The test-only `atan_reciprocal_bounds` is an independent scaled-rational
oracle for its fixed tested arguments, not a general production transcendental
kernel. The actor size test is a layout gate, not interval arithmetic.

After the inventory/report edit, mechanical checks verify all 33 live declaration
locators, the removed symbol's absence, source hashes, the 27 updated and seven
added rows, and byte preservation of the other 1,498 original rows. They also
verify the reused test log's counts and its source revision. These checks validate
the inventory and evidence links; a finite corpus is not a proof of all possible
control-system behaviors.

No arithmetic refactor, performance improvement, allocation measurement, target
size/stack improvement or hardware safety result is claimed by this documentation
change. Further migrations require owner and consumer correctness/cost evidence.
Final firmware images, MCU cycles, callee/interrupt stack and HIL remain separate
release gates. No tests or policies were weakened to close an inventory item.
