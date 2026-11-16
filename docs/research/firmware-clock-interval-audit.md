# Firmware clock interval ownership and publication

This Phase 0/9 audit covers the clock interval path at firmware commit
`2d68e82b7341db7b747b20e4ad1455a4c1113a00`, source `crates/alumina-clock/src/lib.rs`,
SHA-256 `343434a6947fcf91aef655cba8e36a4788d03d43b7dee9bc063ae1512e94fa50`. It updates two old lexical clock inventory rows
and adds eleven omitted state/report/operation declarations. This record originally left 50 firmware candidates unreviewed. The later
[FOC audit](firmware-foc-interval-audit.md) and
[remaining-entry audit](firmware-remaining-interval-audit.md) classify those entries. This is not completion
of the firmware audit, migration, HIL qualification or interval plan.

## Authority and ownership

`ClockEstimator` stores a boot-scoped affine outer envelope: `u128` scaled rate
bounds, `i128` scaled offset bounds, and exact integer timestamps/cycle counts.
It is a fixed-size `Copy` value with no allocation. `alumina-clock` is `no_std`
and enables Hyperinterval's integer feature; no arbitrary-precision backend is
needed. Model and prediction structs have no serde or native replay codec.
Canonical heartbeat inputs retain their checked wire representation. Rebuilding
the model from accepted observations establishes the envelope; a copied midpoint
or serialized status is not a substitute for that evidence.

| Declaration | Retained domain role / generic owner |
| --- | --- |
| `ClockEstimationPolicy` | Clock-specific drift, round-trip, freshness, sample-count and deadline constraints |
| `ClockEstimator` | Causal intersection, boot/sample identity, health/deadline evidence and publication |
| `ClockPrediction` | Future-start enclosure and admitted integer schedule; selected point retains its bounds |
| `ClockInstantEstimate` | Instant enclosure; midpoint is a display representative |
| `ClockUiEstimate` | Inverse-cycle enclosure with original cycle and boot/probe identity |
| `observe` | Validates heartbeat and sequence, constructs causal sample, delegates offset intersection to `IntegerRange`, publishes atomically |
| `predict` | Checks health/freshness and host/device lead/horizon; delegates integer midpoint/radius |
| `estimate_at` | Retains instant/freshness semantics without claiming future-start admission |
| `estimate_ui_for_cycle_interval` | Validates ordered cycle inputs and inverse uncertainty; delegates midpoint/radius |
| `cycle_bounds` | Uses `IntegerRange` scaling/addition and `integer_quotient_bounds`; retains units and checked narrowing |
| `earliest_ui_for_cycle` | Uses upper rate/offset and directed integer quotient; retains nonnegative browser-time domain |
| `latest_ui_for_cycle` | Uses lower rate/offset and directed integer quotient; retains checked output conversion |
| `scaled_offset` | Checked exact point arithmetic in clock units, not a generic interval implementation |

The declarations above and complete estimator implementation were reviewed.
Selected consumers were read at Interface `f5341f368`: client
`DeviceClockModel::accept_response`, prediction/inverse wrappers, and browser
clock qualification/lease-estimate consumption. The distributed simulator's
clock fixture and prediction consumer were reviewed. This does not claim an
exhaustive transitive call graph or a review of every firmware byte/storage
range matched by the original lexical scan.

## Reproduced failure and repair

At the accepted-sample counter limit, `observe` previously returned `Arithmetic`
after assigning the new sample's identity, timestamps and other model fields.
The regression reproduces this on `b9ecf8df5`: the failed observation changes
sample 2's metadata to sample 3 while the accepted counter stays at its maximum.
The client publishes its latest heartbeat only after `observe` succeeds, so
partial estimator publication violates the boundary between those owners.

The fix computes the prospective accepted count after existing validation and
intersection, before the first state assignment. It preserves the last legal
increment, earlier rejection ordering, existing typed error and diagnostic
rejection-counter semantics. It does not saturate the count or accept overflow.
The regression checks the whole estimator, published estimate, repeated failure,
earlier boot validation, and reset/reconstruction. It sets the private counter
to the boundary to avoid billions of otherwise ordinary exchanges; this is not
a full-duration endurance test.

## Reproduced qualification

Rust 1.88 passes all 602 native firmware tests/doctests (55 suites), strict
all-target Clippy and workspace formatting. The isolated Interface client passes
90 tests/doctests (3 suites) on Rust 1.95 and its WASM check. The clock crate builds
for WASM and Thumb with Rust 1.88, and Xtensa ESP32 with the installed ESP Rust
1.90 toolchain. Existing canonical heartbeat tests remain unchanged.

The existing clock-owner probe covers 24 frequency/offset/delay combinations,
including observation, prediction, instant and inverse queries. All 481 output/
wire/query/rejection records and zero-allocation measurements are identical to
the pre-fix baseline. Its release executable remains 423,712 bytes with 267,100
text bytes. The host `observe` symbol shrinks by 7 bytes. Eight CPU-0 ABBA blocks,
ASLR disabled, contain 960 timing samples / 5,898,240 completed probe operations.
Medians are 586.094 ns before and 584.945 ns after; this small difference supports
no observed median regression, not a claimed speedup or MCU-cycle result.

| Target | `observe` bytes before → after | Local frame before → after |
| --- | ---: | ---: |
| Thumb v7em hard-float | 1,862 → 1,814 | 408 → 408 bytes |
| Xtensa ESP32 | 3,193 → 3,159 | 464 → 464 bytes |

Target symbol sizes come from assembling compiler-emitted assembly, since the
Cargo LTO objects contain bitcode. Frame measurements cover this function's
own prologue only. They exclude callee/interrupt stack, final firmware linking,
flash fit, hardware cycles and HIL. Those broader release gates remain open;
no deployment or hardware safety claim follows from these local checks.
