# Coefficient ownership reconsidered under the bounded size exception

The combined coefficient-association ownership and lazy adaptive preparation
candidate remains **rejected**. Current-revision measurements expose increased
retained memory in an existing preparation workload. The size exception does
not waive that gate. No production source or cumulative size account changes.

Hyperdynamics owns stage lifetime and adaptive scheduling; Hypersolve retains
coefficient generation and validation authority. This Phase 13 investigation
covers P1/P4/P5/P6/P8–P10/P17–P20/P27/P36 on the existing finite prepared-order
expression-field domain. It revisits the historical size-only rejection, not
an unchanged extraction-only variant already rejected on runtime.

On baseline `184faefa1d4cc1353325b82dcd5d019d5b3da537`, the original ownership
witness fails: restricting a generated program copies its immutable coefficient
association. Candidate `6af61b53b1b5ab53b591448599d17f0d3b7a546c` applies the
archived combined patch without changing its original hard cases or assertions.
All 20 focused tests pass. Two observers match the current baseline's complete
373,388-byte coefficient/graph/report/work/dispatch artifact exactly. Pointer
identity is a storage witness; separate output and trace comparisons establish
the measured semantic agreement. This is finite strict-enclosure qualification,
not a full two-policy geometry or universal completeness claim.

## Memory result

Seventy-two fresh processes supply 288 matched pairs, or 576 observations, across
constant, exponential and Riccati fields. Each binary exercises preparation,
restriction, and their combined use at orders one and three. Half the processes
prewarm sixteen times; in the other half only the first recorded call is
unprimed. Each process records eight calls with full output materialization and
destruction.

| Existing workload | Order | Retained byte delta | Allocation count delta |
| --- | ---: | ---: | ---: |
| Preparation | 1 | -32 | +1 |
| Preparation | 3 | +16 | +3 |
| Restriction | 1 | -56 | 0 |
| Restriction | 3 | -168 | 0 |
| Preparation and restriction | 1 | -88 | +1 |
| Preparation and restriction | 3 | -152 | +3 |

Each delta is identical across all three fields and observations. Initial
counts and live bytes after destruction agree. Peak requested memory never
increases; order-three preparation reduces its peak by 64 bytes. Nevertheless,
all 48 ordinary order-three observations retain 16 additional bytes while the
prepared output is live. Restriction and peak-memory improvements do not waive
this existing-workload retained-memory regression.

These are requested Rust allocation bytes. Common source fields are excluded;
restriction-only also excludes its pre-existing higher-order program. Allocator
metadata, tracer headers, C allocations and RSS are excluded. No timing, total
heap bound or memory-leak claim follows from the measurements.

## Restoration and evidence

The isolated checkout is restored clean. Its original nine adaptive tests and
eight order tests pass, workspace formatting passes, and the rebuilt allocation
probe reproduces the baseline executable byte for byte. The rejected candidate
remains under `refs/interval-unaccepted/adaptive-stage-current-20261116`.

Further runtime, stack, lint, downstream, fuzz and WASM campaigns do not follow
this decisive rejection. No new numerical size allowance is adopted; all 54
accounts remain unchanged. A future design must address the retained-memory
cost before repeating qualification. The automatic noise controller, coordinated
CI and the complete fifteen-phase goal remain separate ongoing work.

Evidence record `docs/evidence/2026-11-16-interval-adaptive-stage-reconsideration`,
csgrs-evidence commit `29ca992059b924c21c5ecf6e63203e90fab33a14`: 125 files,
2,880,317 bytes, SHA-256
`d6882e5ab9b8738c8eb7e1b8210e52878a8f03c9f61139a7fe1344d1d372bda3`.
It retains the failed baseline witness, complete candidate patch, original
assertions, commands, raw allocation observations, exact comparisons and
restoration proof. Earlier archive records and manifest rows are unchanged.
No push or publication is claimed.

## Reassessment under the bounded memory exception

On `f402f67bd`, order-three preparation retains 16 extra requested bytes in
each of 48 cases, at most 0.211% of the corresponding live preparation.
This passes the proposed incremental limit of 32 bytes and 1%. Even carrying
forward the earlier rejected candidate's 16 bytes as a separate positive
charge gives 32 bytes, within the cumulative limit of 64 bytes and 2%.
The other 240 observations reduce retained bytes. Peak requested memory
decreases in every case and reaches at most 17,256 bytes. All post-drop
observations agree with their controls, including the existing warm state.
The extra three allocation calls during order-three preparation remain visible.

The original sharing assertion failed on the control. The revised checkout
passed 21 tests, and both candidate observers independently reproduced the
baseline observer's 373,388 bytes of coefficients, reports, work and dispatch
records. The final comparison uses 108 fresh processes and 864 observations
across the fixed program revision, primary and candidate. The original probe
excludes the input field and any existing higher-order program.

The linked `adaptive_taylor` benchmark still exceeds two existing size caps
on the target release configuration with `adaptive-taylor,taylor-evolution`.

| x86_64 release metric | Fixed program | Primary | Candidate | Increment | Cap |
| --- | ---: | ---: | ---: | ---: | ---: |
| Final file bytes | 3,466,240 | 3,466,232 | 3,470,528 | +4,296 | 4,096 |
| Defined function bytes, overlapping ranges counted once | 1,746,620 | 1,746,620 | 1,748,275 | +1,655 | 1,024 |

The size gate keeps `a0018ebe4` on `interval-coefficient-sharing-memory-review`.
This review admitted no memory or size charge for coefficient sharing. Full
stack, allocator-usable bytes and runtime qualification remain open. Raw
measurements, the proposed caps, build inputs, linked images and exact-output
comparisons are archived at `2026-11-17-interval-coefficient-memory-recheck`
in evidence commit `ed27541a2`. [Evidence index](../evidence-archive.tsv)
