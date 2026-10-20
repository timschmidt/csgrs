# Jacobi epsilon consumed by geodesy

Baseline `5bd072da7`, branch `hyperspatial`. Owners: Hyperreal for the unwrapped real composition and exact Legendre
quarter endpoints; Hyperlattice for complex continuation; Hyperspatial for
development consumers. Properties P1–P7, P17–P20 and P35–P39 apply. The
[native contract](../../../hyperlattice/docs/admission-jacobi-epsilon.md)
defines the finite argument, real m<1 and pole domains. This is an E2
rectification prerequisite, not a completed ellipsoidal projection.

The missing consumer acceptance uses all 22 scalar families and their ordered
pairs at positive and negative parameters. All outputs remain native Real
expressions. Refine both complex epsilon components, then use its real square
plus two as an ellipsoid axis and its imaginary component as geodetic height.
The equatorial ECEF x coordinate must equal axis plus height exactly.
Consume that result in an anchored ENU/NED operation and its inverse; exact
zero local coordinates and the qualified input receipt must survive. Also
require F(inverse_E(real_epsilon(x|m)|m)|m)=x as an exact scalar certificate.
No numerical closeness assertion may replace these identity requirements.

Keep both 484-pair matrices and 176 family/regime cases, covering both signs,
zero, quarter-period translations, parity and display warming. The independent
native MPFR ODE oracle additionally verifies epsilon values across every scalar
family, with an explicit Taylor tail. Public tests cover true poles, finite
quarter periods, near-pole offsets, huge quasi-periods, tiny arguments,
serialization, cancellation/recovery, precision order and concurrency.

Extend the shared six-byte probe, per-family directed selection, sanitizer
target, every primitive-cache configuration, source consumer and workflow
benchmarks. Existing coverage denominators/floors remain independent; epsilon
and its shared quarter rule belong to the already measured Jacobi, complex
Jacobi and forward-Legendre files. ASan keeps the ten-second input and 2048 MiB
RSS limits. The 1800-second campaign must reach mutation after full seed replay.
These are external containment, not native time/work/heap bounds.

Record actual final-source checks and retained failures separately. No
Hyperspatial production API, wire format, observational accuracy claim or
Thompson/Lee inverse is supplied by this increment.

The initial spatial run fails its exact ECEF assertion for positive
`[0,0,2,2,0,0]`, negative `[0,0,2,2,1,0]` and mixed-family
`[0,7,127,39,0,0]` inputs. The represented values refine correctly, but the
general rational normalizer expands the complex axis before recognizing its
shared a²/a identity. Hyperreal's multiplication constructor owns this
P2/P18/P19/P20 gap. Retain a directly shared square quotient only after the
inverse operand has a nonzero certificate. Require identical owned node
identity for every repeated factor, so the retained value keeps the same
descendant graph and its cancellation signals. If removed wrappers carry
different abort signals, decline the rewrite. No inverse guard or proof
budget is relaxed, and no new approximation is requested by recognition.
The original corpus and exact ECEF assertions remain unchanged.

The shared square-quotient helper has its own 95% production line floor.
Private guard tests cover both operand orders and square representations,
positive and negative facts, unknown/zero divisors, independently owned equal
nodes, unrelated products and cancellation at each removed wrapper. Retained
values keep their entire shared descendant graph. New constructions can
serialize as the reduced operand; old wire tags and meanings are unchanged.
The initial three failing inputs also remain sanitizer seeds.

## Final verification

The [verification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-19-e2-jacobi-epsilon/README.md)
records passing stable/MSRV validation, six independent source consumers,
532 executions across four cache configurations and all nine independent
coverage floors. Every configuration retains all 1,144 epsilon workflows.
Both directed campaigns and both full ASan mutation campaigns pass; initial
seed-only/stopped runs and old/new slow artifacts remain diagnostic evidence.
The existing spatial production API and its 1,902-line coverage denominator
are unchanged. E2's ellipsoidal transverse projection and later stages remain
open; the increment supplies their native epsilon prerequisite.
