# Bounded parameter replay and spatial consumers

Increment, 2026-10-19, based on `2aaf10dea`. Hyperreal owns the
principal-root domain and exact algebraic identity proofs. Hyperspatial owns
the downstream coordinate semantics and provenance assertions. This admission
covers P1–P6, P18–P20 and P35–P38; it does not admit a Thompson/Lee inverse,
Transverse Mercator model, or a new spatial public API.

## Domain and missing acceptance

For 0 <= m < 1, encode r = sqrt(m/(1-m)) and recover h = 1/(1+r*r) = 1-m.
For -1 < v < 1, encode a = v/sqrt(1-v*v) and recover v = a/sqrt(1+a*a).
For -1 <= u <= 1, encode tau = asin(u) and recover sin(tau) = u. Principal
roots select the nonnegative branch. Finite arbitrary r/a fields give legal
open-interval parameters even without trusting serialized domain assertions.
These identities are useful prerequisites for bounded analytic chart payloads;
a scalar identity alone does not certify a chart's continuation or inversion.

Before the scalar change, the unchanged 22-family acceptance fails: only one
family retains the parameter certificate, and only the two rational signed
meridian cases pass. An initial root-atom change resolves fresh construction
but loses most certificates on cold serialization replay. The native admission
at [Hyperreal](../../../hyperreal/docs/admission-bounded-parameter-encodings.md)
records the failing variants, domain proof, limits and recovery tests.

The existing native strategy runs first. Its optional root-completion attempt
uses the same remaining operation/node budget, existing atom/term/degree and
coefficient caps, and cancellation controller. It can prove root signs from
exact syntax, admitted quotients and the shared constants' fixed rational
bounds. Unknown domains remain unknown; no numeric closeness or serialized
trust flag supplies a certificate. A completed proof may be reused by a later
operation; it does not rewrite the authoritative expression or add a wire tag.

## Required downstream acceptance

The shared `dev/parameter_encodings.rs` workflow chooses parameter and meridian
values independently from the existing 22-family inventory, verifies their
public representation kinds, encodes and serializes six exact Real fields,
and replays all three identities cold or after explicit display warming. It
also covers m=0, v=0, closed time endpoints, both meridian signs, and exact
decimal offsets from parameter/meridian boundaries. The byte generator limits
these offsets to 1e-1 through 1e-256; this bound is a workload domain, not a
floating tolerance. All proof assertions require exact rational zero and a
subsequent sin consumer must retain that zero.

A replayed complement h supplies the ellipsoid's first axis 2+h; the replayed
meridian value supplies equatorial height. The same workflow constructs ECEF,
converts to an anchored ENU or NED frame and returns to ECEF. Every coordinate
identity remains exact, and each receipt must retain the qualified source
provenance without upgrading it to unqualified input accuracy. The two test
matrices contain 968 pair/sign/warming workflows and 528 boundary/regime/time
workflows. Both pass in the isolated external consumer probe and the final
stable/MSRV, coverage and four-cache-configuration runs recorded below.

## Testing posture and limits

- Native acceptance covers all 22 scalar families, both signs, exact near
  endpoints, JSON/CBOR replay, concurrent readers and precision order. Private
  guards cover opposite branches, nonidentities, unknown domains, zero inverse
  rejection, sign recovery, coefficient/degree limits, work exhaustion and
  cancellation. Query-local facts must not populate numerical root caches.
- Existing downstream complex-Jacobi and epsilon matrices remain unchanged.
  The earlier variant that preempted their proof strategy failed two Jacobi
  norm assertions; its complete failure and source archive are retained.
- Stable/MSRV, all four primitive-cache configurations, independent README
  consumers, native/WASM check and strict Clippy remain required. Existing
  production coverage denominators are preserved; the new domain helper and
  complete changed shared-ring functions each get an independent 95% floor.
- The same spatial workflow supplies the directed family buckets and ASan
  target. Actual post-initialization mutations, complete inputs and artifacts
  must be retained. Wall/RSS containment does not establish native heap bounds.
- The native benchmark records completed exact certificates after cold payload
  replay and after retained-certificate reuse. These are distinct workloads,
  not an external-library speed comparison. Both require materialized exact
  outputs; timing alone is insufficient acceptance.

The preliminary focused domain measurement was 133/157 executable lines
(84.71%), below the new floor. Keep that result and add the missing cold
arithmetic, interval coefficient-growth and zero-domain obligations. Do not
lower the floor or exclude those production paths.

The completed candidate adds those obligations and measures 156/157 domain
lines (99.36%) and 101/103 shared-ring function lines (98.06%). All fourteen
independent floors pass. Stable and Rust 1.95.0 each pass 206 spatial executions
in 51 suites, eight README consumers, strict Clippy and benchmarks. All four
primitive-cache configurations pass 640 executions in 128 suites. Native
documentation and WASM check/Clippy pass. The directed campaign completes 176
cases; ASan performs actual mutations after initialization and finishes 11,268
runs without new failure artifacts. The native benchmark accepts all 2,200
evaluations in 220 rows on both compilers. The
[verification record](../../verification/2026-10-19-e2-parameter-encodings/README.md)
retains source hashes, command exits, coverage diagnostics, earlier failures,
resource measurements and target integration checks.

No scalar wire tag, public spatial API or external runtime dependency is added.
Full Earth/sky development, native resource bounds and published distribution
remain open; these tests do not establish physical model accuracy.
