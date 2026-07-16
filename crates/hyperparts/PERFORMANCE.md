# HyperParts performance and reference audit

This audit covers every source in the README reference section. Timings came
from the crate's optimized `queries` benchmark on 2026-07-15. They are
comparative local measurements, not portable latency guarantees.

## Retained results

| Path | Baseline median | Retained median | Result |
|---|---:|---:|---:|
| capability-constrained query over 256 families | 22.948 us | 20.849 us | 9.1% faster |
| representative EDA authoring intake | 11.461 us | 10.979 us | 4.2% faster |

`HasCapabilityTarget` is a graph-global fact. `query_parts` now compiles each
such constraint once before visiting families instead of allocating and
rescanning the capability collection for every family. The public
`has_capability_target` API also provides an allocation-free, short-circuiting
existence query when callers do not need the matching records.

The EDA intake pass now sizes its output and issue vectors from the input bundle
and no longer constructs a package `Pinout` vector merely to read its length
before dropping it. Exact values, evidence, handoff status, and output order are
unchanged.

An optional `dispatch-trace` regression imports reduced rational and decimal
circuit parameters. The trace records rational reductions and GCD work with
zero approximation and unknown-fact events.

## Reference mapping

### Yap, Towards Exact Geometric Computation

Numeric source strings continue to enter `hyperreal::Real` directly rather
than passing through a primitive float. Unknown, rejected, and lossy fields
remain separate states. The trace regression checks the exact-arithmetic
boundary, while property tests compare imported decimal values with direct
`Real` parsing. No approximate value is promoted to an exact part assertion.

### Nagel and Pederson, SPICE

SPICE combines circuit models with distinct DC, small-signal, and transient
analyses. HyperParts therefore retains topology, nets, model handles, and exact
parameters as `CircuitResidualFactHandoff` records but does not solve or stamp
them. `hypercircuit` remains the numerical owner. Adding a simulator or
interpreting generic EDA records as solved electrical behavior was outside this
crate's evidence boundary and was not attempted.

### tscircuit, Circuit JSON compilation

tscircuit's ordered render phases and typed Circuit JSON intermediate support
the existing pass-oriented import design: source records are normalized first,
then geometry, circuit, route, and fabrication facts leave through separate
typed handoffs. Known bundle cardinalities now size those pass outputs, and
graph-global query constraints are likewise compiled before candidate
evaluation. The crate intentionally consumes normalized records rather than
embedding a React/JS compiler or a general JSON schema implementation.

### KiCad S-expression format

KiCad specifies lowercase tokens, UTF-8 quoted strings, millimeter values,
relative coordinates, and library identifiers with explicit nickname and entry
components. HyperParts preserves imported handles, source text, units, and
provenance but does not claim that its compact footprint expression is a KiCad
parser. A raw S-expression reader belongs in an adapter that can report format
version and unsupported tokens before producing this crate's normalized bundle.

### LibrePCB developer and file-format documentation

LibrePCB separates symbols, components, packages, devices, and other library
elements, and version-tags directory-backed entities so migrations happen
before normal loading. HyperParts' typed package, terminal, aspect, and source
records match that normalized boundary. It does not duplicate LibrePCB's
migration machinery; adapters must preserve element identity and version in
their `SourceRef`/revision evidence.

### EU RoHS and REACH instruments

Both instruments are amended over time, so compliance cannot safely be a
timeless Boolean. `ComplianceClaim` already retains an `AssertionValue` and
source, and now also carries an optional `SourceRevision` or consolidation date.
Explicit unknown remains valid when a source was checked but did not establish
the claim. Substance thresholds, exemptions, candidate lists, and legal scope
were deliberately not hard-coded into this catalog crate; those require a
dated authoritative source and application-specific review.

## Considered but not retained

- Reserving every internal `PartVariant` vector from a coarse upper bound
  regressed the representative intake median from 11.127 us to 14.014 us
  (25.9%), so that change was fully removed.
- Secondary capability indexes would accelerate much larger graphs, but every
  mutation would then need synchronized index maintenance. The measured query
  needed only one graph-global prepass, so no duplicate index was added.
- Raw KiCad, LibrePCB, Circuit JSON, SPICE, RoHS, or REACH interpreters would
  duplicate domain/version owners and could silently invent semantics. The
  retained API remains a provenance-bearing normalized boundary.
