# Exact fields and visible 3D metaball failures

This is a local qualification of the 3D metaball migration under
[the admission contract](../../admission.md). It does not close the full
properties plan or establish release readiness. The implementation is committed
separately; [closure-v1.json](closure-v1.json) records its revision, source and
artifact hashes, retained failures, executed checks and remaining limitations.

## Contract and ownership

Hypersdf now owns retained inverse-square sources and their strict scalar
validation, normalized field and conservative bounds. CSGRS composes that field
with its existing sampled-field adapter. Hypermesh owns the shared standalone
Surface Nets extraction. No alternate mesh carrier or caller-specific kernel
was added.

The domain requires positive radii and iso value, nonnegative padding and
requested grid dimensions of at least two. Empty input is successful only after
parameter validation. Invalid input, undecided predicates and resource
exhaustion remain distinct typed errors. Native scalar causes survive through
the public modeling error's source chain.

This deliberately migrates geometry as well as the primary Rust return types:

```text
I = sum(radius² / distance²)
F = (iso - I)/(iso + I), with continuous value -1 at centers
```

The continuous zero set is unchanged. Finite interpolation samples F rather
than the former truncated influence with a finite pole sentinel. The existing
conservative bounds and one-cell sampling border are retained; output
coordinates and serialized artifacts change. Diagnostics describe F, negative
inside. Authoritative extraction retains Real coordinates instead of lifting
f32 vertices. Numerical certainty is about the sampled proposal; closed
topology and fidelity to the continuous field are separate claims.

## Semantic evidence

Ten regressions fail on revision `5d49530ee` with the original pending native
changes. They cover hidden invalid input, empty shortcuts, unresolved radius,
resolution clamping, grid overflow, a distant cutoff, a finite pole sentinel
and inexact construction. Five native field tests independently check rational
values, scale invariance, coincident centers, conservative bounds and domain
failures. The former near-center input is preserved in the native test.

For a unit source at requested resolution three, the conservative half-width
is `a = 101/100`. A crossing lies at `(a²+1)/(2a)` on an axis edge. Averaging the
three edge crossings gives coordinate magnitudes `20201/60600`. Tests check
those exact values, the independent cube-volume formula, closed topology,
diagnostic counts, union from empty and subtraction back to empty.

The two-source benchmark fixture keeps its original requested 20³ grid; the
README fixture keeps all three original sources and its 8³ grid. Both complete
under strict policy with closed topology, positive exact signed volume and
strictly interior bounds. The existing 6³ two-source diagnostic fixture also
passes. These checks establish completion and specified invariants, not an
analytic-surface error bound or a performance comparison.

## Trials and final checks

| Trial | Result |
| --- | --- |
| `baseline-v1` | All ten regressions failed. Its initial protocol used `(iso-I)/(1+I)`; the final contract uses the scale-invariant `iso+I` denominator. The unit-iso oracle is unchanged. |
| `candidate-v1` | Cargo rejected the optional-feature configuration before compilation. The manifest now preserves the existing implicit `hypersdf` feature. |
| `candidate-v2` | Focused root and native field tests plus strict all-target Clippy passed. |
| `native-v1` | All 19 final commands passed, including both full-size workloads and final docs. |

The final run passes 225 root tests, 236 Hypermesh tests, 115 all-feature
Hypersdf tests, 114 default Hypersdf tests, 10 FFI tests and one doctest. Feature
checks additionally pass 38 minimal root tests, 18 SDF tests and 15 metaball
tests. Counts across configurations overlap. Eight existing cases remain
ignored: one rendering case and seven native external-fixture/manual scale or
memory cases. Formatting, strict Clippy for affected packages and feature
configurations, both fuzz manifests and warning-free documentation pass.
Commands, toolchain, environment, logs and source snapshots are retained per
trial. Source changes were prohibited during each trial.

The closure verifies prior and newly observed executable hashes and confirms
the original 35 pending native files are unchanged. Only `current_work` in the
properties ledger is updated. Disk reclamation removed unused debug seed
copies from release-only SDF trials and the unused seed of the manifest-failed
candidate. The audit records preserve their authority hashes; executed
artifacts, source archives and failures remain intact.

## Open boundaries

The separate 2D metaball field and contour migration remains open. Grid
capacity and primary reservation failures are typed, but scalar growth, total
work, heap bytes and cancellation have no internal bound. The commands used
external 1800-second wall containment; peak memory and allocations were not
measured. Fuzz targets were compiled, not run as a timed campaign.

No metaball API is currently exposed by the C/Go/Python/WASM bindings. FFI unit
tests passed; live binding consumers were not rerun for this Rust API change.
Generated images and export artifacts, sanitizers, the full CI matrix, clean
external distribution/MSRV validation, end-to-end provenance and continuous
fidelity bounds remain unqualified. No push or performance claim is made.
