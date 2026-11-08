# Selected graph sections with common supports — 8 November 2026

This extension starts at published `dad0c06ea8828a02a1e101a7f8dfb1494f36c9f9`.
Hypersolve owns retained-field elimination and exact selected-field signs;
Hypercurve owns source-branch selection, original guards, active domains and
finite point preimages. The affected contracts are P4, P8, P10, P12, P14,
P17–P20 and P35–P39. The algebraic closure target remains unchanged.

The original five-case nonlinear `(D,D^2)/1` probe completes four cases on
the parent. Its fifth case selects the positive square-root branch of
`F=(y^2-x)(y-2)`, with `x in [0,1]`, and queries `(0,0)` for `D=y-2`.
There are no preimages on that branch, but the unselected `y=2` component
makes both ordinary resultants vanish. The parent returns `Unsupported`.
The original inputs and complete-success assertion are retained. The candidate
completes all five cases, including the exact empty fifth result.

After a proved zero ordinary resultant, the shared resultant engine computes
the first nonzero coefficient in formal `z` of `Res_y(F,P+z)`, where `P` is
the pulled coordinate-level or chord equation. It lifts the existing polynomial
arithmetic and pivot-free determinants to `K[z]` inside the ordered field
`K(z)`. Coefficient signs at positive infinity use their leading nonzero
coefficient. No specialization, interpolation, primitive-element conversion
or independent coordinate materialization is introduced. All original-field
arithmetic and sign calls share one allowance, including the final scan.

For the generic first-variable fiber, write `F=C*Q`, assigning to `C` all
factors supported on `P`, with their multiplicity in `F`. For positive
`m=deg_y(P)`, multiplicativity gives

```text
Res_y(F,P+z) = lc_y(C)^m * z^deg_y(C) * Res_y(Q,P+z).
A(x) = first nonzero z coefficient = lc_y(C)^m * Res_y(Q,P).
```

The existing constant-resultant identity covers `m=0`. Every finite isolated
contact on the residual support projects to a zero of `A`, including boundary
fibers and degree drops. Extra leading-coefficient roots remain candidates.
A zero formal resultant is explicit, rather than an empty intersection.

Hypercurve chooses an exact interior abscissa outside every root of `A`.
At that fiber, a zero of `P` must belong to a common support. The original
checked simple-root connected slab prevents the selected graph from exchanging
factors in its interior. This supplies branch selection; an arbitrary point
sample alone would not. A continuous selected section still requires a domain
correspondence and remains an explicit blocker. Otherwise all candidates are
checked against the original selected branch, trim endpoints, query equation
and prospective denominator before returning finite locations. Retained maps,
root/fiber selections and original endpoint objects remain authoritative.

Point-location consumers first execute their original ordinary sections and
constant-image proof/census. Only a remaining `Unsupported` enables the new
common-component completion. Private request-local state transports an already
proved zero ordinary projection for the same immutable source/query/map; it
never survives a request, mutation or native replay. The maintained original
census asserts exactly one `reuse-zero-projection` event under each policy.
Direct chord/unique-section consumers retain component completion. The shared
structural degree authority also declines isolated-section work for a source
and homogeneous image independent of the source abscissa.

The independent quintic endpoint witness also exposed a lower-layer gap.
An exact selected-field sign suppresses the approximate terminal, but retaining
the caller's Approximate-512 selection still stops native tensor replay before
its complete two-root/image proof. A direct owning-layer regression reproduces
`Uncertain(Predicate)` on published main. After that existing whole-value
attempt declines, an exact pass outside a bounded speculative scope now
completes through the strict counterpart. Bounded speculative passes keep their decline; shared
resource scopes are retained. The original sign-attempt body and predicate order
are unchanged. No native field kind or Hyperlimit decision cascade is merged.

| Maintained witness | Independent expectation and continuation |
| --- | --- |
| Original five-case census | Exact parameters `1/2`, `1/4`, `{0,1}`, the selected unit root of `t^2-3t+1`, and an empty fifth locus |
| Shared unselected support with isolated contacts | Interior, endpoint and double contacts; exact reversal and original denominator exclusions |
| Independent quadratic query `(a,a^2)`, `2a^2=1` | Endpoint `x=1/2`, retained nonlinear image evaluation/query, and an empty reversed trim |
| Independent quintic query `(a,a^2)` | Exact selected endpoint `x=a^2`, source point evaluation and another retained-image query without a scalar coordinate view |
| Resource and policy transitions | Actual zero allowance and cancellation fail typed; a larger retry proves empty; a global zero denominator remains invalid |
| Selected continuous support | Explicit `Unsupported`, kept separate from completed finite and empty queries |
| Shear, restriction, reversal and native replay | A fresh process proves `X-Y=1/2`, `Y^2=1/2`, consumes the generated point and recovers the original source parameter |

The quintic is `g(a)=-1+2(a+a^2+a^3+a^4+a^5)=0`. Its reciprocal is
Eisenstein at two, and its derivative is positive on `[0,1]`. Writing
`g(a)=E(a^2)+a*O(a^2)` gives the independent endpoint polynomial
`h(t)=1-8t-8t^2-4t^3-4t^4-4t^5`. It is strictly decreasing on `[0,1]`.
The positive `O(t)=2+2t+2t^2` also gives `a=-E(t)/O(t)`, so the endpoint
retains field degree five. This degree excludes a tower of quadratic extensions;
it alone makes no claim about solvability by arbitrary radicals. The direct
scalar regression proves `a^2-t=0`
for independently selected roots and distinguishes nearby positive/negative
perturbations. Both policy chains consume no approximate terminal.

The focused final source has 3,812 tracked/intended inputs. All 10 ordered-field
resultant tests, 14 selected-field tests and 27 curve location tests pass, along
with four-crate all-feature/all-target Clippy with warnings denied. Rust 1.95
passes the affected tests, strict linting and documentation. Production WASM
feature combinations pass on Rust 1.99.1 and 1.95.0. The full owner run reports
1,375 Hypersolve tests in 46 harnesses and 3,193 Hypercurve tests in 122 harnesses,
with 37 existing Hypercurve ignores. The downstream BRep/IO run reports 996 successful tests,
no ignores and 110 harnesses. These are test reports, not distinct workflows.

The seven new reviewed capability rows pass separately in release processes.
They preserve all 423 previous rows and include an explicit continuous-section
frontier; a passing blocker assertion is not geometric completion. The full
catalog is rerun by the final pre-push contract.

The actual WASM probe executes all eleven pure maintained functions in three
fresh instances, using the default stack and zero host imports. One function
asserts the explicit continuous-section blocker. The native process-spawning
replay witnesses run separately. The final 12,141,146-byte module has SHA-256
`0724b25b7564d4067c185cc352d68621d246c72937d33e3becce2a98200b70bd`.
Its largest observed linear memory is 20,250,624 bytes, increased from the
rejected eager-completion prototype's 5,505,024 bytes. Linear memory is not
live heap or public ABI qualification; the adverse increase is retained.

Fourteen unchanged complete acceptance functions have identical nonempty full
dispatch vectors under both policies: 28 comparisons and 10,870,554 baseline
kernel events. Five further original constant-image functions also have identical
nonempty vectors, totaling 10,260,437 baseline events. These five functions
retain their authored policy coverage: two use the default policy, while three
include explicit policy transitions. Allocation calls/requested bytes match in
both cohorts, and
live-request peaks match for the five-function cohort. Three alternating fresh
processes give descriptive median time ratios 0.8381–1.1201 and 0.9564–1.0079,
respectively. Concurrent host activity prevents a statistical speedup or
nonregression claim. The fourteen-function normal executable grows 54,664 bytes;
its trace build grows 57,448 bytes. This is an exact-completion extension.

**Historical trace correction:** `reset()` clears counts but does not enable
recording. The earlier fourteen-function reports, including archive record 439,
contained 28 empty vectors. Their equality did not establish dispatch invariance.
The new comparison explicitly uses `recording_scope()` and requires nonempty
vectors. Original raw reports remain immutable. Their independent functional,
allocation and timing observations remain execution evidence, with their stated
limits; the empty-vector assertion is withdrawn.

The first prototype added 57,479 allocations and 4,887,196 cumulative requested
bytes to an existing constant-image case, with roughly 9% observed overhead.
A structural gate removed that difference, but the extended five-function
controls then exposed 1.80–2.81 times execution time and 468,434–3,078,320 extra
allocations. Those eager component-completion prototypes are rejected. Deferring
the fallback behind the original successful proof restores all observed
allocation counts and dispatch vectors. The final production bytes match the
cost-build freeze; only a test recording guard and a mathematical comment changed
afterward. The raw
manifests establish that equivalence. Partial deliberately canceled broad runs
and the missing-recording assertion failure are retained without completion credit.

Three fresh native processes also measure every pure acceptance function with
an allocation counter. The maximum observed live requested size is 27,546,508
bytes, excluding allocator overhead. The quintic chain's median is 7.583 seconds,
95,258,573 allocation requests and 9,770,201,744 cumulative requested bytes,
with an 11,433,996-byte live-request peak. The rejected eager prototype used
2.505 seconds, 24,122,322 requests, 3,876,377,972 cumulative requested bytes and
a 1,944,836-byte live-request peak. Preserving the original dispatch schedule
has substantial adverse cost on this new chain. This remains an optimization
target; cumulative bytes are not retained memory. The incomplete parent is not
a same-completed-contract timing comparison.

The default perturbation allowance is one million original coefficient-context
calls and 262,144 Sylvester problem-size units. Neither bounds one field
operation, total arithmetic, coefficient/system growth, heap or elapsed time.
Exact-limit/one-less retries and cancellation during the final coefficient scan
are tested. External process containment is not typed geometric exhaustion.
An early prototype could publish after final-scan cancellation; its failing
regression and the corrected final publication check are retained.

General selected continuous-component correspondences, selected-component
constancy beyond the demonstrated paths, algebraic guard coefficient admission,
arbitrary implicit filled operations and complete resource bounds remain open.
No new support is fitted or demoted, and no filled-region authority is added.
Full pre-push verification remains required for the final committed source.

The published-parent qualification is separately retained in local archive
record 441, commit `947fbd7a35ef5b2b387e8d3325151778553b5115`, with all 280
commands, original fuzz corpora, failures/cancellations and the actual main push
receipt. It qualifies the parent, not this extension. The original pending
evidence payload remains unchanged. The new scoped evidence is record 442,
`docs/evidence/2026-11-08-implicit-shared-sections`, local commit
`53e6b3f32b5d36911d9264b5c7c725f5091ae621`, with SHA-256
`bb42263a24201da787b2e8c9b8a3e1bbd7f6effe08741b115ecb18cfc13a1c2d`.
Separate evidence publication still needs explicit authorization. See the
[archive index](../evidence-archive.md).
