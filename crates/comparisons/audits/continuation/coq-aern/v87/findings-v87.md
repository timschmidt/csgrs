# Coq-aern metric and benchmark contracts — checkpoint 87

The six newly completed files add 2,864 unique lines. Cumulative coq-aern
coverage is 36 complete text files / 11,984 lines; 52 inventoried text files
and the separately classified artifacts remain open. Read ranges and original
hashes are in [read-records-v87.json](read-records-v87.json). These are source
reads, not a successful Coq build or a whole-reference completion statement.

## Exactness and completeness

RealMetric.v constructs continuous absolute value using overlapping choices
and a unique limit. Selecting either sign near zero is safe only because the
error is bounded, with two bits of slack. Later classical sign case analysis
proves metric laws; it does not turn undecidable exact ordering into an API.
The completed file proves nonnegativity, zero/equality, symmetry, triangle,
scaling, convexity and strict/non-strict interval-distance equivalences.

Euclidean.v uses dimension-indexed linked vectors and the **maximum-coordinate
metric**, not Euclidean length. Coordinatewise errors of at most epsilon give
maximum-distance error at most epsilon without a dimension factor. The proof
requires the coordinates of one common sequence. Unique limits, coherent
multivalued paths, closed target predicates and the optional state invariant
remain essential. Erasing dimensions does not authorize arbitrary vector shapes.

Hyperlattice's public norm is explicitly Euclidean: `sqrt(self_dot)`, with a
dedicated structural self-dot path and existing deferred rational reduction.
For a d-vector, maximum norm <= Euclidean norm <= sqrt(d) times maximum norm;
the two cannot be substituted under the current API. Checked normalization
rejects UnknownZero instead of treating unresolved coordinates as zero.
No new max-norm API or representation replacement is justified here. A future
coordinate-enclosure consumer could use a separately specified max metric, but
would need an actual use case and native correctness/performance qualification.

### Hand-written benchmark sqrt has a different, insufficient threshold

The formal real sqrt (Sqrt.v:338) and extracted Sqrt.hs:637 split using
`2^(-2n-1)`, permitting zero only when `x < 2^(-2n)`. This correctly bounds
the root error by `2^-n`. In contrast, bench.hs:95-101 uses `eps=2^-n` for
both split arguments, permitting zero whenever `x < 2*eps`.

An exact rational counterexample to that branch's declared approximation bound:

- n=4, eps=1/16, x=1/64, exact sqrt(x)=1/8.
- A valid initial enclosure [-1/64, 3/64] cannot certify x>0 but can certify
  x<2*eps. Selection can therefore choose the zero branch.
- The resulting claimed enclosure [-1/16, 1/16] misses 1/8.

The separately pinned AERN2 source chooses the right branch when it is certainly
true and the left is not yet certainly true. Its Integer-indexed limit adds
only `2^-p` to the selected approximation. A model of these contracts demonstrates
the missing guarantee; this checkpoint does **not** execute the Haskell helper,
prove that a particular default input stream takes that branch, or claim that
the separately pinned AERN2 checkout exactly equals the benchmark's Stack graph.
This defect is in the hand-written benchmark helper, not the formal/extracted
threshold. The existing two admitted complex-root lemmas are a separate issue.

## Benchmark reproducibility

The complete runner enables `csqrt3ExtractedOnly` and `csqrt5ExtractedOnly`,
which request `csqrt3rE` and `csqrt5rE`. Both dispatcher entries are commented
out in the tracked bench.hs. There are 17 active dispatch names; neither requested
complex name is among them. The active Haskell fallback reports an unrecognized
name. The static checker also runs `bash -n`, but does not execute the runner.

If all invocations succeeded, those enabled loops would plan 100 process runs:
two cases, five precisions, ten repetitions. This is a static plan, not completed
benchmark work. The runner overwrites repeated-parameter logs, deletes the failed
log on a nonzero exit, replaces CPU time text `0.00` with `0.01`, and leaves its
AccuracyTarget column blank. It extracts reported accuracy without independently
asserting the result or minimum accuracy. The manifest enables threaded RTS `-N`.
These choices do not support a fresh matched performance conclusion. A later
native run must preserve failures and all raw samples, control threading, use
equivalent output/error contracts and independently qualify each result.

## Executed qualification

[model-v87.mjs](model-v87.mjs) uses exact BigInt arithmetic, not floating-point
agreement or the donor's arithmetic engine. It checks 35,937 scalar triples,
71,874 strict/non-strict interval equivalences, 2,176 positive-weight convex
cases, 512 vector cases in dimensions 0/1/2/3/4/16/64/256, and 8,448 finite
fast-Cauchy pairs. The hand-zero bound has 510 counterexamples, n=3..512;
16,320 corresponding formal-zero-threshold cases satisfy their bounds.
Eight assertion-based corruption controls include missing absolute value,
dimension mismatch, norm substitution, a wrong signed scaling claim, invalid
integer-division oracle arithmetic and mixed complex-root coordinates. The
model's 55,713 trace records hash to
`35da1b6cb854c77c221298576f498b8fcfd98d3171ea21ae216be6f582f5ca34`.
These finite tests do not prove the Coq scripts or arbitrary infinite sequences.

Offline/locked Cargo runs qualify both selected Hyperlattice integration targets:
six api_surface_coverage tests and 21 vector tests in **each** debug and release.
All 27 names match across profiles, with no failed, ignored, filtered or measured
tests. They include Euclidean 3-4-5 norm/distance and checked-zero/Unknown controls.
The shared target is reused; Cargo recompiles Hyperreal/Hyperlattice and the four
integration executables, not a fresh whole-stack target. Executable paths, hashes
and sizes are bound by the checkpoint verifier. No new all-feature/full-stack,
WASM, Memcheck, timing, allocation or application-size improvement is claimed.

The historical GHC 9.6.7 path used for earlier Haskell qualification is now absent.
PATH checks find neither GHC/runghc nor Coq/Rocq/opam/Cabal; Stack remains present.
This is a check of PATH and named locations, not a proof that no compiler exists
anywhere on disk. No toolchain was downloaded or installed. Native donor builds,
proof checking and regenerated extraction remain pending. At the recorded
post-test capacity check, /tmp has 13,861,122,048 bytes available.

Three denied-subprocess attempts (earlier verifier, environment, shell syntax)
and one code-zero but empty model capture are preserved. Approved reruns succeed.
Empty output is not treated as successful numeric evidence. There is no donor
edit, Hyper production edit, new retained transfer, source-tree copy, broad
cleanup/deletion, external report, commit or push. The seven previously retained
continuation changes and the original full audit scope remain unchanged.
