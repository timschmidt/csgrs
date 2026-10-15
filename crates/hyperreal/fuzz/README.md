# Hyperreal fuzzing

These targets keep their generated inputs in bounded exact-rational form. They
exercise construction, arithmetic, structural/certified queries, fused linear
algebra, lazy elementary-function evaluation, serialization, and direct
`Computable` approximation without treating a primitive-float result as proof.

From `crates/hyperreal`, compile every target:

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
```

From the same directory, run a bounded smoke pass:

```sh
cargo +nightly fuzz run rational_arithmetic --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run real_exact --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run real_elementary --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run computable_approximation --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run structural_representations --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run string_parsing --fuzz-dir fuzz -- -max_total_time=30
```

Long-running campaigns should retain each target's corpus separately. A crash
is a semantic regression until minimized and promoted to a deterministic test.

`structural_representations` uses the same 22 private optimized `Real`
certificate representatives as the deterministic integration tests, including
`Pow10` and `Pow2`. They span all eight public `StructuralKind` values.
Replaying all 22 retained stride seeds covers the complete ordered 22x22
binary-dispatch matrix. Mutated bytes also build shared `Computable` DAGs with
up to 32 variable nodes. The deterministic serde inventory test covers every finite
`Computable` node and shared-constant tag, while this target covers the
unbounded graph-topology dimension.

The same target also checks rational normal-form and affine-determinant facts
before and after refinement. Warming may strengthen an unknown result, but
must preserve every available definite fact. Generated DAGs additionally
retain signs proved at a finer precision when queried again at zero refinement.

`string_parsing` checks that every accepted exact `Rational` literal is
also accepted by `Real`. Its seed corpus includes signed wide integers,
general fractions, long decimals, scientific notation, and invalid
zero-denominator input.

`computable_approximation` also builds mixed exact-rational multiples of pi
and e and requests signs at generated bounded floors. This exercises both
successful outward-binary64 interval certificates and conservative fallback
without accepting a primitive-float value as proof.

`real_elementary` reconstructs generated positive rational roots of degrees
three through nine and requires the independent root node and repeated-power
graph to receive an exact-zero certificate at a bounded floor. This exercises
direct root approximation, algebraic generator dependencies, and the
zero-separation fallback together.

## Retained performance offenders

Run `cargo bench --bench adversarial_library --features simple` to refresh the
deterministic fuzz timing history in `slow_performers.txt`. The run keeps each
case's worst-ever observation, rotates the worst eligible offender into the
100-case `promoted_slow_offenders.txt` lexicase set, and updates the score,
delta, and delta derivative in `benchmarks.md`. The promoted set is replayed as
dedicated Criterion rows by `adversarial_transcendentals`.
