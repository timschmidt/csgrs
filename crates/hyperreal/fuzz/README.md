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
100-case `promoted_slow_offenders.txt` group, and updates the score,
delta, and delta derivative in `benchmarks.md`. The promoted set is replayed as
dedicated Criterion rows by `adversarial_transcendentals`.

The promotion ranks cases by recorded elapsed time. P38 still requires
case-by-case lexicase selection and independent populations for each
representation type; the current timing group does not satisfy those criteria.

## Directed scalar search

The `directed_scalar_probe` example and `directed/campaign.py` implement a
separate adaptive search over all 22 private scalar representations. Each
representation owns its population and random stream. Selection shuffles the
query cases, retains the slowest current candidates on each successive case,
and breaks remaining ties randomly. Selected parents produce byte mutations
for the next generation; every filtering and mutation step can be replayed.

From `crates/hyperreal`, build and run into a new output directory:

```sh
cargo build --release --example directed_scalar_probe --features serde,dispatch-trace --target-dir /tmp/hyperreal-directed-build
python3 fuzz/directed/campaign.py run --binary /tmp/hyperreal-directed-build/release/examples/directed_scalar_probe --output /tmp/hyperreal-directed-run --seed 20261014 --rounds 3 --population 8 --parents 4 --timeout 30
python3 fuzz/directed/campaign.py verify /tmp/hyperreal-directed-run
```

Each input varies the parameters inside its representation, including exact
rational widths, exponents, prime roots and logarithms, rational angles, and
decimal rational offsets near pi. A probe checks the actual private class,
retains three timing samples and raw dispatch paths per query, checks numeric
answers against directed MPFR intervals, and checks cold/warm proof retention.
Timing fitness guides exploration; it is not a competitive benchmark result.
Each sample rebuilds its query object; shared constants may already be warm
within that input's process. These are not claims that every cache is empty.

Near-pi cancellation belongs to the generic `Irrational` population, alongside
non-special sine inputs. The full one-to-78-digit decimal-offset range is
retained. `ConstOffset` separately varies subtraction from pi or e within the
constructor's certified positive range, and positive offsets from products
containing exponentials and positive or reciprocal powers of pi. The first
generator's incorrect `ConstOffset` expectation for `pi - 31/10` is retained
as an exact regression fixture.

Numeric response fields use explicit signed numerators and positive denominators
(`-7/3`), independently of the human-readable mixed-number `Rational` display.
The complete input and response that exposed the first wire-format mismatch
remain regression fixtures; the qualification also checks a fresh Rust response
through the Python exact-fraction validator before starting a campaign.

Campaign outputs include every generated input, process result, raw response,
population, selection trace, and mutation lineage. A failure or timeout closes
the run with its evidence intact. Completed campaigns can be verified from
their retained scores and exact rational endpoints. Existing outputs are never
overwritten or resumed.

P38 still requires sanitizer qualification and competitive benchmarks for the
retained cases, as well as directed populations in the other stack layers.
The bounded scalar campaign is not an exhaustive-input or cross-stack claim.

The `directed_scalar_bench` example and `directed/competitive.py` compare every
final selected input with native directed MPFR interval evaluation. Both must
produce an exact-rational enclosure with absolute width at most `2^-256`.
Both follow the recorded 256-to-2048-bit precision schedule, stopping when an
exact width check succeeds. All intermediate enclosures, all samples, and the
seeded execution order are retained. Each measured operation runs in a new
process; numeric oracle checks and text encoding occur after its timer.

```sh
cargo build --release --example directed_scalar_bench --features serde --target-dir /tmp/hyperreal-directed-native
python3 fuzz/directed/competitive.py run --binary /tmp/hyperreal-directed-native/release/examples/directed_scalar_bench --campaign /tmp/hyperreal-directed-run --output /tmp/hyperreal-directed-comparison --seed 20261014 --samples 11 --timeout 30
python3 fuzz/directed/competitive.py verify /tmp/hyperreal-directed-comparison
```

The comparison requires Linux CPU affinity support, a release build without
`dispatch-trace`, and a completed campaign from the same search implementation.
Qualify the adapters with sanitizers before collecting native timings. The
native benchmark build must exclude sanitizers. These results concern the
enclosure operation on the retained scalar inputs; they do not rank whole
libraries or compare symbolic certificate APIs or warm-result performance.
