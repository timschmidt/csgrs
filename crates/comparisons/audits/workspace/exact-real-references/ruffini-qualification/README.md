# Ruffini scalar/shared checkpoint — partial, 2026-09-06

Donor [Ruffini](https://github.com/jonas-lj/Ruffini/tree/82d552fee22d92e493936183fab8672517694e56)
is unchanged at 82d552fee22d92e493936183fab8672517694e56. At this scalar/shared
checkpoint, full-read credit was 76/245 files and 3,810/26,234 physical lines.
The later [matrix checkpoint](MATRIX_CHECKPOINT.md) records additional reading
and qualification; the live inventory TSV is authoritative. The remaining source is
still in scope. `inventory.mjs` verifies every Git blob and SHA256, but credits
only explicitly read paths. `../RUFFINI_FILE_NOTES.md` records file-level ideas.
The subsequent [polynomial/class-group checkpoint](POLYNOMIAL_CHECKPOINT.md)
records181/245fully read files, native dependent-contract tests and the
cost/density-sensitive polynomial Karatsuba benchmark candidate. It does not
represent a retained Hyper production change or completion of the whole audit.

Later checkpoints: the [square-root caller pilot](square-root-pilot/README.md)
was tested and not retained. The bounded finite-field follow-up is recorded in
`finite-boundaries-analysis.json` and `finite-diagnostics-analysis.json`, with
file-level findings in `../RUFFINI_FILE_NOTES.md`. Current coverage is235/245
files and17,768/26,234physical lines; ten unread resources remain in scope.
Revalidate that numerical/diagnostic checkpoint with
`node exact-real-references/ruffini-qualification/analyze-finite-diagnostics.mjs`.
These local arithmetic checks do not qualify cryptography, Maven, performance,
or APIs that Hyper does not expose; no production change follows from them.

## Native provenance and execution

Private build directory: `.audit-ruffini-build.LmZgYM` in the workspace root.
Unchanged 96 common/reals main Java files compile directly using javac21.0.11,
`--release 16`, UTF-8, then run on OpenJDK25.0.4. Maven is absent from PATH;
this is **not** successful qualification of the original Maven lifecycle or
Java16 runtime. No signing, publishing, deployment, or donor edits occurred.

Dependencies were downloaded from Maven Central at the parent POM versions:

- [Guava32.0.0-jre](https://repo.maven.apache.org/maven2/com/google/guava/guava/32.0.0-jre/guava-32.0.0-jre.jar)
  SHA256 `39f3550b0343d8d19dd4e83bd165b58ea3389d2ddb9f2148e63903f79ecdb114`.
- [CommonsMath3.6.1](https://repo.maven.apache.org/maven2/org/apache/commons/commons-math3/3.6.1/commons-math3-3.6.1.jar)
  SHA256 `1e56d7b058d28b65abd256b8458e3885b674c1d588fa43cd7d1cbb9c7ef2b308`.
- [JUnit4.13.1](https://repo.maven.apache.org/maven2/junit/junit/4.13.1/junit-4.13.1.jar)
  SHA256 `c30719db974d6452793fe191b3638a5777005485bae145924044530ffa5f6122`.
- [Hamcrest-core1.3](https://repo.maven.apache.org/maven2/org/hamcrest/hamcrest-core/1.3/hamcrest-core-1.3.jar)
  SHA256 `66fdef91e9739348df7a096aa384a5685f4e875584cce89386a7a47251c4d8e9`.

Initial network attempt failed DNS inside the sandbox; explicitly approved retry
succeeded. Initial javac subprocess returned status0 plus EPERM; the approved
retry is the qualified build, with the original diagnostic retained in
`native-build-sandbox.log`. The compiler's unchecked-generic note is preserved.
Source/dependency/class fingerprints are in the two build manifests.

`run-boundary.mjs` and `run-shared.mjs` execute separate `-ea` JVMs in default
JIT and `-Xint` modes, each limited to 256MiB heap and 512KiB stack. Negative
reciprocal and zero-GCD probes have two-second external caps and SIGKILL cleanup.
Other processes have 30/60-second caps; none reaches them. Timing metadata is
process diagnostic evidence, **not a performance benchmark**.

## Arithmetic and cache results

`rational-corpus.tsv`: 1,575 cases, SHA256
`f66f1b004d3bb133b8b3e1dc93ba1ef6cef8f7c5b57a5485bebc3c44f7762b0f`.
Precisions 0,1,8,16,17,32,64,128,256, fresh nodes per row, signed magnitudes up
to one million. Rational input callbacks use exact floor at every requested
precision. The separate public-mul family uses only donor int/double factories.
An independent Node BigInt checker uses exact cross-products to check
|A-2^bits*n/d|<=1; no host float decides a pass.

Both modes produce identical corpus output:

| Operation | Pass | Requested-bound violation |
| --- | ---: | ---: |
| Negation | 81 | 0 |
| Addition | 486 | 0 |
| Rational-callback multiplication | 364 | 122 |
| Positive reciprocal | 36 | 0 |
| Public-factory multiplication | 412 | 74 |
| Total | 1,379 | 196 |

The multiplication scheduler takes log2(signed estimate+2), then catches
negative-log input and uses a constant budget. This misses large negative
magnitude. For example, a public-factory positive product can return 99,998
at precision0 for exact100,000. These are precision-guarantee violations, not
proof that the underlying approximation sequence never converges.

Additional identical outcomes in both modes:

- `equals`: 0≈2^-17 and 2^-17≈2^-16 but not 0≈2^-16, including the Field wrapper.
  Two separately constructed exact ones compare equal but have different
  hashes and occupy two HashSet entries. This violates the donor's own exact
  Set contract and Java equality/hash expectations.
- Negative reciprocal: public reciprocal(of(-1)) hits the cap. A valid negative
  estimator with a deliberate callback budget reaches 4,097 calls, last index
  4,112. Source search requires a positive estimate>=3, impossible for exact-1.
  The timeout alone is not treated as a proof of nontermination; the sign
  predicate explains the failure. No precision wraparound was reached.
- Scaling: five negative int scalars return zero; corresponding BigInteger
  cases stack-overflow. Integer embedding and int power at MIN_VALUE also
  stack-overflow. Thirteen zero/positive/BigInteger-power controls pass.
- Formatting: 71/91 integer estimates satisfy the requested 2^-m error,
  20 do not. At m=3, estimate(1,000,000) returns 960,000.00 because the decimal
  scale factor is truncated before multiplication. This does not claim every
  displayed approximation is wrong or that a fixed input fails to converge.
- All seven of(double) cases fit their exact shortest-decimal values, including
  subnormal/maximal inputs. Those semantics are not exact binary-float import.
- Cache: 300 signed floor/ceiling boundary cases pass. The algebraic proof in
  the file notes rejects the initial coarsening suspicion for nonoverflowing
  precision arithmetic. One initial constructor evaluation, one finer query,
  ten cached coarse queries, then ten equals calls yield callback counts
  1,2,2,12: equals bypasses the node cache.

`analyze-boundary.mjs` checks row coverage, exact errors, process outcomes and
output hashes. It initially miscounted the signed-search final index by one;
the corrected expectation accounts for constructor index16 and new17..4112.
A draft alternative cache estimator was discarded before native qualification
because its all-precision input contract was not proved; only exact floor/ceil
estimators occur in the qualified source and class manifest.

## Shared algorithms and resource-count experiment

Original `AlgorithmsTests.java` compiles and runs unchanged with three additional
unchanged integer classes. Both modes run11 tests:10pass,1failure.
`countQuadraticResidues` passes an even first argument to a Jacobi implementation
that interprets its first argument as the positive odd modulus. Full logs kept.
These tests do not certify the whole library; several print rather than assert.

Supplemental exact checks, identical in both modes:

- Logging wrapper add and multiply drop a nonzero2^-17 term because finite-bit
  equals says it is zero. Instrumentation therefore changes numerical semantics.
- Barrett 2,005-case grid:731 positive inputs below2^(2k) all produce canonical
  remainders;60 negative multiples inside the corresponding magnitude bound do
  not. Of the other1,214 inputs,1,035pass and179fail. Oversized inputs are kept
  distinct from the usual one-correction range. All five quotient-ring tests
  incorrectly distinguish -modulus from zero.
- FieldOfFractions.invert(zero) returns1/0 without a domain exception.
- BitLength(1,64) reaches StackOverflowError; BinaryGCD(0,1) hits the external
  cap, consistent with the source's repeating state(0,1).

The DAG resource experiment constructs x <- add(x,x) from an exact-one callback,
at depths0,1,2,4,8,12,16,20, then checks a128-bit answer and100cached64-bit answers.
Every numeric result is exact. Root string length is exactly4*2^depth-3: a
21-node shared graph has4,194,301 root-string characters. Its leaf callback runs
21times during construction, once more on refinement, and not during the100
cached reads. This separates useful cache reuse from eager representation-string
expansion. It is a deterministic resource-count experiment, not wall-time/RSS
measurement or a cross-language speed claim. No Hyper string field is proposed.

## Hyper controls and retention decision

Isolated publish=false Cargo harness, offline cached dependencies, current
Hyperreal working-tree source. `hyper-source-snapshot.json` pins all81 source/
manifest files before and after execution. Both debug and release pass:

- Same1,575 mathematical corpus cases through public Computable operations.
  Decimal-valued donor imports are matched mathematically, not silently replaced
  by Hyper's differently specified exact binary import.
- 60signed reciprocal checks;3close comparisons in one grouped check;
  36signed i64/MIN/MAX multiply/power checks;7exact binary-float imports.
- 480irrational checks: signed multiples of sqrt(2,3,5,7,11), optionally inverted,
  six requested precisions, after warming the768-bit cache. Independently
  directed4096-bit MPFR bounds must fit the resulting Hyper enclosure.

Summary:1,575corpus+584additional checks per mode, identical output SHA256
`0468914da477410a9b3acfd105aae6abdb7fe2b4cea3f397bc73497a81fd00e7`.
Debug binary `94c8534e3d08828c0b49214d91738a1a937f0b312af0d1aec2f1e5b9c0ba4cd7`;
release `78b1a8b32ac3977618fa5800225f85314500fb77eba15202e19a55317eef38ae`.
Initial harness type-inference failure is retained; fixed only in the harness.
An additional sandbox Cargo EPERM was retried with explicit approval. Neither
is a Hyper production failure.

No new production change is retained from this slice. Earlier coefficient,
domain, formatting and fractional-separation repairs remain unchanged. Unsafe
donor contracts are rejected, already-present architecture is not duplicated,
and the possible borrowed-cache-rescale optimization remains **unimplemented**
pending full isolated correctness/performance qualification. This checkpoint
does not complete Ruffini or the overall reference audit.

Revalidate without executing native programs:

```sh
node exact-real-references/ruffini-qualification/verify-checkpoint.mjs
```

Rebuild/run scripts are scoped to private artifacts and local execution only.
All owned processes were terminal at this checkpoint. No agents, commits,
pushes, donor modifications, or deletions were used.

Final concurrency check: the older fractional-certificate aggregate validator
passed at turn entry but correctly refuses its old downstream snapshot now.
Hypercurve HEAD advanced to a46b906a7167542b9decc0bd27a72cca38eeb34c, and the
user's live bezier_offset.rs is now SHA256
`4e9f51163bff9b5a9e298ab58fe5e09f0cd8313cd4a7ed095fcafb108a16f082`.
Region/policy hashes still match the earlier record. No downstream source was
changed by this audit. The earlier 902-pass Hypercurve result remains valid
only for its recorded old snapshot; it is not evidence for this newer live
source. The new Ruffini checkpoint and all81 Hyperreal source hashes still
validate. Frozen prior validators/artifacts were not rewritten to accept drift.
