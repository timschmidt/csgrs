# coq-aern proof dependencies and square roots — checkpoint 86

Status: source and qualification progress; **no production change or new retained
transfer**. The original whole-ecosystem audit remains incomplete.

## Coverage and proof boundary

At the unchanged coq-aern pin `bc11353f450cf866b47c3985eee6150a5f99cf00`, this
checkpoint reads **4,454 new lines**, completes eight new files and finishes the
previously partial MultivalueMonad.v. Cumulative coq-aern coverage is **30 complete
text files / 9,120 lines, no partial files**. Fifty-eight inventoried text files
and the other artifact classes remain open. The [read records](read-records-v86.json)
contain per-file observations, exact ranges and raw-byte hashes. No dependencies
receive automatic read credit.

The complete new files are RealLimit0/1, Sqrt, Complex, MultiLimit, RealSubsets,
RealRing and extracted CSqrt. The monad read finishes lines 201-1077. Reading
formal source is not proof checking: no native Coq or GHC execution is claimed.

Two explicit proof holes occur in the now-completely-read Sqrt.v:

- `csqrt_solutions`, admitted at line 628, establishes that roots differ only
  by sign. General `csqrt` uses it to establish closedness of the solution set.
- `csqrt_small`, admitted at line 710, supplies the small-input/root bound used
  by the zero-approximation and transition branches.

Both statements are consistent with the independently checked mathematics, but
that does not fill either formal proof. These are verified source-level proof
gaps, **not demonstrated wrong runtime answers**. The real nonnegative sqrt
construction appears earlier and does not use these later lemmas; it still
depends on the abstract field/limit/choice assumptions and unqualified build.

## Transfer assessment, in priority order

1. **Coherent choices and closed predicates are indispensable.** The completed
   monad implementation distinguishes a set of whole paths from separate sets of
   values at each index, and relates them through an explicit coherence axiom.
   MultiLimit requires consecutive displacement at most `2^(-n-1)` plus a closed
   target predicate. Merely approaching some member at every precision does not
   establish a convergent path. Alternating between -1 and 1 is a counterexample.
   Likewise, positive approximants `2^-n` can converge to zero, outside the open
   positive set. Do not turn bounded Unknown results or nonzero observations into
   certificates preserved across an arbitrary limit.
2. **Classical existence is not an executable complete decision API.**
   RealLimit0 constructs order-completeness in Prop using classical/countable
   choice. RealLimit1 then uses unique existence and actual precision-indexed
   approximants to obtain executable limits. This is not a total computational
   supremum for arbitrary predicates. Hyperlimit remains the stack's certified
   geometric-predicate layer, not a general sequence-limit evaluator. Its
   uncertainty reports and Hyperlattice's unknown-zero rejection must remain
   separate from scheduling metadata.
3. **Use the exact Newton bound only in its stated model.** On `[1/4,2]`, the
   donor starts at 1 and uses `(y+x/y)/2`, proving error at step s is at most
   `2^(-2^s)`. Its requested-accuracy schedule is `bit_length(n+1)`. The least
   step count implied by that same bound is `bit_length(n)`, taking bit_length(0)
   as zero: `2^s > n` suffices for error strictly below `2^-n`. This removes one
   ideal iteration at n=0 and n=`2^k-1`. It is a donor scheduling opportunity,
   not a measured Haskell improvement or a Hyper patch.
4. **Signed scaling needs floor division.** The coarse magnitude exponent z is
   halved using floor, then x is rescaled by `2^(-2*floor(z/2))` into `[1/4,2]`.
   Truncating a negative odd z toward zero invalidates that interval; the model
   includes an explicit failing control. Hyper's numerical sqrt has a different
   magnitude/guard-bit contract. Its 59-result-bit seed crossover, four seed
   guard bits, and recursive half-precision-plus-six schedule are already present.
   Ideal exact Newton bounds do not prove that fewer rounded-arithmetic guards
   suffice. No guard or negative-MSD scheduling change is retained.
5. **Near-zero sqrt can avoid an exact zero decision.** The real construction
   chooses between proven positivity and `x < 2^(-2n)`. In the latter branch,
   zero is a valid `2^-n` approximation. This agrees with the purpose of Hyper's
   demand-bounded small-argument handling; it is not permission to treat an
   unresolved negative input as zero or to weaken the public sqrt domain check.
6. **Complex sqrt is a multivalued contract, not a principal-root shortcut.**
   The extracted nonzero routine chooses a safe component formula; the general
   routine uses a zero/root state and retains a selected root after leaving zero.
   Hyperlattice's inspected complex API has norms, inverses and integer powers;
   a new root API would need an explicit branch/uncertainty contract, coherent
   component evaluation, consumer justification and native qualification. No
   unqualified principal-root API is introduced.
7. **Generated-code size and prefix sharing remain hypotheses.** CSqrt.hs was
   read through all 1,319 lines, including repeated integer/vector helpers and
   extensive erased-equality scaffolding. `m_paths` rebuilds a fold over each
   requested prefix, while coordinate limits consume the path function. Compiler
   simplification, sharing, laziness and allocation costs must be measured in the
   actual Haskell build before calling this a memory/performance/code-size win.

## Executed qualification

The independent exact-rational model uses integer arithmetic and endpoint
squaring, not a floating approximation to sqrt. It verifies:

- 35 normalized inputs, 350 step/error bounds and 17,920 strict requested-error
  bounds under the original and tighter ideal schedules. The finite corpus
  saves 315 ideal Newton iterations; this is a deterministic operation count,
  **not a runtime benchmark**. Largest rational component is 17,035 bits.
- 65,589 planner indices, including all 0..65535 and sparse boundaries up to
  4096-bit indices; large indices test only integer planning, not enormous Newton
  evaluations. Thirty-four sampled schedules lose one redundant iteration.
- 1,161 valid coarse-magnitude/scaling combinations, 8,481 near-zero selection
  pairs and 4,112 admissible zero branches.
- 1,089 rational complex roots giving 545 distinct squares, with at most the
  two opposite roots per square. For the small-root lemma, 7,837 nonvacuous
  premises pass and 10,676 inputs lie outside its premise. These are models of
  the admitted statements, not repairs or executions of the formal development.
- Two semantic countermodels and six deliberately false numeric/scheduling
  controls. The deterministic trace hashes 67,100 records.
- A separate branch model checks all 2,048 admissible nonzero component formulas
  on 1,088 nonzero inputs, with 512 checks per branch. Two distinct valid outputs
  occur for 240 inputs; five corrupted controls are rejected. At `z=-3-4i`, the negative-real branch returns
  `-1+2i`, while the negative-imaginary branch returns `1-2i`. Both square to z;
  mixing their components fails. This supports the multivalued-contract reading,
  not a principal-root claim or native Haskell validation.

The native Hyper sqrt-filter regression passes **23 tests in debug and 23 in
release**, with the same test-name set. It includes the directed-MPFR raw-node
seed test, square-root identity checks, cache/history tests and aborted-evaluation
handling. This is a focused default-feature library gate, not the full workspace,
all-feature matrix or WASM qualification. The first `--exact` invocation used an
unqualified name and ran zero tests; its successful exit is explicitly **not**
counted as qualification. Corrected invocations run the stated 23 tests.

The first fresh checkpoint-85 verification failed on a sandbox-denied Node/Git
subprocess. Its approved rerun passes. Both captures are preserved. The
branch-model sandbox capture also exits zero with empty output and is unusable;
the approved repeat contains the checked result. One broad
process listing was truncated; the final specific PID check confirms the already
completed model collector is absent. No completion inference uses that truncated
listing.

## Disposition, storage and remaining work

Keep the existing Hyper evaluator and uncertainty contracts. No production source,
API, tests or benchmark code changed; seven earlier continuation transfers remain
retained. No new matched timing, allocation delta, memory or binary-size win is
claimed. Cached test executables were reused without compilation; no new audit
binary, source-tree copy, donor rebuild, large toolchain install or cleanup.
Observed `/tmp` availability after the tests is 13,877,891,072 bytes; this is an
observation, not a product-size measurement.

Next: continue remaining formal base/metric/relator and Euclidean dependencies,
analysis and hyperspace code, extraction support and benchmark files. Native
Coq/GHC tests and prefix-sharing/scheduling experiments remain open. Any future
Hyper guard-bit, negative-MSD planning or complex-root candidate needs a separate
proof-sensitive trial and matched qualification before retention. Calcium's wider
support, other original references and historical experiments remain in scope.
