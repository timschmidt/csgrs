# Zero quotient composition candidate

Baseline `3eca3cce8` preserves the three failing scalar regressions and the
diagnosis against production revision `7cf9d82ac`. Hyperreal now discards an
unused denominator when a quotient numerator proves zero. Addition consumes
both admitted operands before applying zero identities; cross-numerator
cancellation avoids constructing an irrelevant denominator product. Scaling
and multiplication keep the same zero representation. Inverse operands still
need strict nonzero evidence, and both children are expanded before absorption.

No proof limit, precision, policy, fixture resolution, or validation step was
weakened. This is a bounded symbolic-completeness extension, with no expression
rewrite, persistent cache, tolerance, or approximate proof. The scalar layer
owns the identity; modeling and native mesh predicates use its shared query.

## Acceptance evidence

`zero-candidate-v1` rebuilds the complete selected workspace dependency graph
against pinned external libraries, then executes the original modeling tests.
The full 795-test scalar unit suite and seven public quotient tests pass.
All four context tests pass, as does the independent polygonal-membership,
volume-enclosure and manifoldness test over the original 8/16/24/32 drills.

The complete composition tests retain their original assertions:

| Segments | Initial strict drill | Empty union and membership | Complete subtraction and reunion |
| --- | --- | --- | --- |
| 8 | Certified | Certified | Certified |
| 16 | Certified | Newly Certified | Still undecided at subtraction |
| 24 | Certified | Certified | Certified |
| 32 | Certified | Newly Certified | Still undecided at subtraction |

The 16/32 tests now fail at line 99, after certified accumulation and membership,
with `PredicateUndecided { predicate: "Real sign classification" }`. They remain
failing tests, not accepted `Unknown` outcomes. Full workflow completion remains
two of four cases. Initial-drill topology and the polygonal six-volume enclosure
are separate claims; exact equality of the large symbolic volume and fidelity
to an analytic cylinder are not claimed.

`zero-qualification-v1` passes default/all-feature scalar tests and scalar
Clippy, then all 28 doctests fail to compile because rustdoc writes into the
exhausted `/tmp` quota. Preserve that complete failure. The fresh
`zero-qualification-v2` reruns the entire focused command set with `TMPDIR`
inside its independently seeded target. Final results are in its `result.json`:
912 default-feature tests, 1,019 all-feature tests, 28 doctests, workspace
formatting, and scalar and root-curve Clippy with warnings denied. Counts
overlap; this is not the complete repository CI matrix.

After the direct downstream run, the public zero-quotient test gained a serde
assertion that its authoritative serialization survives certification unchanged.
The final Cargo suite checks it. Production code and downstream tests did not
change between those runs. Source hashes and exact commands identify both
snapshots.

## Remaining subtraction investigation

Baseline diagnostic v4 establishes the actual degree-limit declines, preserving
the failed earlier harness trials. Conditional independent algebra shows that
zero normalization lowers the empty-union captures' maximum product degree
from 9 to 4; direct subtraction still reaches degree 10. An additional exact
polynomial-division experiment lowers all four old captures to degree 2 with
one successful factor division per expression. It is evidence for a candidate
direction, not a native proof or a result for the newly composed subtraction.

Reproduce the remaining predicate on the current candidate before implementing
factor cancellation or native construction-proof retention. Preserve earlier
successful bounded proofs and every resource cap; speculative reduction must
not make previously certified inputs undecided. Keep the full hard cases and
add denominator-domain, nonzero displacement and exhaustion controls for any
new reduction.

## Scope and preservation

Every trial uses external wall-time and per-process address-space containment.
These are not internal total-work or heap-byte limits. No comparative speed,
allocation, peak-memory or binary-size claim is made. Broad downstream suites,
matched cost/artifact comparisons, the external user path, release validation
and the complete properties plan remain open. The 35 pending native changes
remain separately unadmitted and unchanged.

All earlier evidence, failed trials, diagnostic copies, binaries and compiler
artifacts are preserved. No artifact consolidation was performed in this
iteration. The checkpoint verifies their hashes and permits a ledger edit only
inside `current_work`; it does not turn this tested candidate into full runtime
admission.
