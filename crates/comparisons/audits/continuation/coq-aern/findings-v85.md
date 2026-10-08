# coq-aern: scalar semantics and extraction boundary — checkpoint 85

Status: meaningful source and Hyper capability progress; **not repository or
ecosystem completion**. No production edit or new retained transfer.

## Source coverage and reproducibility

[Upstream](https://github.com/holgerthies/coq-aern) is pinned to
`bc11353f450cf866b47c3985eee6150a5f99cf00` (2026-07-14). The tracked inventory has
135 artifacts / 3,296,962 bytes: 88 text files / 42,419 lines, 41 binaries, two
historical CSV datasets, three symlinks and one vendored minified JS asset.
Inventory is not reading. [Read records](read-records-v85.json) credit **21
complete files and one partial, 4,666 lines**. The remaining 877 lines of
`Base/MultivalueMonad.v`, 66 wholly unread text files, and other artifact classes
remain open. No source-completion credit is assigned to transitive imports.

The read includes setup/build files; abstract base, monad and real axioms;
extraction mappings; multivalued limits; min/max, magnitude, search and rounding;
and the entire checked-in Max, Magnitude and Sqrt Haskell modules. The formal
sqrt proof has not been read in this checkpoint. Per-file observations accompany
the exact ranges and raw-byte hashes, rather than merely marking directories.

Four AERN2 runtime files were reread in the existing clean checkout
`d1ac3664bfb5c7f70fcf68f7fb412d288def65cb`: Real/Limit, Real/CKleenean,
Continuity/Principles and Select (413 lines). This checkout is **not assumed
identical** to coq-aern's Stack-selected AERN2 0.2.15.1 packages. These are
cross-reference rereads, not new coq-aern or whole-ecosystem coverage.

## Findings, in priority order

1. **Preserve the distinction between continuous values and exact decisions.**
   `real_max_prop` selects between `y-epsilon < x` and `x-epsilon < y`, so at
   least one branch can succeed even at equality. Every allowed result is within
   epsilon of the same maximum. The limit yields the maximum value without
   identifying a discontinuous winning operand. `RealLimit2.v` requires the
   all-branch fast-Cauchy/shared-limit properties; an arbitrary sequence of
   Unknown outcomes is not a substitute for those proofs.
2. **Proof erasure is a trust boundary, not a free correctness certificate.**
   `Extract.v` maps the multivalue monad to ordinary Haskell values and maps its
   operations largely to identity. Selection, limits and continuity observations
   are delegated to AERN2. Inverse nonzero proofs, positive magnitude domains and
   natural-number restrictions disappear at runtime. Calling erased helpers
   outside their formal domain does not demonstrate a valid-input donor defect.
   The explicitly unrealized boolean eliminator needs a reachability check.
   The suspicious `intro` / `real T_lt_plus_lt` text in Minmax.v:205-206 is a
   source/build concern, not an executed compiler failure. No proof-checking
   claim is made for this pinned tree.
3. **Coarse magnitude and rounding cannot replace exact predicates.**
   Magnitude guarantees a positive input lies between `2^(z-2)` and `2^z`, not
   a correctly rounded logarithm or exact leading exponent. Rounding produces
   an integer less than one unit away, not floor or ties-to-even. These are useful
   scheduling/enclosure contracts; Hyper's certified exact rounding APIs require
   stronger evidence. Overlapping search also does not promise the globally
   least index satisfying the first predicate.
4. **The implementation distinguishes fair refinement from ordinary scanning.**
   The reread CKleenean binary selection advances both approximation streams;
   countable selection traverses a diagonal enumeration so no stream is omitted.
   Generic non-stream Kleenean selection has a different contract. The sequence
   continuity monitor uses runtime access observation and assumes eventual true;
   it is not an unconditional terminating continuity oracle. Porting these ideas
   needs explicit fairness, termination/precondition and cache-cost qualification.
5. **The fixed-precision restart branch is not a keeper.** `ExtractMB.v`
   explicitly marks itself broken and explains that its error interpretation of
   the monad does not implement the intended nondeterminism. No transfer made.
6. **Checked-in extraction contains avoidable-looking helpers, but no size win is
   proved.** Sqrt.hs includes recursive integer arithmetic and an unused log2
   iterator. The live log2 call passes `succ n`; the zero-input mismatch in the
   replacement therefore is not demonstrated reachable for valid natural n.
   Haskell dead-code elimination and generated-code costs require a native build
   before drawing performance or binary-size conclusions.

## Hyper qualification

Hyper's `Real::min` / `max` return a borrowed operand, conservatively keeping
`self` when the bounded PartialOrd query is Unknown. This behavior is documented.
Its existing `abs` uses a computable `sqrt(x^2)` fallback when sign is unresolved,
so continuous extrema are already expressible as `(x+y +/- abs(x-y))/2`.
The experiment changes neither API nor production implementation.

The exact corpus uses `offset +/- sqrt(2)` and
`offset +/- sqrt(2+2^-k)`, equal controls, both operand orders, offsets 0 and 7,
and k = 0, 32, 128, 2056, 4096. Fresh operands separate the formula from preceding
comparison-cache warming. Certified intervals are requested 64 bits beyond each
gap (at least -96). Independent BigInt rational endpoint squaring verifies the
selected root and sign, with strict separation for every unequal pair.

- Debug and release match all 49 records / 143,914 bytes, including certificates.
- All 96 extrema enclosures are correct. Ordering gives 24 known unequal, eight
  Equal, and 16 Unknown results. In those 16 unresolved cases both borrowed
  operations keep the first operand, so one is not the mathematical extremum;
  this confirms the documented policy, **not a Hyper correctness defect**.
- Twelve evidence corruptions are rejected. A separate rational mathematical
  model checks 38,025 overlapping-predicate pairs and 39,970 admissible branches;
  this is not Coq proof checking or Haskell execution.
- Memcheck output is numerically byte-identical. Zero errors and zero
  definite/indirect/possible lost allocations; 3,008 bytes in 25 blocks remain
  reachable. The collector makes 39,929 allocations / 39,904 frees requesting
  5,175,511 bytes. These include collection and serialization, not isolated
  operation costs or a before/after memory improvement.
- Fmt, warning-denying Clippy and the offline 21-package graph pass. Hyper uses
  its default feature only. No new full-stack, all-feature, WASM, matched timing,
  allocation-delta or product-size qualification is claimed.

## Environment, storage and evidence limitations

The read-only PATH check finds Stack but no Coq/Rocq, GHC, runghc, Cabal or opam;
the inspected Stack compiler cache is empty. No native donor compilation,
regeneration, tests or benchmark were run. Three benchmark source symlinks point
outside the clone to absent iRRAM examples. Do not count historical CSV timing
results or checked-in binaries as fresh qualification.

The initial inventory subprocess attempt failed with sandbox EPERM; its approved
rerun created the inventory. Two later sandbox captures reported exit 0 but
contained no checker/environment output. They are preserved as unusable harness
captures; the approved repeats contain complete results. Numerical collectors,
compiler output and gate contents are checked separately from exit status.
The first evidence-sealing attempt then failed on an audit-only relative-path
mistake when locating the previously unretained candidate. It created no
manifest; the corrected lookup resolves against the original Calcium directory.
[Failure and correction](harness-failure-v85.md) are retained explicitly.

The existing Rust target is reused. Two new audit binaries total 5,150,272 bytes;
observed available `/tmp` space afterwards is 13,877,915,648 bytes. These figures
are not a product-size delta. No source-tree copy, broad cleanup, library rebuild,
large toolchain installation, external report, commit or push occurred.

## Disposition and next work

Keep Hyper's existing exact/Unknown distinction and continuous abs expression.
An owned extrema convenience API or direct node is still only an idea: a source
analogy and passing examples do not establish a worthwhile consumer, performance,
memory or code-size improvement. Benchmark any real candidate against equal-result
baselines before retention. Continue the formal monad/limit dependencies, formal
sqrt and other analysis/hyperspace modules, extraction support and benchmarks.

The seven previously retained continuation transfers remain unchanged. Calcium
still has 54 unread inventoried text files; its wider support, the other supplied
systems and historical transfer experiments remain open. Completing both qqbar
directories did not close any of those requirements.
