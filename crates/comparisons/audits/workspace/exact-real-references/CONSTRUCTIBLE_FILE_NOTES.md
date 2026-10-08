# haskell-constructible audit

Pinned source reading and declared numerical/performance/Hyper transfer audit
complete. One cold fractional separation certificate improvement is retained
after qualification; original donor is unmodified and its boundary defects are
documented. The full ecosystem goal remains OPEN. Run
constructible-qualification/verify-closure.mjs for aggregate evidence checks.

Source: https://github.com/andersk/haskell-constructible, pinned at
`46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0` (2021-11-09, release 0.1.2).
All four tracked regular files, 490 physical lines, independently read. No
tracked symlinks, gitlinks or AGENTS.md. Original checkout clean after reading.
CONSTRUCTIBLE_READ_COVERAGE.tsv records hashes and actual contiguous read ranges;
constructible-qualification/inventory.mjs verifies them. Repository webpage was
checked against the user-supplied URL; it is not substituted for source reading.

## File-by-file findings

- LICENSE 1-29: BSD-3-Clause, Anders Kaseorg 2013. No donor code copied.
- Setup.hs 1-2: standard Cabal defaultMain, no custom build behavior.
- constructible.cabal 1-36: one library module, no test/benchmark suites;
  dependencies base, binary-search, complex-generic and integer-roots. Version
  0.1.2 and repository tag are declared. Package description explicitly limits
  the exact domain to field operations and positive square roots.
- Data/Real/Constructible.hs 1-62: public abstract Construct, deconstruction,
  floating approximation and exception API; doc examples include golden-ratio
  powers, finite AGM arithmetic, a nested radical, and a 17th-root-of-unity
  identity. Imports delegate integer roots, unbounded integer search and
  complex instances to dependencies, which require separate qualification.
- Lines63-170: typed quadratic field towers, coefficient pairs and canonical
  SqrtZero. Addition/subtraction normalize zero; multiplication/squaring rely
  on irreducible-field/nonzero-input invariants instead. Reciprocal descends
  through a quadratic norm. Ordering uses exact coefficient signs and norm
  comparisons, not an unbounded approximation/equality search. These guarantees
  depend on adjoining only genuinely new positive square roots.
- Lines171-187: square-membership test. Rational exactSquareRoot is the base;
  the quadratic step uses the norm and two candidate coefficient arrangements.
  Its completeness and positive-root selection are major test obligations,
  especially for alternate embeddings and nested radical identities.
- Lines189-219: precedence-aware exact expression printing, including signed
  coefficients and recursive radicals. Zero/one special forms reduce output.
- Lines221-245: floating conversion uses quadratic conjugation for unlike-sign
  terms, avoiding direct cancellation. It still converts a radicand and its
  coefficient separately; finite mathematical results may overflow/underflow
  intermediate target floats. Candidate probe: sqrt(2*10^1000)/10^500 should
  approximate sqrt(2), but the displayed evaluation branches suggest 0*infinity.
  This source-derived hypothesis was subsequently reproduced in both O0/O2:
  five extreme-scale Double views are nonfinite while exact identities hold.
  Hyper's matching exact/finite/MPFR controls pass (boundary-README.md).
- Lines247-286: existential field wrapper, exact deconstruction and recursive
  field joining. Joining first asks whether the next radicand already has a
  square root in the destination; only otherwise does it extend the field.
  Equal values need not deconstruct identically, as the API explicitly states.
  Field rebuilding/search can be expensive even for repeated compatible work.
- Lines287-325: expression printer/parser, exact Eq/Ord, arithmetic and rational import.
  Every binary operation performs field joining. No shared DAG/approximation
  cache, bounded-precision demand interface, or MPFR error contract is exposed.
- Lines327-383: explicit exceptions and partial Floating instance: square root
  and dyadic-rational powers only; transcendentals deliberately unsupported.
  This is an algebraic/constructible system, not a general computable-real
  substitute. Negative-base, zero-power and exceptional exponent cases need
  independent operation-contract tests.
- Lines385-391: rational extraction rejects irrational values. Irrational
  properFraction uses integer search minus one. A likely floor-versus-truncate
  discrepancy for negative irrational inputs was subsequently confirmed
  against the fully read Numeric.Search.Integer.search contract and executed
  signed probes: five negative irrational fractions have the wrong sign.
  Hyper's matching truncation/fraction controls pass (boundary-README.md).
- Lines393-411: enumeration and Template Haskell derivation of complex methods.
- Lines413-423: documented cancellation-stable floating conversion and a
  close radical-sum example. Stability is not a proof of correct rounding or
  immunity to intermediate overflow.

## Transfer questions, in priority order

1. Compare exact norm-based sign decisions and constructive square-membership
   against Hyperreal's radical certificates and Hypercurve/Hypersolve's selected
   field machinery. Identify an actual missing identity before changing code.
2. Check square-root branch selection, negative properFraction, dyadic powers,
   equality across different field towers, and float conversion with independent
   rational/MPFR oracles. First establish native dependency/version provenance.
3. Measure repeated field-join work and coefficient growth; do not replace
   Hyper's demand-driven shared scalar DAG with eager quadratic towers solely
   because this narrower domain has decidable equality.
4. Test whether conjugation helps a current conversion gap, including the
   extreme-scale probe above. Hyper's existing scale-aware conversion may
   already solve it; avoid duplicate symbolic rewrites and object-size growth.

The initial hypotheses are now partly resolved by native evidence below.
Reading all source lines is not whole-target completion.

Initial GHC9.6.7 `-fno-code` check (unaltered source) exits1 because the selected
package environment lacks complex-generic, integer-roots and binary-search.
See constructible-qualification/native-build-initial.log. This is a dependency
availability result, not a numerical or language-compatibility failure. Next:
resolve exact dependency versions and inspect their relevant contracts before
native execution. No donor file was modified by the check.

## Qualification continuation — 2026-09-06

Dependencies were subsequently pinned, byte-verified and compiled unchanged;
the old Cabal upper bounds are bypassed by direct GHC module compilation, not
claimed satisfied. See constructible-qualification/boundary-README.md.

Native exact field corpus: 1,324/1,324 in O0/O2. Hyper agrees on every proved
answer; 14 depth-five inverse identities are UNKNOWN at -2048, four remain at
-4096, all prove by -16384 in debug/release. The independently expanded squares
and sign oracles are explained in constructible-qualification/field-README.md.
This demonstrates a refinement-cost difference, not a wrong answer.

Corrected operation contracts 126/126 and four documentation examples pass in
both native builds. A full fresh-construction paired benchmark has 56 measured
pairs across seven families, plus warmups, with 1,103,040 checked comparisons.
Hyper/native CPU ratios range .2136–.8111 for shallow cases, 1.2069 and 2.0862
at depths four/five. Timings include parsing/ingestion and different language
runtimes; whole-process RSS and standalone binary sizes are not library-only
memory/code-size claims. Exact sample definitions, confidence intervals,
failed initial harness attempts, and frozen hashes are retained in field-README.

Hypercurve already implements recursive shared positive-root fields and norm
sign decisions; Hypersolve already reduces selected-field rational functions.
The donor's Q-base square-membership completeness assumptions do not transfer
unchanged to these broader bases. No duplicate eager backend was adopted.
Cold scalar proof cost remains a measured follow-up. API qualification now
passes8113checks/build; floating qualification covers194signedcases/build
against4096bitdirectedMPFR, including512bitHyperenclosures. See api-README.md,
float-README.md and analyze-api-float.mjs. This was the intermediate checkpoint;
the final transfer outcome follows.

## Final transfer decision

The cold proof-cost question produced a worthwhile Hyper change: keep an
algebraic-integer numerator and denominator in the existing bounded separation
metadata, and swap their bounds at inverse instead of repeatedly taking integer
norms. Exact proof, invalid-denominator guards, signed perturbations, directed
oracles, full downstream gates, before/after measurements and limitations are
in constructible-qualification/fractional/README.md. No eager quadratic backend
or scalar object growth. All1,324corpus identities now resolve at512bits versus
28UNKNOWN before; deepest paired CPU≈30.1%ofbaseline, allocatedbytes≈19.7%.

The scoped pinned-source audit is now qualified-complete, including its
documented native properFraction/finite-view limitations. This is not a claim
that every possible donor input is correct, that equality is generally decidable,
or that the entire reference inventory has been audited. verify-closure.mjs
composes the source/dependency/numerical/benchmark/transfer evidence checks.
