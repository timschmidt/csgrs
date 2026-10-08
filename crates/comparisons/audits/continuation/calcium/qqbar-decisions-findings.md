# Algebraic representation and exact decisions — checkpoint 50

This is a scoped continuation, not completion of qqbar, either whole library,
or the original ecosystem inventory. Both pinned implementations were read
independently. There are 92 new complete-file reads / 8,646 unique lines:
46 archived Calcium files / 4,321 lines and 46 current FLINT files / 4,325 lines.
The exact list, notes, ranges and hashes are in qqbar-decisions-read-records.json
and qqbar-decisions-manifest.json. Twenty of these files are upstream tests;
they were read, not newly executed.

## Representation and proof boundaries

Both qqbar headers and complete manuals describe a reduced minimal polynomial
over the integers plus a complex isolating enclosure. The former identifies
an algebraic conjugacy class, not an individual root. Enclosure identity is
therefore essential. Copying owns a polynomial/enclosure copy; it is not shared
lazy expression storage. The hash currently uses only polynomial coefficients,
so conjugates intentionally collide. Hash equality is not value equality.

Exact equality first checks object identity and canonical polynomial equality,
then rational degree, disjoint enclosures and enclosure containment. Remaining
cases refine both enclosures and certify uniqueness of their union. These
steps rely on valid minimal polynomials and existing root isolation. Applying
the same shortcuts to arbitrary annihilating polynomials or arbitrary balls
would erase their proof preconditions.

The two interval-Newton validators have different contracts. The uniqueness
validator assumes root existence, can accept exact intervals/linear polynomials
immediately, and inflates the enclosure. The existence-and-uniqueness validator
does not inherit those shortcuts and also checks the missing component for a
pure real/imaginary interval. Neither a zero-containing residual nor a guessed
algebraic relation alone establishes a selected-root identity.

Component signs exploit rational degree, exact zero components and certified
separation. Undecided zero components require root-existence evidence on the
corresponding axis; the real-part path filters impossible pure-imaginary roots
using odd polynomial coefficients. Comparisons use enclosure disjointness,
rational/conjugacy shortcuts, then refinement; difficult nonreal comparisons
eventually construct an exact difference. Absolute-value comparison can use
exact squared magnitudes. No arbitrary computable-real equality algorithm is
obtained from this algebraic-only representation.

Raw refinement can return an already adequate enclosure without rounding. It
otherwise uses interval Newton and, when convergence is inadequate, recomputes
all conjugates and requires a unique overlapping root. Current FLINT adds a
real-root-specific refinement route absent from the archive. Those recursively
called root-finding kernels are not newly source-complete here.

Polished output separately resolves exact zero components and tries to recognize
dyadic parts. Ordinary readonly queries do not retain their expensive refined
enclosures. Explicit cache polishing swaps only a contained enclosure; this is
not automatic shared memoization. The manual distinguishes heuristic height
limits and relation guessing from rigorously checked field identities. Formula
flags still include unimplemented methods, and serialized algebraic input is
documented as trusting valid representation data. No malformed-input probes run.

## Independent mathematical qualification

The new collector constructs the 49 values
`sign(a)*sqrt(abs(a)) + i*sign(b)*sqrt(abs(b))`, for integer `a,b` in `-3..3`,
using documented constructors only. These are degree-at-most-four algebraic
values, not approximations passed off as exact objects. Every ordered pair is
queried before and after explicit 256-bit enclosure caching. Copies and
readonly-input polynomial/enclosure preservation are checked. Polished output
is requested at 32, 128 and 512 bits.

The independent JavaScript oracle uses signs and integer squares, not qqbar
operations, floating-point square roots, or interval overlap. Each exported
dyadic endpoint is checked exactly with BigInt arithmetic against its known
coordinate. Squared norms are the small integers `abs(a)+abs(b)`. Root order is
derived directly from the documented real-first/descending-real/ascending-
imaginary-magnitude/descending-imaginary-sign order, including equality.

All 5,293 records are present: 98 unary, 4,802 ordered-pair, 294 enclosure,
98 preservation and one terminal record. The oracle checks 35,826 assertions:
35,562 pass and 264 fail. The mathematical gate exits 1; it is not relabeled a
pass by the later evidence-integrity verifier.

Two contract failures are distinguished:

1. Root ordering returns a nonzero result for equal nonreal values: 84 same-
   object pair results and 84 distinct-copy results, representing 42 nonreal
   values in two states, not 168 different defects. The final sign tie-break
   does not compare both signs. All distinct-value root orders in this corpus
   match the independent expected order. The root-construction source calls
   this comparator after duplicating roots by multiplicity; however, this pass
   does not demonstrate an incorrect root list or run sorting stress probes.
2. Polished output leaves an exactly representable component inexact in 96
   outputs: 48 real-component and 48 imaginary-component observations across
   16 values, three precisions and two states. The affected exact components
   are +1 or -1 paired with an irrational component. The source scales to an
   integer grid, then uses the same positive exponent when reconstructing a
   candidate, instead of undoing the scale. The exact sign check rejects that
   candidate. This is a failure of the documented exact-component polish
   promise, not a demonstrated incorrect enclosure.

All 588 real/imaginary containment checks and all 588 `prec-2` relative-accuracy
checks pass. The tolerance is the one used by the read upstream test, not a
claim of correctly rounded nearest output. All 28,812 ordinary pairwise
equality/component/magnitude comparisons, component and complex signs, copy
equality/hash consistency, and readonly input-preservation checks pass.
No branch coverage, general minimal-polynomial certificate, arbitrary close-
root corpus, algebraic LLL qualification or archived runtime claim is made.

The read upstream tests compare shared qqbar operations, enclosure overlap,
zero-containing polynomial residuals or conjugate sums. They do not independently
establish comparator equality or exact dyadic polishing. Their existing strengths
and limitations remain distinct from the new independent corpus.

## Memory, storage and evidence

Native and Memcheck output are identical, 658,089 bytes each (1,316,178 paired
workspace bytes). The focused Memcheck exits zero with zero errors/suppressed
errors, no blocks live at exit, and 2,729,471 allocations/frees totaling
154,454,444 cumulative requested bytes. This includes setup, queries and output;
it is neither a donor-only allocation benchmark nor a peak/RSS measurement.
Clean memory instrumentation does not repair the failed mathematical assertions.

The first evidence-binding attempt failed before writing coverage or a manifest:
it compared an in-memory JavaScript result with parsed JSON, which omits optional
undefined fields and normalizes negative zero. The initial scripts and a full
failed capture are preserved. The corrected binding compares JSON wire forms;
it changes no oracle assertion, numerical output, failure count or donor source.

Only one 18,152-byte executable was added under
`/tmp/calcium-qqbar-decisions.xUeflv/controls`. The existing pinned FLINT and
system libraries are reused and hashed; linked paths are separately captured.
No Rust rebuild, native dependency rebuild, source-tree copy, cleanup, deletion,
donor patch, external report, commit or push was performed.

## Comparison with Hyper and disposition

Hyperreal's current bounded algebraic-integer quotient separation metadata
already supplies a zero-separation certificate without constructing a canonical
minimal polynomial for each expression. Generator/height/work caps fail closed.
Its sign dispatcher tries stored facts and cheap separation before paying for
algebraic metadata, and returns Unknown when its budget cannot certify a result.
The exact read ranges are bound separately from donor coverage.

Hypersolve's represented-root comparison already checks local evidence under
the strict policy, recognizes exact points/same representations, uses disjoint
isolators and bounded Sturm refinement, then tries common-root certification
and an exact difference. Its sign path can resolve a zero-touching unit isolator
from the constant coefficient. These are not the same invariant as canonical
irreducible integer minimal polynomials; the documented local validation does
not itself rerun the upstream uniqueness proof. No new global soundness claim
is made for that boundary.

Do not replace these mechanisms with qqbar's unchecked representation assumptions,
uncached repeated refinement, or its root-order/polishing implementations.
The promising general ideas—cheap facts first, exact root identity, precision
refinement separate from exact facts—already exist in Hyper. No new production
candidate is justified by this slice, so no matched performance or size gain is
claimed and no benchmark is substituted for a missing candidate.

All five previously retained continuation transfers remain unchanged. Continue
the unread algebraic arithmetic, relation/field-expression code, generic adapters,
root-finding/recursive polynomial support, remaining formal/symbolic/historical
references and full original inventory reconciliation. The full goal stays open.
