# Expression formatting, builtin symbols and streams — checkpoint 49

No new production or donor change is selected. This checkpoint closes source
reads, not runtime qualification, for the expression formatting/builtin slice.
The full ecosystem audit remains incomplete. The five retained continuation
improvements and all prior successful and failed evidence remain unchanged.

## What was actually read

Both complete `fexpr/write_latex.c` implementations; both builtin headers, tables,
lookup and inline-emission files; and both complete builtin manuals were read
line by line. The stream dependencies led to both complete `calcium/` support
directories and both `calcium.h` headers. Previously credited header ranges are
excluded from new line counts. One short truncated archived formatter section
was reread before credit; paired source similarity is not used as read credit.

The 25 new read records add 13,667 unique lines: 24 previously unread complete
files and completion of the archived calcium header's partial record. Both
expression and builtin directories, headers and manuals are now source-read,
as are both calcium support directories. This does not close recursive Arb,
polynomial/GCD, algebraic or general I/O support, let alone the original ecosystem.

The pins remain Calcium `8dbb16fc4fe92eaf3ebbc7478d629e994d39f944` and FLINT
`e269d38061d7a42070ddcffe6eb114466ed4aa7e`. The inventory and source hashes bind
the credited ranges. Current generated `config.h` lines 201–227 and template
`config.h.in` lines 200–226 were also read to understand the optimization macros;
these separately hashed auxiliary inputs are not added to donor coverage totals.

## Semantics and completeness

The catalog is a symbolic vocabulary, not an implementation inventory. It contains
names for calculus, quantifiers, sets, special functions and extended-number
domains even where no numerical interpreter implements them. The manual explicitly
describes eventual support, and many entries are declaration-only documentation.
The 474 entries in each pin agree in identifier, spelling, LaTeX text and callback.
Static catalog consistency does not establish evaluation or exact-real closure.

Binding forms such as `For`, `Where`, `Def` and `Fun` have intended scope semantics.
The structural replacement routine reviewed at checkpoint 48 is not a general
capture-avoiding evaluator for that language. Likewise, a displayed `Equal`,
`Same`, nearest-decimal claim or uniqueness operator is syntax, not proof that
the assertion has been checked. Compatible overlapping cases are a semantic
precondition in the documentation, not a nondeterministic exact-real decision
algorithm implemented by the formatter.

LaTeX arithmetic uses structural signs, parentheses and display conventions.
Calculus and special-function writers construct notation; they do not compute
derivatives, integrals, limits or roots. Range, collection, step and matrix
previews substitute a few endpoint/next-point expressions and insert ellipses.
That is not enumeration or proof of the represented range's cardinality.
`ShowExpandedNormalForm` explicitly runs the formal normalizer during formatting;
its documented cost warning is real architectural coupling, and its result
does not discharge the authored-domain obligations discussed at checkpoint 47.

## Performance, storage and size ideas

Most traversal uses borrowed child views and streams text to one destination.
The string stream starts at 16 bytes and grows geometrically. Some sign-dependent
sum/fraction/polynomial rendering materializes a child string to inspect its
leading character; other branches create expression owners for substitutions.
Thus neither allocation-free formatting nor work proportional only to emitted
text follows from the representation. No new timing, allocation or peak-demand
measurement is claimed.

The alphabetically ordered table supports binary-search name lookup without a
mutable index. It also stores display strings and formatting callback pointers,
so lookup metadata directly refers to the presentation implementation. The
current formatter adds internal linkage to several helpers and an `Os` request;
the observed generated configuration expands that request to a GCC optimization
pragma. These are source-level size-conscious choices, not evidence of a binary
size reduction in Hyper or an actual static-link retention measurement.

Hyper already emits ordinary `Real` structure through typed formatting and sends
literal fragments to `Formatter::write_str`. Fixed decimal `Computable::Display`
uses a demand-bounded leading-bit search before requesting a certified binary
approximation. Scientific formatting still uses `iter_msd`; this checkpoint does
not claim every display mode is productive for every opaque exact zero, or that
all decimal ties are correctly rounded. A broad display/LaTeX registry would be
a different capability, not an established scalar exactness or performance gain.

## Defensive output and ownership review

The display layer is not a verified serializer or a proof boundary. String
escaping is explicitly unfinished. The source also raises ownership and
temporary-buffer provenance concerns in symbol formatting. The file-backed
stream passes text to a formatting API, whereas the string-backed route copies
literal bytes. These distinctions rule out transplanting the code as an
arbitrary-text safe writer. Prefer literal output APIs and explicit ownership
if applying the general streaming idea in Hyper.

These are defensive source-review observations. No malformed-input, memory-error,
resource-boundary or old-crash reproduction was run. No donor patch or external
report was submitted, and no clean-memory or safe-parser claim is made here.

## Qualification and decision

The new checker independently parses the static enum/table/documentation rows,
checks names, indexes, lexical order, paired metadata identity and that each
registered callback has one definition in the read formatter implementation.
This is a bounded metadata check, not a C runtime test or mathematical oracle.
Existing expression tests were source-read at checkpoint 48; the arbitrary-syntax
LaTeX smoke test remains unexecuted. Earlier numerical campaigns retain exactly
their earlier scope and cannot qualify all formatting paths.

No new implementation candidate justifies a matched benchmark. Keep Hyper's
shared scalar graph, demand-driven numerical contract and explicit proof state.
Retain the design lessons of borrowed traversal, literal streaming and separating
presentation metadata from semantic capabilities. All work here is workspace
source/evidence recording; no new `/tmp` build, cleanup or deletion is needed.
