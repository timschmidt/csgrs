# Expression representation and numerical interfaces — checkpoint 48

This is a source-only architectural checkpoint, not a new numerical campaign.
No production or donor change, new transfer, performance result, memory gate or
whole-ecosystem completion is claimed. The five previously retained continuation
changes remain the selected changes. All prior evidence is preserved.

## Source scope and provenance

Pinned sources are standalone Calcium
`8dbb16fc4fe92eaf3ebbc7478d629e994d39f944` and FLINT
`e269d38061d7a42070ddcffe6eb114466ed4aa7e`. The read records identify each file and
range and the manifest binds its inventory hash. The previous checkpoint's
already-complete files are not credited again. Current manual lines 490–585
were previously read; this checkpoint credits only 1–489 and 586–620.

Both `fexpr.h` headers, both complete manuals and every file in the two `fexpr/`
directories except the two `write_latex.c` implementations have now been read.
This includes the formatter tests, not their implementation. The new records
cover 8,321 unique lines in 88 files: 87 previously unread files and completion
of one partially read manual. Builtin headers/tables, LaTeX implementations,
recursive polynomial/GCD and numerical support remain separate open work.
The full original reference inventory remains in scope.

The earlier expression read batches are recorded along with the final current
replacement test and five archived tests. Truncated navigation output received
no credit. Two truncated manual chunks were reread in full before credit.

## Storage, construction and reuse

The donor representation is an owned flat word array with compact tagged atoms.
Calls concatenate the function and arguments, copying their complete contents;
they do not retain shared expression-node pointers. Calls with at least five
arguments carry argument-count/offset information and an index every four
arguments. This makes in-bounds random access skip at most three intermediate
arguments; it does not eliminate construction copies or provide cached numeric
evaluation. Whole expressions grow their capacity geometrically. Each initialized
owner initially allocates a word, including zero; vector capacity management
initializes and retains slots. These are source properties, not measured allocator
or peak-memory comparisons against Hyper.

Read-only views borrow the owner's storage and must not outlive it or be cleared
as owners. Generic call construction forbids output overlap with inputs. The
builtin unary/binary wrappers explicitly handle whole-object output aliasing via
a temporary and swap; arithmetic wrappers use those paths. Replacement likewise
supports replacing an expression in place, but not overlapping the rule vectors.

Replacement is structural and simultaneous: first matching rule wins, inserted
right-hand sides are not recursively rewritten, and the function head is also
visited. Unchanged children are viewed rather than independently copied during
the traversal; changed branches get owners before the result is flattened again.
It uses stack descriptors for small calls and heap descriptors for larger ones.
That local allocation discipline is useful, but it is not persistent DAG sharing.

Equality, fast ordering and hashing operate on representation, not on real-number
semantics. Unique vector insertion is linear in its retained entries; sorting
uses structural ordering. Compact symbols/strings and builtin IDs could be useful
in a separate syntax or interchange layer. No end-to-end workload demonstrates
that replacing Hyper's scalar graph with them is worthwhile.

Hyper's actual `node/representation.rs` lines 1–265 retain `Arc<Node>` sharing,
immutable expressions, atomic facts and a lazily allocated `RwLock` containing
one approximation. Cache publication cannot replace a finer value with a coarser
one; coarsening borrows the stored integer under the read lock. This is the code
used for the comparison. The computable README's description of a lock-free
swap cell is stale relative to that implementation; it is not used as evidence
for a lock-free claim. No previously bound production documentation is altered
by this source-only checkpoint.

## Exact construction is distinct from approximate display

Integer and rational constructors preserve exact values structurally. A canonical
rational with denominator one becomes an integer; otherwise it becomes a division
expression. Finite ARF construction extracts an exact integer mantissa and binary
exponent: small nonnegative exponents become integers, small negative exponents
become rationals, and other cases retain a power of two with an optional mantissa
factor. Binary64 construction first imports the exact binary floating-point value
into ARF. Complex binary64 construction adds an explicit imaginary unit. This
offers no missing mathematical closure relative to Hyper's existing exact leaves
and binary scaling. Nonfinite constructors use symbolic infinity/undefined atoms;
that syntax is not a finite-real certificate.

The numerical interpreter recursively evaluates the supported syntax at one Acb
precision, with left-folded variadic arithmetic and special-function dispatch.
Unsupported or nonfinite evaluations return failure/indeterminate. The formatting
retry loop increases precision within a finite cap and returns the last evaluation
success flag; reaching the requested accuracy is a separate condition for leaving
the loop early. Decimal output then omits the radius. Consequently, success here
is not a proof of the requested accuracy and is not a substitute for Hyper's
demand-driven certified approximation contract. This conclusion follows from
the source control flow; no newly demonstrated inaccurate output is claimed.

The integer-extraction documentation and implementation disagree about the
noninteger failure contract. This is recorded as a source/documentation mismatch,
not as a newly reproduced failure. Symbol/string printing is also not a general
escaped, verified serialization format. No invalid-input, resource-boundary,
malformed-alias or old-crash probes were executed.

Formal rational-function normalization from checkpoint 47 remains a separate
domain issue: it treats terminal expressions as formal indeterminates and cannot
establish their independence, selected-root obligations or the original authored
denominator domain. It must not silently strengthen a partial-real claim.

## Test interpretation and decision

The source tests check builtin ordering/lookup, integer roundtrips, call views
and copies, replacement against a simpler recursive implementation and documented
whole-expression aliases. The LaTeX test generates arbitrary syntax and only
checks that formatting returns a string; it is not a mathematical or formatting
correctness oracle. These tests were read, not run in this checkpoint. No new
runtime, allocation, binary-size or source-size gain is claimed.

Existing checkpoint 47 numerical evidence remains unchanged: 7,308 complete
values and 2,052 independent formal-polynomial/canonicality certificates, with
its focused clean Memcheck. Those results qualify the stated prior corpus only,
not this entire expression layer. The new verifier checks recorded read coverage,
source identity and the existing evidence bindings; it does not rerun tests or
the full historical verification chain.

Decision: keep Hyper's shared exact graph, bounded retained approximation state
and explicit certification boundary. Flat syntax storage and temporary borrowed
views are optional future representation ideas, not retained scalar changes.
No plausible new implementation candidate warrants a matched benchmark yet.
No new `/tmp` files or build outputs are needed for this checkpoint.
