This experiment shares rational/radical arrays across temporary and persistent
Laurent certificates. The frozen endpoint/P4 parent reconstructs each array on
a persistent read. Earlier diagnostics counted 35,239,829 recursive certificate
copies, including 30,843,135 single-radical copies. Those are coverage counts,
not allocations, time shares or predicted savings.

The candidate uses an optional Arc of a Vec for radical pairs. Empty storage
needs no allocation. Reading or cloning a nonempty array shares it; mutable
access first obtains a private copy, and consuming iteration unwraps a unique
buffer or copies a shared one. Only Rational values enter this storage. Opaque
Computable owners and polynomial terms keep their ordinary Vec behavior, so
their graph-sharing decisions remain intact. No unsafe code or new cache lock
is introduced. The earlier rejected memo-publication change is not included.

This changes reference-count traffic for Rational data. The review covers its
ownership-sensitive negation, inverse and extraction paths; the proposed
storage must obtain owned entries before any consuming or mutable operation.
New alias controls check live closed certificates through scaling, cancellation,
term-frontier failure, products, inverses and concurrent query mutation. Query
views must remain exact after the expression graph is reclaimed. A candidate
helper control checks empty, unique, shared and detached buffer reclamation.
The same eight storage and three publication/memo controls run on both sources,
along with the inherited P4 and cache-frontier tests.

New buffers pay for an additional shared allocation, and mutations can copy.
Reference counting and retained capacity may offset read savings. Qualification
will record the layouts and pointer-sharing inventory, followed by the fixed
60-process scalar comparison described in [scope-v1.json](scope-v1.json).
Construction, first, warm, cloned and opaque controls retain every sample.
No smaller-layout or reduced-allocation result can excuse a significant
performance regression.

Preparation and qualification inputs are ready but have not executed. Both
variants will use fresh sources and targets. All arithmetic, insertion order,
eight-term/eight-radical bounds, original work/depth limits and the mandatory
original retry remain intact, including release of the first query memo.
Manifests, lockfiles, the main runtime and the 35 pending native entries stay
unchanged. Native qualification, both original full-board limits, endpoint and
broad controls, abort receipts, complete reports and production sizes remain
required before any runtime admission. All 39 properties, 14 targets and
crate-specific future work remain in scope.
