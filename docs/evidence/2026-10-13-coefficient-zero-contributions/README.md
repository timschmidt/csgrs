The original coefficient multiplier executes 9,488,190 linear radical products
with an already stored zero rational scale on the complete boards. That is
32.310% of all 29,365,695 executed linear contributions. Neither opaque loop
executes a zero-scale product.

| Existing loop | Executed entries | Zero-scale entries | Zero share |
| --- | ---: | ---: | ---: |
| Left radical | 10,673,994 | 4,721,952 | 44.238% |
| Right radical | 17,980,956 | 4,766,238 | 26.507% |
| Left opaque atom | 76,218 | 0 | 0% |
| Right opaque atom | 634,527 | 0 | 0% |

[The checked assessment](assessment-v1.json) preserves all 64 shape/zero bins
for 19,938,583 calls that pass the original unsupported-product guard. Later
insertion may still decline. Counters at the four original loop bodies measure
actual execution; input lengths are not used as substitutes. Their sums happen
to match on these boards, while a separate regression demonstrates an early
decline after only one of three right radical entries.

Four calibration/site tests, six retained-cache regressions and five endpoint
regressions pass. Formatting and strict release, all-feature, all-target Clippy
pass for scalar, curve and DRC crates. The diagnostic preserves every original
arithmetic operation, early return, ownership path and cache pass. Its separate
source and executable, all inputs and all 35 pending main entries are hashed.

Both complete boards emit their reports with 12/197 findings. The original
worker-CPU assertion still fails at **133.052481708 seconds / 120**. Reporting
runs after the assertion, including on unwind. Synchronization changes the
instrumented cost, and these counts do not predict time or allocation savings.
Finding counts are not complete report-byte identity.

The preregistered screen of 100,000 zero products and 5% of linear entries passes.
[The next experiment](decision-v1.json) guards only the two radical linear loops
when their opposite rational scale is zero. Rational multiplication already
returns early on zero, before canonicalization or product-cache work; the
candidate would avoid clone/drop pairs and shared-zero temporaries. Both opaque
loops and all nonzero arithmetic stay unchanged. Two additional sign branches
could regress other shapes, so fixed scalar/native costs and the original gates
remain necessary. No runtime change is admitted; all endpoint tradeoffs and
the broader properties/future-work scope remain open.
