# Live Hyper boundary capability check

Bind a small audit-only crate and its lock, driver and unchanged 957-file live
source map before compiling. Reuse the existing shared target; no source-tree
copy or production edit. Run locked offline debug/release, formatting,
all-target/all-feature warning-denied Clippy and full dependency metadata.

Emit 19,340 ordered rows plus terminal:

- 18,432 exact float imports matching the independent binary32/binary64 corpus.
  Print canonical signed numerator/positive denominator and the borrowed f64
  export bits. Finite inputs must remain exact rational; f32 widening and signed
  zero cache behavior are checked independently. Infinity and NaN retain distinct
  explicit errors; no failed value is consumed.
- 322 real rounding rows: seven origins, 23 exact rational/quadratic offsets and
  two repeated queries on the same expression. Check certified floor/ceiling
  values against exact Q(sqrt(2)) inequalities. Exhausted is permitted and remains
  visible; near_integer must always be one of the mathematically adjacent integers.
  Repeated-query phase is not claimed to match explicit donor enclosure polishing.
- 576 derived reciprocal inverse rows at predicate floors -64/-256, plus ten
  real controls. acos(1/x)/asin(1/x) are compositions, not claimed Hyper asec/acsc
  APIs. Pole/domain errors must be explicit. Unknown is not inequality.

Compare all ordered debug/release records including certificates and error
variants. Reject deliberate stream/value corruptions. The native donor corpus
also includes complex inputs and numerator/height operations; this Hyper probe
does not imply those belong to the real scalar API. Neither executable is a
matched benchmark. A possible two-candidate certified-rounding design remains
unimplemented and unqualified; do not report performance or retention from this
source/capability check.
