# Empty product acceptance repair

During the inverse-isometric extension's broad scalar validation, the existing
`determinant_product_cancellation::other_arities_keep_their_mathematical_values`
test panicked at `debug_assert!(FACTORS > 0)`. An isolated source archive of
`33dd55ccc`, using its committed lockfile and the same stable compiler,
reproduced the failure: [baseline output](baseline.log). This predates the
inverse extension. No fixture or policy was removed or weakened.

Owner: Rational aggregate products, consumed by Real's ordinary and active
sum-of-products entry points. P1, P6 and P18 require the empty product to equal
one, the empty sum to equal zero, and exact sign queries to agree. The declared
fixed-size input domain includes zero factors and zero terms. The repair handles
zero factors before denominator/factor schedules; ordinary positive arities
retain their existing schedules. All three Rational entry points share the
mathematical convention. Counts become exact integers without a signed machine
integer overflow.

The unchanged failing test now passes. Added checks cover positive/negative and
mixed term signs, a cancelling pair, zero terms, shared-denominator evaluation,
ordering, and both Real wrappers. [Focused output](focused.log) records all nine
tests passing with all features and Clippy `-D warnings` on stable Rust 1.99.0.
`cargo test --locked -p hyperlattice -p hyperlimit --lib --all-features` also
passed; the exact test totals and command output are retained in
[downstream output](downstream.log).
This is a correctness repair with no performance claim. Broader native-owner
and downstream validation is recorded with the inverse extension's admission.
No push or publication is part of this change.
