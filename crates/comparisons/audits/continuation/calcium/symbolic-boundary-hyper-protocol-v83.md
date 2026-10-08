# Checkpoint 83 — Hyper comparison and diagnostic addendum

Use current Hyper's public operations on the same 112 unique large-angle cases
and 84 explicit/absent-Pi controls. Destination sentinels are a C mutation concern,
so are not duplicated in this owned Rust API. Test two precision policies (-128,
-256). Export exact rational enclosure endpoints; independently verify ordering,
width and containment in the algebraic field or exact Taylor bounds. A pole may
produce an explicit error; a finite computable radian value must not be confused
with its Pi-scaled counterpart. Complex exponential on imaginary arguments is
formed from cosine/sine, not claimed as a separate Hyper complex-exp API.

Additionally test the norm_squared and square-root magnitude of the same 96
scaled twelfth-angle complex inputs, at both policies, and collect bounded norm
equality certificates. Report Unknown as a capability outcome, not a false value.
Require identical debug/release records and reject corrupted oracle records.
Reuse the shared Cargo target and offline lock; no broad source copy or rebuild.
This is a capability check, not an A/B benchmark or new full-stack regression.

The original C diagnostic protocol is frozen. A subsequent causal-control
experiment copies one 1128-line donor translation unit into audit storage and
adds only two missing fmpz_clear calls, with the public function renamed for
linkage. This is explicitly an audit-only source copy, not a live donor edit,
Hyper transfer, or retrospective claim that the original no-copy plan held.
An isolated abs(-7/3 * exp(pi*i/12)) diagnostic reuses the built library and
disables core dumps. Native, Valgrind-none and Memcheck outcomes remain separate.
