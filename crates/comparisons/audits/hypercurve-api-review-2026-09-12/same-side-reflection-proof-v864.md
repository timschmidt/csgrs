# Same-side branch order and reflection

For x>0, let two regular graph germs be (x,k1*x^n), (x,k2*x^n), n=2 or 3 and 0<k1<k2. Their oriented determinant at the common x is (k2-k1)*x^(n+1)>0. The first ray precedes the second in the upper half-plane. Reflecting y changes the determinant sign and puts both rays in the lower half-plane, reversing their counter-clockwise order. Their absolute curvature or cubic magnitude order is unchanged, so deciding from squared magnitudes alone loses this orientation.

V863's public probe supplies exact rational derivative jets with both policies, both orders, positive/negative sides and swapped inputs. All sixteen requests complete; the eight negative-side cases produce the wrong decided ordering. V865 replays the identical source and requires zero mismatches.

The native implementation now reverses its magnitude result when the already-decided first side sign is negative. Both native callers reach this branch only for equal nonzero side signs. The represented shared magnitude comparator applies the same reversal using its retained first determinant sign. No arithmetic, root construction, refinement, evidence payload, zero-sign behavior or speed normalization changes. Equality and uncertainty remain unchanged under reversal. Positive-side comparisons retain their prior decisions.

Independent native and public represented tests use the graph orientation oracle, both derivative orders and both input orders under both policies. These tests complement the existing speed-invariance and zero-curvature prerequisite regressions. The change repairs local predicate semantics; universal region and composition closure remain open.
