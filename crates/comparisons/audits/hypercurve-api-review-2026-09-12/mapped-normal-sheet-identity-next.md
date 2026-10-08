# Remaining mapped-point normal-sheet identity

The implementation goal remains active. Hypercurve `8977b811b66efcaeee00c9d8005f97ca6d788e28` contains the independently qualified projective transport. Hypersolve `3d900ee` contains the separately qualified rational-unit determinant reduction. The unfinished mapped-point implementation stays in `hypercurve/src/bezier_offset.rs`; its required analytic normal-sheet regression remains unchanged.

Let `a = sqrt(1/2)`, `b = sqrt(a)`, and `N = 1/sqrt(1+4a)`. The compact source stores `A²=1/2`, the positive selected `A`, and the positive selected fiber `u²=A`, without a represented `u` value. An authored exact rational line supplies the point

`Q(u) = ((1-2N)u, a+N)`.

The target is the left offset at distance one of `P(t)=(t,t²)`, on `[1/2,1]`. At `t=b`, the displacement is `(-2Nb,N)`. Its dot product with `(1,2b)` is zero, its squared length is `N²(4a+1)=1`, and its left-normal sign is positive. The target is regular and finite throughout the consumed range. Thus this exact point has an exact representable inverse. The regression also requires translated parameter charts `t=b+h` for `h=±2`, opposite-sheet rejection, and both policies. It currently stops at the first positive-sheet query; those later cases have not passed under the consolidated implementation.

The generic inverse isolates the correct projected root and certifies its normal sign. The second incidence fails when the retained-field resultant reduces to `r0 + r1*A`: both coefficient signs are known, but the sign of its value at the selected `A` remains undecided. This is a proof failure, not a missing candidate or a numerical tolerance issue.

The public probe `fiber-unit-pivots-identity-probe.rs` reproduces this algebra through Hypersolve alone. Both the parent and new determinant implementation report an undecided selected-root sign and an undecided direct evaluation at the exact scalar witness `a`. The factored identity `(4a+1)N²-1` certifies zero. The new determinant reduces construction work (three fresh-process median times: 11.552548 ms to 0.7321 ms), but does not close the equality proof. Artifact-bound logs are `fiber-unit-pivots-identity-{parent,candidate}.json`, and the timing records are in `fiber-unit-pivots-identity-timings.json`.

All these probes use pinned Hyperreal `a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef`. They do not assess the other session's uncommitted verification work. That working tree has not been changed or used as a build input.

The next implementation must preserve or reconstruct the algebraic identity through coefficient expansion, or retain a reusable factored incidence proof through candidate replay. Keep scalar meaning in Hyperreal and algebraic replay in Hypersolve. A geometric special case, approximate zero, dropping the negative-sheet check, or skipping the required regression does not satisfy this obligation. Prior attempts to materialize the source projection or linearize the entire base field did not resolve it and sometimes caused another regression to time out; their sources and binaries are archived. Do not repeat them without a narrower proof and new evidence.

Other required work remains independent: unsupported point kinds, source-cusp frames, normalized public-region admission, corner degeneracies, the five known broad failures and eight previous expensive exclusions, and the wider algebraic/representation consolidation. Neither this numeric diagnosis nor either new commit establishes full closure.
