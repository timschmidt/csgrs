# One selected-circle point containment predicate

Parent Hypercurve: c13778eb15662d3b12093a54e518e9fb137c39d1.

The independent-field corner V1 regression replaces the obsolete loop-zero
fragment-count contract with exact set checks. The chamfer passes, including
its independent contact midpoint and added-lobe centroid. In the first STRICT
fillet, the independently derived circle extreme returns Uncertain(Ordering)
instead of Boundary. The two exterior contacts, original material samples and
both original axis-edge samples pass before this query.

V1 source and binary bindings are in independent-field-corners-20260924-v1-*.
The first debugger command used an unavailable panic symbol and let the test
exit with its original assertion; that inferior and debugger were reaped.
The second stopped at the confirmed __rustc::rust_panic symbol and killed and
reaped its own inferior. Arguments and recursive payloads were not printed.
Return-address disassembly identifies the seventh location call (connector)
and second construction call (fillet), without changing the frozen test.

BezierAlgebraicCuspSemicircleFragment2 had two point-containment entry points.
The Point2 path interpreted an internal ray-winding Boundary uncertainty as
incidence and inherited unrelated angular Ordering uncertainty. The general
CurvePoint2 predicate already signs supporting-circle incidence and determines
finite-arc ownership using the endpoint chord, with retained parameter proofs
for mapped contacts. V2 removes the Point2 implementation and makes this
existing general implementation the sole contains_point method. All callers,
including represented points, are migrated; the previous _evidence entry point
is removed. No forwarding method, new representation or cache is added.

The endpoint-chord theorem applies to these retained semicircle fragments;
major fillet arcs are published through their existing half-circle fragments.
Both traversal direction and semicircle direction remain in the ownership
predicate. Circle nonincidence is false, exact endpoint incidence is included,
and an unresolved proof remains uncertainty. The new regression retains both
policies and both edited vertices under reversal and names each sample on
failure. The algebraic point oracle is derived from the independent triangle
and exact radius, not from the output's circle coefficients.

V2 initially missed three test callers; its compile errors were recorded and
all processes reaped before those callers were migrated. V3 passes both Clippy
configurations but still fails the same independently derived fillet boundary
sample. API unification alone is therefore not a complete fix.

The separately frozen V4 debug diagnostic prints only bounded classifications
and carrier labels. The original triangle's loop returns Outside. In the added
lobe, both chords return false for incidence, while both selected-circle
fragments return Uncertain(Ordering). All owned V4 processes are reaped and its
2,044 source and executable bindings verified. This rules out an early uncertain
loop masking a later boundary result in this case. Direct selected-circle
incidence has no general exact replay beyond independent coordinate refinement.

V5 removes the temporary diagnostic code. After existing construction-identity
fast paths and short exact enclosure checks, the circle predicate imports the
query and center through the existing least-shared recursive point authority.
For homogeneous points it forms dx^2+dy^2-r^2*d^2, where d is their common
nonzero denominator. Its square preserves the affine residual sign even for
negative projective weights. Shared selected-root and positive-radical relations
remain available to prove zero; no coordinate projection or new field carrier
is introduced. The existing complete refinement fallback remains available
when exact replay is unavailable. This candidate is not yet qualified.

V5's physical snapshot binds 2,044 inputs and four changed files. Formatting,
both Clippy configurations and all ten focused cases pass. The independent-field
normalized-set regression completes in 0.72 library seconds, covering both
policies and both edited vertices under reversal. Its previously uncertain
fillet extreme is now certified Boundary. The full library and public integration
qualification is running against the same frozen sources and library executable.
No failed regression is hidden by weakening its geometric expectation.

The first full-run integration build stopped before any integration test:
rustc could not create its Hypercurve archive because `/tmp` exceeded its
quota. Its compiler diagnostics and terminal receipt are retained unchanged
under the original `-full-` prefix. All subprocesses were reaped and all input
bindings verified before reclaiming 1,785 MiB of rebuildable Cargo debug cache.
The separately hashed V4 diagnostic executable remains intact in this audit.
The new `-full-resumed-` driver and receipts resume from the same frozen V5
sources and release library executable; no candidate source bytes changed.

Final V5 qualification passes: 1,346 of 1,361 cases pass, six are ignored,
and nine unchanged nonpasses remain (eight 75-second timeouts and the separate
parabola corner's obsolete boundary-layout assertion). All 121 public integration
cases pass. The three slow passing compositions run in isolation with unchanged
limits; all other cases use two workers. Both all-target Clippy configurations
and formatting pass. Every one of the 2,044 input bindings, copied executable,
index and commit bindings is verified. All owned processes are reaped.

Committed as Hypercurve 91ddad8970b8ffda7053a38bce3e72a0400d7abe. All thirty
repositories are clean after commit; nothing was pushed. Qualification,
staged and post-commit receipts use the independent-field-corners-20260924-v5
prefix; the successful complete run uses its full-resumed suffix. The full
implementation goal remains active.
