# Retained tangent evidence at Boolean junctions

Parent Hypercurve: `68bc4f1a5abae015f1908e40d4ea28a226db1502`.
Dependencies retain Hyperreal `28eaac35e8c24006c532b25a393b7d430d03823c`
and Hypersolve `d7cd3d9ffe33e6ef26c0c409dda986a88f16f7ef`.

## Reproduction and diagnosis

The existing public `homogeneous_boundary_closes_through_boolean_corners_and_offset`
test composes a homogeneous semicircle with rectangle clipping, chamfer/fillet,
outward round offset, and a second batch of four Booleans. It covers the authored
rational quadratic, independently elevated rational degree twelve, and two NURBS
representations under both strict and approximate policies. All decisions must be
certified; the middle homogeneous control is at infinity while the chart remains
finite.

The unchanged-parent stage probe retained all original operations and assertions.
It timed out after 150 seconds on the first ordinary rational-circle case. Admission,
clipping, classification, and the first corner/offset operations completed in under
a second. Chamfer Boolean replay took about 0.65 seconds; fillet Boolean replay did
not finish. The probe's source, binary, linked library, 2,044 snapshot/main input
hashes, and terminal process status are bound in
`homogeneous-composition-20260924-probe1-terminal.json`. Earlier parent/current
90-second bounds are recorded in the direct-endpoint qualification.

The first debugger attempt could not start its inferior because the sandbox denied
ptrace. The authorized second and third captures ran and killed/reaped their own
inferiors. The outer stack localizes the work to XOR composition fallback, unary
regularization, rational parameter-map image elimination, coefficient normalization,
and a deep scalar approximation chain. No recursive geometry or scalar values were
dumped; stack frames suppress arguments.

An isolated diagnostic snapshot prints only graph indices, direction flags,
locations, contact signs, and traversal reasons. It stops at the first XOR fallback
instead of changing the decision. This reveals an unresolved degree-four junction
on the chamfer replay: two adjacent source carriers meet the rectangle carrier at
one vertex. The retained pair contact vocabulary does not name all incident
carriers, and coordinate-derivative traversal returns Boundary uncertainty. The
diagnostic is not a correctness candidate. Its immutable inputs and terminal result
are in `homogeneous-composition-20260924-trace1-{sources,terminal}.json`.

## Candidate and exactness argument

`certified_boolean_successors` reuses `certify_curve_tangent_successors`, the same
retained-curve direction authority already used by unary regularization. It runs
only at an unresolved junction with multiple outgoing fragments, under a strict
predicate pass. The existing helper requires balanced incoming/outgoing incidence,
decided exact angular comparisons, and unique unclaimed successor targets. Equal
tangents and unavailable evidence remain unresolved; no tolerance, guessed branch,
or coordinate-materialization requirement is introduced. Existing contact and
authored-continuation certificates retain precedence.

The region composition test now also checks every Boolean result against five
independent exact sample expectations: all four operand membership combinations
and clipping-boundary ownership. Its original family, policy, candidate, connectivity,
and classification assertions remain in place. The first production candidate
passes the expanded test in 0.40 seconds (0.55 seconds including process startup).
This is a bounded workload comparison, not a general performance claim.

The second candidate avoids allocating incoming-vertex groups for unbranched
junctions. Clippy also exposed leftover identity conversions, needless borrows,
redundant wildcard patterns/closures, and test-only lints from earlier migrations.
Those callers are being simplified directly. The curve-support test module is
moved after production items; a byte comparison confirms this is exactly a module
relocation. No compatibility API or mathematical representation has been added.

## Qualification status

v1's focused public regression passes. Its all-target Clippy attempt found 55
pre-existing diagnostics and was reaped before any edit. v2 fixes those and reaches
nine additional integration-test/benchmark diagnostics. v3 includes their direct
cleanup. All frozen snapshot versions remain unchanged.

The final v3 snapshot passes all-target Clippy with warnings denied under both
all-features and no-default-features configurations, plus formatting checks on
every modified Rust file. The individual-case release sweep attempts 1,348 cases:
1,207 library passes, six ignored library tests, the same fourteen existing library
nonpasses (eleven 75-second bounds and three representation/layout assertions),
and all 121 public integration cases passing. This includes all twelve path-closure
cases, all 37 Boolean cases, all 57 arrangement cases, all eleven Boolean fuzz/corpus
cases, and all four PCB regressions including both large board-sized fixtures.
No new nonpass appears. The expanded homogeneous composition case passes in
0.465 seconds including process startup.

The original stage-only probe source is also replayed unchanged against the final
release library. It completes all four representations and both policies in
0.449 seconds including process startup (about 121 ms inside the probe), whereas
the parent exceeds 150 seconds during its first fillet replay. The first fillet
replay itself takes about 3 ms in this run. This is a bounded workload result,
not a universal speedup. Source, library, executable, and input bindings are in
`homogeneous-composition-20260924-final-probe-terminal.json`.

Hypercurve `5af1dfe` commits the shared tangent proof and expanded regression;
`07cb100` commits the remaining migration cleanup. Staged bytes and final HEAD
match the qualified snapshot. The test-module relocation is byte-for-byte, and
the cleanup removes 26 net lines. All owned processes were reaped before edits
or commits; all thirty workspace Git repositories are clean after the two commits.
Nothing was pushed. The full API/four-closure goal remains active, including the
fourteen library nonpasses and the separate root-certificate construction debt.
