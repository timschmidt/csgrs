This experiment keeps temporary radical arrays in ordinary owned Vecs and shares
only immutable retained arrays. It starts from the frozen endpoint/P4 parent,
without the rejected shared-scratch, memo-publication or inline-storage changes.
The previous shared-scratch version added allocation calls in thirty scalar
phases and slowed conjugate-then-affine first, warm and cloned queries in every
pair. Its source and samples remain frozen.

New scratch arrays and their clones keep Vec ownership. Publishing an owned
array copies it into a retained Arc; reading that payload creates a shared view.
The first mutable access turns the view into an ordinary Vec by unwrapping a
unique buffer or copying a shared one. Subsequent writes keep that owned buffer.
An empty retained array has no allocation. Closed storage exposes no mutable
access and owns only Rational pairs; opaque Computable owners and polynomial
term Vecs keep their original behavior. Publication eligibility, cache locks,
arithmetic and insertion order are unchanged.

This may reduce reconstruction on repeated reads, but publication still adds a
shared allocation, and the two-form scratch enum may enlarge coefficients.
The private unreachable arm follows an exhaustive conversion under exclusive
access; it is not a numeric-domain failure path. Source review covers mutation,
consuming iteration, empty states, republishing, unique-buffer transfer and
release. These are implementation arguments, not measured performance claims.

The shared alias tests retain every mathematical assertion, concurrent mutation
check and graph-release check. Only their pointer inventory changes: it reports
whether the original scratch shares the published payload and whether returned
views share it. Baseline should record 0/0 and candidate 0/1. Exact token
recovery verifies that this observation is the sole test edit. Two additional
candidate controls check owned cloning, publication, detachment, republishing,
consuming iteration and buffer reclamation. The same eight storage and three
memo controls run in both sources, along with inherited cache and P4 controls.

Preparation froze a 10,134-file shared-test baseline and a 10,135-file
candidate. They differ only in algebra.rs and the private storage helper. Exact
parent-token recovery and unchanged mathematical alias controls are verified;
all manifests, lockfiles, main runtime and pending native entries are unchanged.
Qualification is running serially from fresh targets through default/all-feature
scalar suites, doctests, formatting, strict Clippy and warning-denied docs. Qualification records every layout rather than assuming
the scratch enum's size. If it passes, the fixed three-suite comparison uses
twelve preflights and 60 measurement processes, with separate timing/allocation
executables and all builds before probes. Every original phase and sample stays
in scope. See [scope-v1.json](scope-v1.json).

The eight-term/eight-radical and original work/depth limits remain intact, as
does the mandatory original retry after releasing its first memo. No manifest,
lockfile, approximation, public API or main runtime change is proposed. All 35
pending native entries, both original native limits, native controls, complete
reports and production size remain requirements before runtime admission.
All 39 properties, 14 targets and crate-specific future work remain in scope.
