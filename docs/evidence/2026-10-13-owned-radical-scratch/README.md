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
shared allocation. The measured scratch coefficient remains 56 bytes, while
the closed coefficient shrinks from 32 to 16 bytes.
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
Version 1 completed 23 successful commands before warning-denied Clippy rejected
`mem::replace(self, Self::new())` in the helper. Its tests passed, including
890 all-feature and 803 default candidate library tests, but qualification is
incomplete: candidate default Clippy and docs did not execute. The original
source and failure remain frozen in
[qualification-failure-v1.json](qualification-failure-v1.json).

Version 2 changes only that expression to `std::mem::take(self)`. The existing
Default implementation already delegates directly to `Self::new`; exact byte
comparison binds this correction. Preparation froze fresh v2 source trees with
the same 10,134/10,135 file counts and exact parent-token recovery. All 26
commands passed in fresh targets. No v1 cost probe executed, and no v1 pass is
relabelled as a v2 execution. The correction and preserved failure are bound in
[preparation-v2/protocol.json](preparation-v2/protocol.json).

The independent [qualification assessment](correctness-v2.json) verifies all
commands, logs, source bindings and inventories. Baseline passes 888 all-feature
and 801 default scalar library tests; candidate passes 890 and 803, including
two additional buffer-ownership controls. Both pass 28 doctests, shared storage,
memo and alias controls, inherited cache and P4 controls, formatting, strict
default/all-feature/all-target Clippy and warning-denied docs. Counts overlap;
these are scoped checks, not full-workflow coverage. The other five recorded
type layouts are unchanged. Both sources retain the original local memo
entries, while only candidate reads share the immutable retained array.

The next fixed three-suite cost comparison uses twelve preflights and 60
measurement processes, with separate timing/allocation executables and all
builds before probes. Every original phase and sample stays in scope. No cost
claim follows from the smaller closed coefficient. See
[scope-v2.json](scope-v2.json).

The eight-term/eight-radical and original work/depth limits remain intact, as
does the mandatory original retry after releasing its first memo. No manifest,
lockfile, approximation, public API or main runtime change is proposed. All 35
pending native entries, both original native limits, native controls, complete
reports and production size remain requirements before runtime admission.
All 39 properties, 14 targets and crate-specific future work remain in scope.
