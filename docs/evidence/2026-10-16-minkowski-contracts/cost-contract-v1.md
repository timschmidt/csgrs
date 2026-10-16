# Cost and artifact evidence

This is a correctness/completeness change, not an optimization. The original
cube(2)+cube(3) construction driver was frozen and run before the implementation
edits. Replay that exact driver against matched old and final libraries, including
input construction and complete output materialization in the timed interval.
Perform separate topology, exact volume 125, and ordered serialization checks.
Keep the original baseline samples and the later matched replays separately.

The scalar driver is retrospective: it was written after the two proof gaps were
captured. Compare the same frozen source against immutable old and final libraries,
without presenting this as a pre-edit performance baseline. Rational and unresolved
controls preserve their completion categories; the two captured zeros change from
Undecided to Certified and cannot support a same-completion speedup claim. Input
parsing/construction precedes the timer; the timed operation materializes a strict
plane-side decision. Warm means one earlier query on that exact retained value in
the same process. Check that every query leaves symbolic serialization intact.

New disconnected, concave and hollow construction rows have no correct old
equivalent. Record their certified completion, exact volume, independent gap/notch/
cavity witnesses and closed-PWN topology alongside cost, without a comparative
performance claim. Their timing includes input construction and output materialization;
validation and serialization replay follow the measured interval.

Use isolated child processes, three timing samples per cold/warm case and one
separate allocation run per cold/warm case. Retain all raw output, exit codes,
resource reports and artifact hashes. The allocator measures requested Rust
allocations, reallocations and live/peak changes, including query caches where
retained; it excludes native allocator internals and is not a total heap measure.
Also retain OS peak RSS and matched library/executable/source sizes. Preserve
toolchain, lock, profiles, features, dependency fingerprints and host details.
No model-cache byte bound or primitive/scalar total-work bound is implied.
