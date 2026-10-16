# Fixed completed Boolean cost probes

Freeze `resource_probe_v1.rs` before production edits. Its native strict union
controls are two overlapping side-two cubes (volume 12), three coincident
side-two cubes (volume 8), and two face-touching side-two cubes (volume 16).
Check output closed-PWN topology, exact volume and analytic inside/outside
witnesses independently of timing. Preserve each control's ordered artifact;
any intentional artifact change must be separately identified and justified.

Timing includes input construction and complete native output materialization.
Validation and serialization replay follow the measured construction. Cold uses
a fresh process; warm repeats the complete construction once in that process,
not a cold-filesystem experiment. Use three timing children and one separate
allocation child per case and temperature. Retain all raw rows, failed attempts,
resource reports and artifact hashes. The requested Rust allocator measures
allocation/reallocation and live/peak deltas, excluding native allocator
internals; OS peak RSS is a separate measurement.

The same driver includes full 26-operand and merged-shell fixtures for eventual
candidate-only completion measurements (volume 124 and an empty central unit
cavity). Their current failures are retained in the baseline diagnostic; an
earlier error cannot be a successful cost baseline. Later comparisons of the
certified controls must use pinned matching library features, profiles, toolchain
and dependencies, preserving the original pre-edit rows separately.

Each measurement child has a 120-second wall limit and inherits external
8-GiB address-space containment. These limits do not establish internal total
work, cache-retention or heap bounds. This is a correctness/completeness repair,
not an optimization claim; unfavorable costs remain part of admission.
