# Remaining native test-storage failures

Owning layer: HyperDRC test fixture setup and format-output test boundaries.
Target revision: 0ed5a970cf0d9204e9f4c5c73e93b16a6a13bc89.
Relevant properties: P6, P26 and P35–P39.

The complete native replay reproduced five additional quota failures: four
app input-discovery/metadata tests and the Arrow round-trip test hardcode /tmp.
They ignore the writable TMPDIR selected by the resource contract. Reproduce
these unchanged test bodies using the independently built current-main harness
before editing them; the cache proposal is not required for the failure.

Change only those five test path constructors, and any test-only import made
unused by that change, to use std::env::temp_dir(). Keep fixture bytes, source
and manifest assertions, Arrow schema/row checks, parse-error checks and cleanup
intact. Production implementation must be byte-identical before the test module.
Do not replace the failed result with an inferred pass.

Qualify the exact two edited source files independently of the cache proposal:
run the affected app module and Arrow test with selected writable storage,
verify cleanup, and run strict all-feature/all-target HyperDRC Clippy in a fresh
target. No unchanged test is weakened or removed.

Then compile the indexed native harness with the same test-only repair and
replay the whole native library after the all-feature curve run and Clippy have
finished. Keep the original three workload gates and containment. Reuse the
unchanged integration and documentation results only with source/dependency
equivalence; report the original five failures alongside the recovery.

All failed scratch artifacts and completed build paths remain retained. This
small test repair can be admitted independently; the cache and rejected native
geometry proposal still need their remaining acceptance evidence.
