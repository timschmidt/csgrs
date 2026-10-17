# Temporary test storage and report recovery

The file-output tests now honor the platform temporary-directory selection.
Conversion success and directory-error cases also use distinct input directory
names, so concurrent tests do not remove one another's fixture.

The current test bodies reproduced five failures while a writable TMPDIR was
selected; they wrote to hard-coded /tmp paths instead. The repair changes only
test paths, their cleanup and two unused test imports. Production prefixes,
assertions, fixture contents, converter arguments and report readback remain
unchanged. The converter stub still uses /bin/true; broader operating-system
support is not claimed.

All 18 conversion tests pass with four test threads, and the Parquet and SQLite
readback tests pass. The selected directory is clean afterward. Strict Clippy
passes for all HyperDRC features and targets, and formatting passes. Source-matched
main dependencies were reused read-only; the cache index was not included in this
test-only repair. Two dependency-selection attempts and a missing Cargo metadata
environment are preserved separately from the successful final test build.

The previously successful E38 native build already contained a complete CLI.
Using that pinned executable, complete CPArti and HVP109A reports were compared
with the verified baseline at the same original project paths. JSON values,
ordering, serialized bytes, sizes and checksums are identical: 12 and 201 findings,
respectively. This closes that scoped report check, not cache admission, source
fidelity, a performance claim or the full user-path contract.

The cache proposal remains unadmitted. Its earlier quota failures remain failures;
full affected-crate and binding validation and matched cost evidence are still
required. The per-user /tmp quota remains full. A small fresh lint target fit in
repository storage with a monitored 768 MiB reserve; no completed target was
reopened. The original 35-file disposition and rejected exact-track integration
remain intact, and all broader plan obligations stay active. The inherited artifact
audit still has 27,093 missing paths.
