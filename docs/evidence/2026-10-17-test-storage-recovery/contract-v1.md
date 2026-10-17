# Continue acceptance after the storage failure

The target revision is 8982f85b09740ab89309e3be9b6a970b65c50ec8.
The E38 cache prototype remains unadmitted. Its completed native-build command
produced a retained CLI before later quota failures; verify that binary and its
source equivalence, then compare complete CPArti and HVP109A project reports.
This read-only comparison can establish the report-specific P27 contract even
while broader qualification remains incomplete. It cannot admit the cache by
itself or establish analytic source fidelity.

The five native file-output failures also expose a test-harness restriction:
the conversion, Parquet and SQLite tests hard-code /tmp despite a configured
TMPDIR with available repository storage. Reproduce those failures using the
retained executable and verify that the relevant test source is identical to
the target revision before making a repair.

The owning layer for temporary test paths is the test harness, under P20,
P35, P37 and the release/validation targets. The input domain is the same
converter fixture, complete report model, and output-error cases. Use the
platform temporary-directory choice with test-specific filenames, preserve all
assertions, formats, report fields, input contents, conversion arguments and
cleanup, and avoid test-name collisions between concurrent cases. No production
geometry or predicate code is part of this test-harness change.

Acceptance requires the original affected tests under an explicitly selected
writable temporary directory, error-path and cleanup checks, strict Clippy and
formatting for the affected code, and source equivalence outside test-only
paths. Do not reinterpret the original quota failures as passes. Cache
qualification, actual binding consumers and full-project timing remain separate
unfinished requirements. A fresh target may use repository storage with an
explicit free-space reserve; never reopen E38's completed targets or alter its
snapshots, runners, logs or failed files.

All broad plan obligations and the inherited 27,093 missing artifact paths
remain open. Only current_work may change in the protected execution ledger.
