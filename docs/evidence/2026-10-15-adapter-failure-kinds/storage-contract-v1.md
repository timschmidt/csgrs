# Final validation storage

`candidate-v2` stopped during build preparation: copying the completed release
directory exceeded the `/tmp` user quota. The filesystem still reported about
13 GiB free, but the user had reached its 51,490 MiB quota. No validation command
ran. Its preparation record, complete copy diagnostics and partial target are
retained; this attempt is not a failed test or a passing qualification.

The final test-helper edit only replaces `.err().expect()` with `expect_err()`.
`candidate-v3` runs the same five final checks after an explicit storage setup
command. It uses the installed `fuse-overlayfs` with the preserved candidate-v1
build as its read-only lower directory. A private upper/work pair under
`/run/user/1000/csgrs-adapter-failure-kinds/candidate-v3` receives every new or
changed artifact. This avoids another multi-gigabyte build copy; it does not
reuse mutable hard links or delete previous artifacts.

Before mounting, the setup records every lower file's SHA-256. It verifies copy
up by writing an identical byte to a disposable view of an existing consumer
binary, checking the private upper file and the unchanged lower inode, mtime
and content. Closure must verify all lower hashes again and retain the overlay
mount specification, physical upper files and final artifact view. The mount
must remain available while its view is part of the retained evidence.

The frozen runner's ordinary copy seed is null for this phase; its first command
and `overlay.json` describe the copy-on-write preparation instead. The source,
tests, feature combinations, warning policy and 1,800-second/8-GiB external
containment remain unchanged. These are command/per-process limits, not a new
internal query budget or heap-byte guarantee.
