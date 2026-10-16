# Retained diagnostic limitations

`baseline-cost-v1` exited 1 in the runner after the fifth timing child. Its
`result.json` is truncated to zero bytes. All existing child stdout, stderr,
resource files, build outputs and output JSON remain retained. The last child's
resource record reports exit zero; no failing construction was observed. The
runner exception text was not retained, so its cause is unknown. A subsequent
source-hash comparison found no source changes. This phase is incomplete and
is not credited as a completed cost matrix.

`baseline-cost-v2` uses the same frozen Rust harness and dependency libraries,
with a fresh target and all driver output redirected to a retained log. It
completes both builds and all sixteen fresh cost children (twelve timing and
four allocation). No failed phase is resumed or overwritten. The first phase's
available samples remain visible; the complete second matrix is the baseline.

`candidate-v1` passes formatting and all five native simple-ring regressions.
The root test target fails to compile because the new harness used nonexistent
`TriangleMesh::default()`. No production failure is inferred from that harness
error. `candidate-v2` uses the public empty-buffer constructor and passes all
sixteen focused root tests plus native/root warning-denied Clippy. The nine new
root tests include full original README fixtures and qualified section
coalescing; the seven existing minimal-context tests remain unchanged.
