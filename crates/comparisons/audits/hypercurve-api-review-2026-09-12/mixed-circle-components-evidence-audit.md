# Mixed circle components — e99dfbd6902c

The full architecture goal remains active. This migration is a prerequisite for
nonlinear finite circle domains: clipping cannot recover isolated parameter
visits discarded by a supporting-component kernel.

At Hypercurve ba836e13a98ec5e61087072d6eaf19432170d517, the rational circle
P(v)=((1-v²)/(1+v²),2v/(1+v²)), v(t)=(t-1/4)²(t-3/4), has an isolated upper-half
visit at t=1/4 and a positive overlap on [3/4,1]. The saved baseline reports
(0 contacts, 1 overlap), rather than (1,1). Its source patch, build log, test
log, binary hash, and production HEAD are retained under
`mixed-circle-components-baseline*`.

The replacement internal result variants distinguish mapped and selected-fiber
evidence, each carrying both contacts and overlaps. They replace the four
exclusive contact/overlap variants directly. Open-curve, region, corner,
reversal, and rational-to-analytic consumers are migrated without shims.
Closed cells own their endpoint visits; independent source visits to the same
point remain distinct. Endpoint inverse replay includes those isolated visits.
The region adapters share one circle-overlap converter.

Existing single-kind tests explicitly check that the other component collection
is empty. New tests exercise mixed components, reversed source charts, both
operand orders and policies, exact endpoint inverse replay, and clipping through
the analytic component adapter. Public intersection topology splits the target
at both visits. Re-intersection of its three pieces retains one isolated contact,
two endpoint contacts, and one positive overlap respectively (with traversal
order reversed on the reversed source). These checks pass for both backends.
Final qualification has 2,252 passes (2,020 Hypercurve + 232 HyperBREP) across 49 targets, five unchanged known failures, nine ignored tests, eight existing expensive exclusions, no new failures and no timeouts. All 21 public matrices, all-target feature builds/checks, three fuzz/UI caller checks, formatting and 396 frozen source hashes pass. All 30 repositories are clean after commit e99dfbd6902caf5b1bee89617d36ceeca4165082. The full architecture goal remains active.

The first broad run had one test-migration failure: the recursive pair-radial
quarter collector previously accepted contact-only results while collecting
overlaps. Its migrated pattern incorrectly required the contact vector to be
empty. The corrected collector accepts both collections and checks that any
isolated contacts have zero tangent cross sign and endpoint location. Production
source is byte-identical before and after that correction. The complete first
run, matching source snapshot, normal library, and libtest binary are retained in
`mixed-circle-components-attempt6-qualification/`.

The first selected-fiber version exceeded the 120-second focused-test limit.
The exact attempt4 patch, focused metadata/logs, and copied libtest executable
are retained. Its executable SHA-256 is
f7ae76ff1008db5700dce5b2028ebc0ddee59eed1e193577344b57c1ed74f362.
Attempt5 removes conjugate squaring when one angular term is certified zero.
The retained center already certifies positive source speed, so this preserves
the boundary zero set without manufacturing repeated roots. The same selected
geometry and assertions, with added phase diagnostics, finish in 18.44 seconds.
Attempt6 removes those diagnostics and adds split/re-intersection checks; its
three tests pass, with the extended selected-fiber test taking 55.54 seconds.
These are observed qualification times, not a general performance benchmark.
The attempt6 pre-correction executable SHA-256 is
1dd62fc075f5492ae0a2cf303079da90d8d20e833c75f095bc7756200ac1b087.
The corrected recursive collector passes against the final libtest executable
45f799dc034081084d0545edaead54aaac2278890425b1babee7b1e274c2b640.
Final requalification reruns 28 changed executables and reuses 21 identical
executables, checking their SHA-256 hashes against the first run. Their runtime
fixtures are unchanged. The execution plan records the choice per target, and
the final qualification writer verifies every target's executable hash.

The empty second collection does not allocate. Mixed results retain existing
maps and selected roots rather than reconstructing new coefficient fields. The
result value itself carries two vector headers, and component publishers track
which isolated boundaries are already owned by cells. No memory-size reduction
is claimed.
