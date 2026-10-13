This tests a radical-only inline slot with SmallVec's `union` feature after the
previous storage variants were rejected. The previous fixed sample set stays
closed. Three fresh sources share the same eight storage tests: Vec with the
current features, Vec with `union`, and one inline radical slot with `union`.
The feature-only control is necessary because dependency features also affect
other native SmallVec consumers.

Every source starts from the uninstrumented endpoint/P4 combination. Polynomial
terms, persistent closed certificates and opaque atom vectors remain Vec. The
radical candidate's algebra code matches the earlier radical-only prototype;
the separate feature change is the new hypothesis. No package version or
checksum changes. Exact operations, ordered insertion/removal, original retry
and memo release, proof retention, owner release and all recognition limits
remain unchanged.

Preparation froze three 10,132-file sources with four changed files each.
The two Vec controls differ only in the feature flag; the radical candidate
differs from the feature control only in algebra.rs. Package versions/checksums
are unchanged. Qualification is running in fresh targets. Each source records
its actual layouts and runs the eight shared storage controls,
six cache-frontier controls, eleven P4 regressions, all-feature/default scalar
libraries, doctests, formatting, strict Clippy and warning-denied docs. Actual
dependency feature sets are bound to the builds. Equal type sizes, if observed,
will not establish equal total memory or faster execution.

A distinct fixed scalar cost comparison must assess the radical candidate
against both the unchanged baseline and the feature-only control, and assess
the feature-only control against baseline. Supported variants still require
native qualification, fixed endpoint/broad/abort controls, complete report
identity, production CLI sizes and the original full-fixture assertions.

The native parent remains unadmitted: its original board assertion fails at
133.31248412 seconds of worker CPU against 120; its separate 60-second neck
check-body assertion passes. Those are parent results. All 35 pending native
entries, all 39 properties, 14 targets and crate-specific future work remain in
scope.
