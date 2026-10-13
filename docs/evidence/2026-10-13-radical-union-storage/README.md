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
are unchanged. Qualification completed in all three fresh targets. Each source recorded
its actual layouts and passed the eight shared storage controls,
six cache-frontier controls, eleven P4 regressions, all-feature/default scalar
libraries, doctests, formatting, strict Clippy and warning-denied docs. Actual
dependency feature sets are bound to the builds: baseline uses no SmallVec
features, and both union variants use exactly `union` for these scalar checks.
Each source passes 882 all-feature and 795 default scalar library tests, 28
doctests, the shared and inherited controls, formatting, strict Clippy and docs.
Configuration and focused counts overlap. See [correctness-v1.json](correctness-v1.json).

All seven recorded layouts match across the three sources: Rational 8 bytes,
coefficient 56, polynomial 24, persistent coefficient 32, persistent polynomial
24, Computable 16 and Node 56. The radical candidate avoids the previous extra
eight coefficient bytes. These equal type sizes establish neither equal total
memory nor faster execution.

The prepared fixed comparison runs 18 preflights and 90 measurement processes
in five predeclared three-source blocks, with separate timing/allocation
executables and fresh targets. All builds finish before measurements. It will assess the radical candidate
against both the unchanged baseline and the feature-only control, and assess
the feature-only control against baseline. Supported variants still require
native qualification, fixed endpoint/broad/abort controls, complete report
identity, production CLI sizes and the original full-fixture assertions.

The native parent remains unadmitted: its original board assertion fails at
133.31248412 seconds of worker CPU against 120; its separate 60-second neck
check-body assertion passes. Those are parent results. All 35 pending native
entries, all 39 properties, 14 targets and crate-specific future work remain in
scope.
