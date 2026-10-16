# Lossless referenced artifact audit

The repository filesystem has less than 150 MiB available. Repeating the full
inherited audit (435,075 expected paths, 36 MB compressed before this work)
would consume that space again without adding evidence. This milestone uses a
version 2 audit: an exact hash-pinned reference to the preceding audit plus a
map of new paths and hashes. The full expected map is their disjoint union.
No expectation is removed, replaced or weakened.

Closure still verifies every inherited and added available path, checks every
immutable overlay lower, and requires the missing set to equal the preceding
27,093 missing paths exactly. The reference preserves those original expected
hashes. Counts report the full reconstructed audit, not only the new entries.
The parent checkpoint and all its evidence files are independently pinned too.

`load_artifact_audit_v1.py` reconstructs either the old complete-map format or
this referenced format and verifies parent hashes. This is a storage change,
not recovery of the historical missing artifacts or a substitute for hashing
the available files at closure.
