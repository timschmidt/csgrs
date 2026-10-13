This prepares a public-API observation for the open P4 question: whether
`compact_quadratic_tower` preserves an already available exact sign when it
rebuilds an equal value. The source currently reconstructs the tower and copies
the abort signal without explicitly transferring the source's cached sign.
That observation alone establishes no behavioral defect; reconstruction may
prove the same sign independently.

Version 1 built both scalar libraries, then failed its first probe compile:
`probe.rs` has an unmatched closing brace in its JSON output format string.
No observation executed. `failure-v1.json` binds the original inputs and all
logs; those files remain unchanged. `probe_v2.rs` adds exactly one closing
escape brace and is prepared but **has not been compiled or executed**.
The version 2 runner and assessor retain the same fixed matrix and use fresh
build targets and output files.

The probe uses three
fixed families (`sqrt(2)`, `sqrt(3 + sqrt(5))`, and `sqrt(2) + sqrt(3)`), linear
and squared differences from a certified 1,024-bit lower endpoint, and scales
of 1 and -7. Each of the twelve cases must run in a fresh process. It records
the original low-precision fact, a 1,100-bit proof, original/cloned low-precision
facts, and immediate/low-precision facts after compaction, cloning and repeated
compaction. An exact equality query runs last. Unknown and declined compaction
remain observable outcomes; no source change or contradiction is presumed.

The prepared runner binds the exact source, all-feature
scalar library, manifest, target, compiler and probe bytes. Compare current
main and the unchanged endpoint runtime baseline with separate targets; record
their scopes separately. Preserve all 35 pending native entries and existing
frozen experiment sources. Run only after the current native validation and
cost/report processes have finished. No concurrent build, test or measurement.

Any observed loss requires a focused regression and source review before a
runtime experiment. This probe is neither a universal cache proof nor a
performance benchmark, and it does not qualify runtime retention.
