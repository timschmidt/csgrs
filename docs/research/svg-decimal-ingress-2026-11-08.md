# Exact decimal SVG path ingress — 8 November 2026

The D03 continuation starts at published main
`157c29e0601f390eb7bccc787aa6226494e450f6`. Hypercurve owns standard SVG
path-data ingress; CSGRS and the prospective comparison adapter consume its
native curves and sole filled-region carrier. The affected contracts are P1,
P6, P12–P20, P25–P27 and P35–P39. This is a correctness extension, not a
behavior-preserving refactor or a completed D03 comparison.

A new ordinary regression fails on the published production source: `.1`
becomes `3602879701896397/36028797018963968` before the native line is
constructed. The same conversion rounds Bézier controls and circular radii,
merges adjacent large integers, and turns an accepted underflowing nonzero
coordinate into zero. Selecting STRICT does not recover the original decimal.
The initial reproduction used installed stable Rust 1.98.1. The final
maintained witness also reproduces the failure against the published parent
on Rust 1.99.1 with its admitted registry versions. Qualification is explicitly
pinned to Rust 1.99.1 and MSRV 1.95.0.

The existing svgtypes parser still validates each complete command, repeated
commands, numeric grammar and compact arc flags. Its public interface exposes
floating values without token spans. A private cursor therefore retains each
already-validated numeric span and delegates its conversion to `Real`'s exact
decimal/scientific parser. Parsed floating values never supply geometric data.
Arc flags consume one character; radii remove the authored sign before scalar
conversion, preserving the existing magnitude convention without adding scalar
sign decisions before geometry predicates. There is no replacement path-command
parser, arrangement or geometry kernel. The existing order of geometric
predicates is retained. A later review reproduced a diagnostic regression in
unpublished `8d0a6d9caf`: malformed `em`/`ex` suffixes became scalar decimal
errors. The published parent reports malformed SVG syntax. The strengthened
existing malformed-input table fails before the repair. The cursor now mirrors
the validator's numeric boundary before these unit prefixes, leaving invalid
suffixes for its normal syntax error. Valid exponents retain their exact values.

The admitted extension covers the existing path command grammar and number
tokens accepted by its finite-number validator. The validator's overflow range,
numeric primitive attributes, style dimensions and transform strings are
separate limits. Native exact metadata remains authoritative, including
previously serialized values; this change does not reinterpret an old exact
metadata graph. Public signatures and native versions are unchanged. Standard
path geometry intentionally changes to match its original decimal tokens; the
migration note records that correction. Distinct unequal ellipse radii retain
the existing typed unsupported-ellipse result instead of becoming a circle.

Five maintained acceptance functions check both policies, without an
approximate terminal:

- Absolute and relative line commands retain independently specified rational
  endpoints, including `.1 + .2 = 3/10`.
- Scientific notation, adjacent integers `2^53` and `2^53+1`, and `10^-400`
  retain distinct endpoints. The small endpoint is independently constructed
  from the reciprocal of the exact integer power `10^400`.
- Cubic, quadratic and smooth commands equal independent rational control
  constructions, including reflected controls.
- A decimal semicircle retains its exact radius relation with compact flags;
  genuinely unequal radii and malformed flags/numbers remain distinct failures.
- An imported decimal rectangle equals an independent rational boundary by a
  complete XOR. Reflection, scale and translation, native replay in fresh
  processes, further difference and empty-set recovery consume the result.

The final ten SVG integration harnesses pass with 104 successful test reports
and no ignores. The filled-region acceptance function separately checks both
fresh child processes actually execute one test, then prefixes their transcripts
so they cannot masquerade as additional root harness summaries. The capability
validator originally rejected the unprefixed nested summaries; its original
17 evidence checks and one-summary requirement are unchanged. All five new
capability rows now pass. All 430 previous rows and all target/frontier
metadata remain. A passing blocker assertion is not geometric completion.

Four-crate all-feature, all-target Clippy passes with warnings denied for
Hypercurve, CSGRS, HyperBRep and HyperBRep IO. Rust 1.95 passes the final five
functions, the same strict lint scope and Hypercurve documentation. The
BRep/IO all-feature release run passes 996 test reports in 110 harnesses with
no ignores. Final focused checks retain source-freeze manifests over 3,816
unchanged inputs. These counts describe reports, not universal operation support.

Three alternating fresh-process samples compare the same three complete
unchanged SVG functions. Allocation calls and cumulative requested bytes match;
incremental live-request peaks also match. Absolute live baselines and peaks
differ by one byte, consistent with the candidate executable's one-byte-longer
path; raw counters are retained without adjustment. Descriptive median time
ratios are 0.965–0.992 on the corrected source. Earlier `8d0a6d9caf` samples
include the adverse 1.186 ratio and remain in record 444. Concurrent host activity
prevents a statistical speedup or nonregression claim. Full nonempty dispatch
vectors change in scalar parsing/construction/reduction. Recorded Hyperlimit
counts match in all three controls; the third also emits matching Hypercurve counts.
The first two have no Hypercurve-layer labels. This is not a full-vector
refactor gate.

All five complete new native functions also run in three fresh processes.
Their median times range from 0.040 to 4.487 milliseconds in the derived driver.
The filled/replayed chain takes 2,993 parent allocation requests and 439,066
cumulative requested bytes, with an observed 73,537-byte live-request peak.
Parent counters exclude child heaps and allocator overhead. The derived native
driver executes the requested complete child acceptance function before emitting
its successful completion summary; that custom driver is not libtest. The
native binary grows 26,984 bytes and its trace build grows 29,952 bytes relative
to the parent probe. The incomplete parent's new decimal workload is not a
same-completed-contract speed comparison. An earlier draft quoted the
earlier-stage size differences; the values above use the corrected source
with its strengthened invalid-input workload. Original stage records remain
immutable. Different cache warming from added cases is not an optimization claim.

A derived WASM module executes all five pure maintained functions in three
fresh instances, with the default compiled stack and zero host imports. Native
child spawning is excluded by the maintained cfg gate; pure native replay and
further region operations run. The module is 14,869,916 bytes with SHA-256
`819347609a80188cc709a7092e53de147b11c4e8262c95aef9995c7dfd5ed3ac`.
The largest observed linear memory is 1,572,864 bytes. Linear memory is not
live heap, a resource limit or public ABI qualification.

The first prototype attempted to use a private svgtypes lexer interface and
failed compilation. A later acceptance-test compile failed because an integer
power required its explicit BigInt conversion. Both diagnostics remain in the
raw record; neither is execution or completion evidence. An initial standalone
cost-probe resolution selected newer registry packages and was stopped without
qualification. The final probes use the workspace's admitted package versions and preserve
their complete locks.

The first full pre-push attempt on `8d0a6d9caf` is retired after the diagnostic
regression was reproduced. Its completed and partial commands remain historical,
not qualification of the corrected source. Two inverse-isometric seeded inputs
hit the original ten-second watchdog while other validation ran. Both original
six-byte cases, `0909e00e0700` and `0b094adf6101`, are now mandatory maintained
seeds even if future lexicase rankings change. Every original seed, target,
precision, RSS limit, timeout and campaign duration remains. Fixed-file replay
bypasses that watchdog and does not qualify bounded or mutating completion.
The fresh final gate schedules the unchanged thirteen campaigns after the other
checks and requires actual post-seed mutation. This is scheduling and failure
promotion, not a numerical algorithm or resource-limit change.

The original feature is local record 444, commit
`0a50ee763bf6265c07f7b6b5d9d74ca8c059e588`, SHA-256
`cf2ec6041b0c4f1389fdc85142d8fdd64a16b1c6df80d484cef88fb133acecea`.
The corrected Rust implementation, preserved retired validation, permanent
failure fixtures and current measurements are local record 445,
`docs/evidence/2026-11-08-svg-path-diagnostic-correction`, commit
`5078d920e0fade0f177299879a761e5194dcec8d`, SHA-256
`09e7ccef5d13f4bb6cfad6535d3c35c6838a95e6adacf7b88adc71dbef22f7ca`. All 445 records and four historical excluded digests verify.
No separate evidence remote was written. The complete fresh 280-command
qualification remains required on the final committed source. No performance
improvement, total-work/memory bound, complete SVG domain or completed 524-file
comparison is claimed. All 120 original modified/new external BRep files and
3,736 tracked inputs rechecked unchanged; this increment edits no historical
control worktree.
