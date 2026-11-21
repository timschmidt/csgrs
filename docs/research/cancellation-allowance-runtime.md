# Cancellation memory allowance and focused runtime review

The standalone publication check remains isolated at
`8a454c86054ae4d5a4b0813d8e9edd1e6775ec47`. A proposed, target-specific byte
allowance reduces its failed resource accounts from 17 to one while retaining
all measurements and historical positive charges. The remaining allocation/free
count failure still blocks admission. This review extends the
[standalone qualification](cancellation-standalone-qualification.md), under
Hypersolve work-budget ownership, P5/P6/P20/P36 and plan 14.4.
The functionality prevents a cancellable scope from publishing success after its
callback requests cancellation without reaching another instrumented check.

The later [finish-borrow review](cancellation-finish-borrow.md) evaluates a
revised owner implementation with a destructor-reentry witness. It preserves
this record and its failed obligations; neither candidate is admitted.

## Proposed byte allowances

These allowances cover only the measured four-case consumer on
`x86_64-unknown-linux-gnu`, glibc, one worker, with active tracing. Both the
absolute and relative limits must pass for each incremental and cumulative
positive charge. They are not general memory allowances or accepted exceptions.

| Workload and metric | Previous cap | Proposed cap |
| --- | --- | --- |
| One variable: requested/usable temporary peak | 4,096 bytes and 2% | 2,048 bytes and 10% |
| One variable: requested/usable allocation traffic | 4,096 bytes and 2% | 4,096 bytes and 5% |
| 64-variable stress: requested/usable allocation traffic | 4,096 bytes and 2% | 16,384 bytes and 0.25% |

The one-variable peak proposal permits a small discrete cost in an approximately
16 KiB working set while tightening the absolute allowance. Its 2 KiB ceiling is
0.1953125% of the unchanged 1 MiB heap capacity and 0.048828125% of the 4 MiB
worker reserve. The stress traffic proposal tightens the relative limit; its
16 KiB ceiling is 0.006103515625% of the unchanged 256 MiB traffic capacity.
Traffic's absolute allowance scales by 32 for the existing 32-cycle histories;
the relative allowance and peak allowance do not scale.

All count, retained/persistent byte, stack, capacity, headroom and lifetime
retention gates remain unchanged. So do exactness, certainty, dispatch traces,
runtime, binary size and validation obligations. Zero denominators still require
zero growth. Added cancellation/retry has no fabricated successful-old baseline.

The offline review changes 72 byte-metric caps and 42 metric verdicts across
288 accounts. Independent fraction arithmetic verifies the result. Original
values, denominators, positive charges and verdicts remain alongside the proposed
verdicts; later decreases never erase earlier charges. The sole remaining failed
account is the one-variable STRICT, active-trace, reused-input warm-state case.
Its historical allocation and free charges are two each, exceeding the unchanged
1% limit against denominators 179 and 194 respectively. The absolute count cap
remains 16. All proposed byte checks pass; this does not establish admission.

## Runtime contract

The focused screen compares primary `e63cbf68aea24a5939332c738c4a6652be881627`
with the isolated candidate. Primary `cfbff85ad075d81c2d4d3bddc04d2977bdb57b82`
differs from that base only in documentation. The existing owner certification
benchmark supplies 15 cases; the complete consumer supplies 16 default and 32
trace cases, covering both policies, reused/rebuilt input and 1/4/16/64 variables.
The trace build covers passive and active recording separately.

Each public timed iteration materializes its output and optional trace, checks
the original exact expectations and generation invariants, and destroys the
output, trace and any rebuilt owned input. Reused-input construction and final
teardown, initial printing and debug serialization are outside timing. Process
and thread-local caches may be warm. No allocator interposer or stack probe is
present, and no cold-latency claim follows. Added cancellation/retry checks exact
diagnostics, scope cleanup and fresh retry inside each iteration; its absolute
costs are recorded separately without an old-success denominator.

The first build fails because source extraction also copied an unused stack-probe
extern declaration without its `c_void` import. Removing only that unused
declaration preserves every executed observer function, fixture and assertion.
The original failure and owner image remain. Both attempts share the original
3,600-second build deadline and 160-child ceiling: together they use 51 children
and about 720 seconds. Eight final images pass 174 functional case invocations,
six strict Clippy configurations and formatting. Independent build verification
passes. Public/resource consumers share their locked core dependency identities;
the owner benchmark retains its separate repository lock.

Timing uses CPU 0, disabled ASLR and the same executable path and environment
per family. Each common family runs in ABBA/BAAB order, with 30 Criterion samples,
0.5-second warmup and one-second requested measurement. Slow stress cases retain
all samples. Host checks before and after each invocation detect no competing
compiler/routing work; they do not prove continuous host exclusivity. Within-run
confidence intervals are not independent cross-process non-regression proofs.

## Results and disposition

All 26 benchmark invocations complete, covering 552 case invocations and 16,560
samples. The audit verifies 432 public full control records, all image and raw-log
hashes, sample arithmetic, the launch controls and 53 host boundary records.
The 63 common cases have four balanced process comparisons each; 48 added
cancellation/retry cases retain their absolute completed costs separately.

| Family | Cases | Range of median paired time changes | Slower / faster pairs |
| --- | --- | --- | --- |
| Owner benchmark | 15 | -0.74% to +0.86% | 27 / 33 |
| Default consumer | 16 | -3.34% to -1.45% | 0 / 64 |
| Trace consumer, passive and active | 32 | +0.25% to +3.96% | 96 / 32 |

Two passive-recording cases in the trace build are slower in all four pairs:
reused one-variable APPROXIMATE_512 has a +1.02% median (about 777 ns), and reused
four-variable STRICT has +1.71% (about 6,553 ns). The largest adverse median is
reused one-variable STRICT at +3.96%; its individual pairs range from -8.75% to
+7.69%. That contrary result and all other variability remain in the evidence.
These observations do not establish a universal slowdown or its mechanism.
They also do not establish non-regression. Favorable default results cannot
cancel the trace-build obligations.

The timing producer uses 80 direct children and about 1,233 seconds, within its
original 160-child / 3,600-second limits. The offline audit passes in about
1.3 seconds of its separate 300-second bound. No unchanged benchmark rerun is
used to replace adverse observations. Full owning benchmarks, successive
operations, distribution and remaining admission checks are still open.

The candidate stays isolated. The byte proposal resolves 16 failed accounts
under its declared limits, but the remaining count failure and runtime evidence
prevent admission. This applies the bounded-memory exception without relaxing
other gates. Primary production code, accepted totals of 57 size / zero memory
exceptions and the 1,644-row inventory remain unchanged. All producers are
terminal; the source worktree is restored clean, the workflow queue stays held
and its original watcher stays terminal. No push; the full goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-allowance-runtime`,
archive commit `d88ee4db5d43c610ad77bf8d59313fb02dc1d411`; 3 files / 25,668,639 bytes,
SHA-256 `c6b396475714d5702a72a53f5ec73deba66ebaf831b61f31c36872a8dc6f7b53`.
The lossless payload has 4,677 members / 100,888,275 raw bytes and
nine validated images. Original, decoded and committed hashes are verified;
earlier archive paths are unchanged.
