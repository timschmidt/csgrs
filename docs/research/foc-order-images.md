# FOC order-image delegation

**Rejected on runtime.** The Phase 9 experiment delegates the private FOC
`interval_min` and `interval_max` helpers to existing Hyperinterval fixed
order images. Both production and test-only proposals are restored; no firmware
implementation, test or size account is retained by this experiment.

The reviewed baseline is firmware `2d68e82b7341db7b747b20e4ad1455a4c1113a00`
and CSGRS `7166cfdc4263693db7a7db054631986e4349cb16`. Hyperinterval owns generic
closed-set arithmetic; FOC retains nonempty Q2.30 coordinates, modulation,
range/error checks and public const construction. P1/P3/P6/P19/P27/P31–P33 and
P35–P39 apply. The missing acceptance witnesses were independent enumeration
through these private helpers and measured completed modulation costs. The
candidate preserves the nested maximum/minimum call order and every subsequent
common-mode, centered-phase and duty check. This fixed-only path has no scalar
decision policy, cache, dispatch trace or approximation terminal.

Two proposed tests enumerate 784 reduced-grid interval pairs and feed 256
generated pairs through modulation and further interval squaring. They pass
before and after delegation. The separately evaluated test-only version adds
explicit image-domain bounds and exact full-`i32` boundary expectations. It
passes 604 portable firmware tests in 55 result lines, 90 release FOC tests,
strict all-target Clippy and workspace formatting. These finite witnesses do
not prove all possible controller behavior or hardware safety.

Native Rust 1.88 and 1.95 probes compare all sixteen result/error fields for
4,108 inputs, including exact points, widths, endpoint extremes, invalid limits
and a deterministic generated corpus. WASM execution matches every field and
makes no host calls. Twelve named cases retain zero measured allocation calls
and bytes; Q30 interval size/alignment remains 8/4 and modulation-result size
remains 56 bytes. The wrappers have no direct native serialization; existing
configuration/protocol tests remain part of the portable suite. The probe's
integer error tags are a test transcript, not a new wire format.

Each compiler has two timing campaigns of eight alternating ABBA/BAAB blocks.
Every process records 31 samples of 4,096 completed calls per named case,
including the returned struct or typed error. Runs use CPU 0, disabled ASLR and
the same executable path, with separate warmups. Across 195,035,136 measured
calls, 81,264,640 are successful modulations and 113,770,496 are typed rejections;
warmup/allocation controls are excluded. Background build activity was not
excluded from every warmup; these are matched host probes, not MCU cycle or
universal performance claims. All samples, including noisy controls, remain.

Rust 1.88's successful nonzero cases regress about 1.98–2.17% in campaign
means. Several rejection paths regress 10.13–10.67%, and the full-range input
regresses 18.62–18.64%. Rust 1.95 repeats smaller but adverse successful-case
means, about 0.61–0.92%, and the same rejection paths regress about 3.38–3.51%
and 9.16–9.32%. Zero-input and early-rejection controls include noisy results.
The repeated supported-workload regressions reject production retention under
the unchanged performance gate. Host disassembly contains separate min/max
helper calls in the candidate; that observation is not a complete attribution
of timing or a reason to try arbitrary layout/inlining variants.

Native probe file growth is 152/328 bytes for Rust 1.88/1.95; `.text` grows
176/112 bytes. WASM grows 515 file bytes and 409 code-payload bytes. These are
diagnostic final host/WASM artifacts, not final firmware images. The initial
size screen does not excuse the runtime regression. Whole-board links, flash
headroom, MCU cycles, transitive/interrupt stack and HIL were not pursued after
that rejection; existing 4 MiB packaging gaps remain release blockers.

The tests were also evaluated separately, with production helpers restored.
The admission declared a test-executable allowance of 32,768 file bytes and
16,384 code bytes, each additionally limited to 4%; other allocated sections
kept the existing 4,096-byte **and** 1% incremental caps. File growth is 22,328
bytes (0.3644%), `.text` grows 1,824 bytes, and the named text-address union
grows 1,823 bytes. Those fit. However, `.data.rel.ro` grows from 75,800 to
76,624 bytes: 824 bytes, **1.0871%**, which fails the unchanged percentage cap.
The test-only proposal is therefore also unaccepted. No retroactive allowance,
baseline reset, assertion removal or binary-layout workaround is adopted.

Recoverable firmware snapshots:

- `refs/interval-unaccepted/foc-order-images-20261116`:
  `15dc920214c971554caccd7a64d1cbc05594f11a` (delegation and initial witnesses).
- `refs/interval-unaccepted/foc-order-tests-20261116`:
  `52d629964e3f5e1402bd0185bcd7df2b13a65df1` (strengthened tests only).

After restoring both paths, the portable suite passes 602 tests, release FOC
passes 88 tests, and strict Clippy/formatting pass. All five measured restored
artifacts are byte-identical to baseline: two native consumers, WASM, FOC unit
tests and the Q30 integration tests. Complete native transcripts and WASM
replay agree. All 89 earlier frozen stages and 45 size accounts are unchanged.
Phase 9 and the full fifteen-phase goal remain open. No push or publication.

Raw commands, patches, original/strengthened witnesses, complete transcripts,
timing samples, section/symbol records and restoration checks are retained in
`docs/evidence/2026-11-16-interval-foc-order-images` in the evidence repository.
Labels `retained` and `final` in experimental files denote intermediate
test-only candidates; the final disposition is rejected, and `restored`
records are the accepted baseline restoration.


Review commit `59d1b3f6416b480b5a725c17b1403d111308808d`. Evidence archive commit
`d068b83c24cccc8c8bedc139815f9b355936c07c`: 352 files / 21,273,567 bytes,
SHA-256 `40bcfaa2fe4394c74a78dc70295d5401cff7ebb4ab0cb7e3c9e6835dd59bb780`.
