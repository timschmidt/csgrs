# Haskell comparison harness audit — 2026-09-07

Source: https://github.com/michalkonecny/haskell-reals-comparison
Pin:7c53d4e4259f633606b18d655cdbbf813866ab81,2022-07-15.
All567tracked regular files are inventoried:14text/code files1534lines,
525stored logs14025lines,14SVG assets42outer physical lines,14PNG assets.
Total7,746,380bytes. Initial unbounded rg listing was truncated and gives no
read credit. No donor file is changed and no donor build/script is executed.

Current credited reads:14files/1534physical lines, complete numbered untruncated
reads. all.js and results.html are now covered; archived logs and chart assets
have provenance reconciliation, while semantic source review remains bounded to the
recorded parser/asset findings.
The primary GitHub project page was browsed; local pinned source is authoritative
for file coverage. No native performance or correctness result is claimed yet.

| File | Full range | Findings / Hyper comparison |
| --- | --- | --- |
| .gitignore |1–4|Ignores Stack/Cabal build output and a benchmark dependency directory; no scalar mechanism.|
| LICENSE |1–27|BSD3 text, old2015–2017copyright and aern2 endorsement name; not algorithmic. No code copied.|
| README.md |1–48|Distinguishes lazy Cauchy composition from fixed-mantissa restart evaluation. Its three-consecutive-within5% repetition claim is commented out. Precision/absolute-accuracy distinction needs checking against each actual adapter; comments and historical dates are not validation. Hyper already has per-node requested error and a synchronized finest cache; whole-program restart remains a different execution model.|
| Setup.hs |1–2|Only Cabal defaultMain; not executed.|
| benchmarks/README-charts.txt |1–22|Historical CSV/chart-tool instructions conflict with current JS row output; includes destructive cleanup and converter commands, none executed. Chart-generation provenance remains pending.|
| benchmarks/runBench.sh |1–183|Single fixed-order process per task/parameter/method; reuses logs by name without binary/source/accuracy fingerprint. Failed logs are deleted, so do not execute unchanged. Parses achieved accuracy but omits it from exported rows; reused logs get aggregation-time date. Replaces the first0.00substring with0.01 in CPU strings, not solely exact-zero values. No repeated/error-bar/oracle gate; report is a useful workload inventory, not a validated Hyper comparison.|
| haskell-reals-comparison.cabal |1–47|Generated hpack0.34.4 manifest matches active YAML dependencies; lower bounds only, no frozen build plan. One executable, RebindableSyntax/O2/Wall/rtsopts, no test suite despite hspec/QuickCheck dependencies.|
| package.yaml |1–44|AERN2MP/Real>=0.2.9, CDAR-mBound>=0.1.0.4 and mixed-types-num>=0.5.9; version0.1.1.0. These bounds do not establish historical compiled versions.|
| src/Main.hs |1–215|CLI destructures exactly four arguments; no explicit iteration-domain guard. Logistic CDAR uses round(bits/3.32), whereas AERN2 uses the supplied bits; possible unit mismatch requires reading the selected CDAR dependency contract. ManyDigits maps nominal decimal N to round(3.32N), which is not generally a sufficient binary demand for N decimal digits. It limits CDAR mantissa after require; final achieved accuracy needs independent checking. Generic MP hooks use fixed100bits while MP1 uses requested accuracy and starts at3.32times its bit count. Outer AERN2 iteration still checks final requested accuracy, so fixed100alone is not a demonstrated wrong answer. Ireal/cached-arrow variants are commented out, not active coverage.|
| src/Tasks/PreludeOps.hs |1–74|Strict-to-WHNF logistic loop c=3.82,x0=1/8 and seven explicit elementary-function workloads. Negative iteration count moves away from zero; n=0 returns the input. Hooked Maybe operations control restart/precision. No reference-value oracle. ManyDigits6is absent,8is present but disabled in driver. Hyper has exact rational constants and demand evaluation; a new recurrence node/restart API is not justified merely by this synthetic loop.|
| src/Tasks/MixedTypesNumOps.hs |1–58|Mixed rational coefficient operations avoid promoting the constant unnecessarily; already fits Hyper rational/dyadic specialization. Positive arrow composition applies hooks in countdown order, matching the scalar loop. Arrow foldl1 over take n is empty for n=0, unlike the scalar identity case; native boundary qualification pending. No transfer yet.|
| old/IRealOps.hs |1–51|Dormant adapter: per-step precision forcing with doubling, or an a-priori ceiling(n*log c)+digits estimate. The latter uses a floating logarithm and is described as exactly required without a local proof/rounding guard. Hyper's proof-bearing integer demand budgets should not be replaced by that heuristic; native reproduction and dependency semantics remain pending.|
| benchmarks/all.js |1–526|Static export of 525 records dated 15 Jul 2022 for logistic and ManyDigits 1,2,3,4,5,7 across cdar_mBound/aern2_CR/aern2_MP. Rows contain time, benchmark, parameter, method, user/system time and memory, but no achieved-accuracy field despite runBench.sh parsing it. Useful for workload shape and memory-growth inspection, not independently verified accuracy/performance; timestamps are regenerated for reused logs by the driver.|
| results.html |1–232|Plotly page groups all.js by benchmark/method and plots user+system time and memory on log axes. No correctness or accuracy visualization, uncertainty bars, data hashes, or pinned CDN assets (`plotly-latest`, MathJax). C6 is absent and C7 is shown; setup records Ubuntu 20.04/GHC 9.0.2. No native execution performed.|

Archived-log reconciliation (all525 `.log` files, 14,550 physical lines/14,025 non-empty, 2,940,371 bytes) was performed by a bounded parser without modifying donor files. Every log maps one-to-one to an all.js row (525/525); all exit statuses are zero and maximum-RSS values match (525/525). User/system time differs in 385 rows because the exporter normalizes displayed `0.00` values to `0.01`; this confirms the report is not a byte-for-byte raw timing export. Achieved-accuracy lines occur in 350 logs (all175 aern2_CR, all150 aern2_MP, all25 aern2_MP1); all175 cdar_mBound logs lack one, so CDAR accuracy is not represented in the report. The logs are provenance evidence, not a correctness oracle or fair modern benchmark. SVG chart assets were all parsed: 14 static three-line SVGs, no script/image elements, 400x300 viewBox outputs. The 14 PNG counterparts are static 800x600 RGB images. They contain no scalar implementation or reusable evaluation machinery; retain only as historical report artifacts.

Next: read all stored reports/assets under explicit coverage; reconcile525logs
with exported data and chart consumers;
verify precision conversion/achieved accuracy against the already audited
CDAR/AERN2 snapshots; qualify native task boundaries and only then consider a
fair performance experiment or transfer. Do not infer performance winners from
the README's status table or historical charts. Full goal ACTIVE/OPEN.


## Archived-log line traversal

All 525 archived `.log` files were traversed line-by-line by a bounded parser without modifying donor data. `HASKELL_COMPARISON_LOG_REVIEW.json` records each file's line/byte count, SHA-256, and recognized timing/RSS/exit/accuracy fields. These entries are marked `DATA_REVIEWED` (not `READ`) in the coverage ledger because they are generated benchmark outputs, not implementation source; no scalar design or correctness claim is inferred from them.

The 28 generated chart assets are now explicitly `REVIEWED_ASSET`: each PNG's signature and 800×600 dimensions were checked, and each SVG's opening/closing envelope and physical line count were checked. They remain provenance/visualization artifacts, not scalar implementation source.
