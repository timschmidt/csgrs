# Native combination without persistent certificates

The isolated candidate is rejected: the original full-board gate fails and
library consumer costs remain. All measured library witnesses are preserved.
The [decision](decision-v1.json) closes this candidate without admitting it.
All 35 authorized native paths and the admitted P4 repair remain intact.
The complete properties plan remains open.

The [source review](source-review-v1.json) covers all 12 changed/added paths.
The original normalizer, cache representation, fact APIs and Laurent
compaction are unchanged. The scalar change is one checked integer-leaf
conversion; curve and allocator changes are the previously reviewed bytes.

The [fixed scope](scope-v1.json) reuses 11 unchanged baseline binaries and
collects entirely new paired samples. All 155 commands succeeded, including
868 scalar tests (11 admitted P4 regressions), five endpoint regressions,
three shared metadata tests per variant, candidate scalar/metadata Clippy
with warnings denied, 20 preflights, six abort controls and 110 cost runs.
The original native gates and wider stack are not covered by these counts.

The [assessment](library-assessment-v1.json) retains 365 timing and 134
allocation rows. Every witness and representation inventory matches, as do
all 210 abort receipt rows and the 231 historical broad witnesses.

| Suite | Timing rows | Lower in all five pairs | Higher in all five pairs |
| --- | ---: | ---: | ---: |
| rational | 24 | 13 | 0 |
| quadratic | 32 | 21 | 1 |
| integer | 12 | 3 | 1 |
| arc | 48 | 23 | 2 |
| metadata | 18 | 6 | 0 |
| broad | 231 | 24 | 16 |

The [interpretation](library-interpretation-v1.json) keeps all adverse rows:
approximate non-endpoint first-query timing has median ratio 1.1855; broad
affine quadratic and sparse drill rows have ratios 1.1819 and 1.1009. Fourteen
allocation rows contain increases. Phase-relative requested/live/peak bytes
are not RSS or whole-life retention. All eleven comparable binaries shrink.
Ratios and pair directions are descriptive, not significance estimates.

The [native assessment](native-gates-assessment-v1.json) records ten commands,
including two passing allocator controls and verified MiMalloc dispatch.
Both full-resolution archives passed integrity checks. The full-board test
completed both reports with 12 and 197 findings, but failed at **136.717191044
seconds of worker CPU**, above the unchanged 120-second limit. Copper-neck
passed its original 60-second check-body assertion. Its internal passing
duration is unprinted; the 127.080-second process wall includes setup.

The [consumer source review](consumer-source-review-v1.json) records untested
integer/endpoint hypotheses and the cache copy/publication paths. The
[next diagnostic](next-diagnostic-v1.json) will use existing library builds
to compare arithmetic traces across construction and query phases. Trace
counts cannot establish elapsed costs or erase any closed adverse sample.
