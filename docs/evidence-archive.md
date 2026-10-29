# Evidence archive

Generated evidence does not live in this repository. That covers raw
command logs, benchmark samples, source snapshots, audit corpora, replay
bundles and checkpoint archives. It is kept in a separate evidence repository.
This repository keeps a digest for each archived record in
[`evidence-archive.tsv`](evidence-archive.tsv).

| | |
| --- | --- |
| Evidence repository | <https://github.com/timschmidt/csgrs-evidence> |
| Historical archive commit | `b20000e377626a41b6ff8e6f87aee2b20ebd7865` |
| Extracted from | CSGRS `7e3c36c1248823a8bb36b2b161068660cea2ebf3`, a pre-rewrite commit ID (see [history rewrite](#history-rewrite)) |
| Historical records | 309 records, 552,370 files, 13.96 GB |
| Research branch | [`research-2026-10-25`](https://github.com/timschmidt/csgrs-evidence/tree/research-2026-10-25) |
| Research closeout commit | `cd8bf060aa577231f84fffd7cf21e15127c028b0` |

The archived commit keeps the original paths and the history of every archived
root. A link such as `docs/evidence/<record>/README.md` in older commits,
ledgers or admission records resolves at the same path in the evidence
repository:

| Archived root | Record granularity |
| --- | --- |
| `docs/evidence/` | one record per child |
| `crates/comparisons/audits/` | one record per child |
| `crates/comparisons/integration/` | one record |
| `crates/hyperbrep/docs/evidence/` | one record per child |
| `crates/hyperspatial/verification/` | one record per child |
| `crates/hyperreal/benchmarks/checkpoints/` | one record |
| `benchmarks/baselines/` | one record per child |

The archive also carries the comparison crate's audit verifiers
(`crates/comparisons/scripts/`) and its `.gitattributes`, so the audit corpus
can be rechecked from an evidence checkout. Five audit results are Git LFS
payloads; run `git lfs pull` after cloning. Their pointers, which embed each
payload's SHA-256, are what the record digests cover.

## History rewrite

On 2026-10-20 the archived roots were also removed from all CSGRS history
with `git filter-repo`. The packed repository shrank from about 1.4 GiB to
173 MiB. That gave 2,033 commits new IDs and dropped 218 commits that only
changed evidence. Commits that never touched an archived root, tags, and the
`pages` and `v0.20.0` branches keep their IDs.
[`history-rewrite-2026-10-20.tsv`](history-rewrite-2026-10-20.tsv) maps each
changed pre-rewrite commit ID to its new ID; dropped commits map to zeros. Use
it to translate commit IDs cited by ledgers, performance records and older
documentation.

## Verifying a record

`evidence_archive.py` computes one SHA-256 per record from git objects, so
no working-tree checkout is needed. A record digest covers the content digest,
file type and relative path of every file in the record:

```sh
git clone --bare https://github.com/timschmidt/csgrs-evidence.git
python3 scripts/evidence_archive.py verify csgrs-evidence.git \
  cd8bf060aa577231f84fffd7cf21e15127c028b0
```

## Excluded third-party archives

Four downloaded package archives (`*.crate`) were part of the audit corpora.
They carry their own licences and are not redistributed in the evidence
repository. Their `excluded:` rows in the manifest record each path, size and
SHA-256, so the identical package can be fetched again from crates.io:

| Package | SHA-256 |
| --- | --- |
| `num-bigint 0.4.6` | `781d22cfabc4348bc57143607a8b288d98c050c5f8b87f5a2a36bc27eaa9425c` |
| `computable 0.1.0` | `e3134cfa5d49bf2fb4292fb9711cedc1eb0571f4ce790a4e840eea8f3f847912` |
| `computable-real 0.3.0` | `d15f7d44e4a3b8bbaf4222ccc45a3196a9dec539d2a0d10eb14e31635fd50d9c` |
| `reals 0.4.0` | `5931e5a33599abd0514ffc6e7bb902d161963bb1a1670b1c03df89fe0f4d9775` |

## Adding evidence

The retention budget in [`admission.md`](admission.md#7-complete-the-admission-record)
governs new records. Admission records in this repository stay short and cite
evidence by repository path and digest. Raw material is committed to the
evidence repository, and its record digest is added to `evidence-archive.tsv`.
`scripts/check-repository-budget.sh` rejects archived roots and oversized files
in this repository.

## Follow-on records

The local hypercurve-selected-closure-integration-evidence-2026-10-29 branch ends at
7b14c809b12cef95623e853878286c401c38ac0f, adding docs/evidence/2026-10-29-hypercurve-closure-selected-closure-integration (590 files,
16,528,148 bytes), SHA-256 1aadc6e3a90206655e2100570527b1f68497ced2f9c3f650ee5d8eb174fa3ae2. Its parent merges all six selected-query
increment evidence histories and the earlier selected-field publication record.
Fourteen focused cases and 25 combined qualification groups pass. Original
derivative/pole failures and seven remaining region-operation blockers are
retained. General closure and publication of the combined source are not
claimed. This record is unpushed.

The local hypercurve-selected-query-fibers-evidence-2026-10-29 branch ends at
081ece0c6fb112e62edeed9e00dc051923d1a342, adding docs/evidence/2026-10-29-hypercurve-closure-selected-query-fibers (723 files,
24,874,907 bytes), SHA-256 24c4e6fddbed73a91bffa8466b176f8fab26ff22b4abea8749712c8b9f4b380d. It records dependent polynomial fibers
over retained selected queries, ten exact/resource/replay cases and 25 affected
qualification groups. Original unsupported, depth-limit and error-attribution
failures, reconstruction patches and seven region blockers remain explicit.
General operation closure and publication are not claimed. This record is
unpushed.

The local hypercurve-selected-field-root-discovery-evidence-2026-10-28 branch ends at
1fa282786f3adf54016b166024c1886e2b03af43, adding docs/evidence/2026-10-28-hypercurve-closure-selected-field-root-discovery (371 files,
24,836,942 bytes), SHA-256 2d7816185d4199b16fb59c194500599805dc62ebb230f7f3f6a7ef0f2c15aa8e. It records complete ordered-field
root discovery, eleven named exact/resource/replay tests and 15 affected
qualification groups. Original declined discovery cases, reconstruction
patches and seven geometric region blockers remain explicit. General operation
closure and publication are not claimed. This record is unpushed.

The local hypercurve-rational-query-guards-evidence-2026-10-29 branch ends at
12613042e4442901f1f6afa397a5af9bb22a7be3, adding docs/evidence/2026-10-29-hypercurve-closure-rational-query-guards (553 files,
18,433,327 bytes), SHA-256 8dd0311b5bd14b8b8502a1c6f267a3bd45317ff8bda41a18549a2105caf4978d. It records preservation of original
rational quotient guards through scalar recipes and native replay, five named
witnesses and 20 affected qualification groups. Source checkpoints are
reconstructable from retained patches and inventories; receipt-only parent
alignment preserves implementation bytes. The original failure and seven
region operation blockers remain explicit. This record is unpushed.

The local hypercurve-selected-query-derivatives-evidence-2026-10-28 branch ends at
43ed287e44c8d26df3df90621662a9ba81b56a06, adding docs/evidence/2026-10-28-hypercurve-closure-selected-query-derivatives (845 files,
38,478,490 bytes), SHA-256 a2de299ec93bb57265fd466575f7ca4a72fc11b86a4b93a5f8d85b0ef889ccd4. It records source-preserving rational
query derivatives, twelve exact/domain/resource/replay witnesses and 20 affected
qualification groups. Source checkpoints are reconstructable from retained
base/seed/qualified patches; receipt-only parent alignment preserves every
implementation byte. Original failures and seven region operation blockers
remain explicit. Full closure and publication are not claimed. This record is
unpushed.

The local hypercurve-selected-join-correspondence-evidence-2026-10-29 branch ends at
ff5b5ed28c523df708e1eb734869a111fd0d4871, adding docs/evidence/2026-10-29-hypercurve-closure-selected-join-correspondence (619 files,
21,592,106 bytes), SHA-256 450fce88ea94d1730595564db3bf8e48f84ca8335e3f0243f7823143c475af21. It records retained selected-field
generator correspondences, original-guard validation and fresh-process graph
replay, nine named fixtures and 24 qualification groups. Source checkpoints are
reconstructable from patches and inventories. Original duplicate-generator and
cold re-export failures remain explicit, along with seven region-operation
blockers. This record is unpushed.

The local hypercurve-selected-polynomial-points-evidence-2026-10-28 branch ends at
3a6a311a298224770578f5d42ae684f01d642d62, adding docs/evidence/2026-10-28-hypercurve-closure-selected-polynomial-points (1,576 files,
91,256,621 bytes), SHA-256 cf7d597a8f89561195e6748497e678ad3c34a0a3635ce9741df6ff3f5886631b. It records selected query points and
parameters, cold native replay, restrictions and subsequent operations, exact
source guards, native-base retention and 24 affected qualification groups.
Original failures, resource limits and seven remaining region operation
blockers are retained. Full algebraic closure and publication are not claimed.
This record is unpushed.

The selected-field publication archive ends at `2f897f1fbaedb6569c0302c5b0485b50e3212ba3` with
380 records. Its new record `docs/evidence/2026-10-28-hypercurve-closure-selected-field-publication-validation` contains 14,191
files and 55,660,767 bytes, SHA-256 `3a7a6ffabad06e26c19f33dbeb24f66eb0ac5f3251f2b4f4e52ca5ea55a85164`. It records fresh success for
all 328 pre-push command groups on `c57653e9963a199e88fdfddac3d60f5ff9b07351`: 131 locally executable workflow
groups, 196 strict Clippy groups and the CI security audit. All 13 ASan campaigns
include mutation evidence. Stable 1.99.1, MSRV 1.95.0, nightly-2026-10-28, both
release/package checks and the hosted-action review are retained. All archive
digests verify and four historical exclusion receipts remain. Only three receipt
files differ from the qualified code and are checked separately. This supersedes
pending pre-push status through selected polynomial field replay. Later selected
query point/parameter and derivative increments are excluded. Exact geometric
operation closure and hosted publication/deployment are not established here.

The local integration candidate combines selected polynomial field evidence
2472aef555210a4222d04fe1b746c8c7ac015be3 and published evidence e784a8ea0b53cc24f3b56a61d63a13e66b9e7701 at 9700b7d8fd1124660c33a1059ebf321e968ce1f3.
All retained records from both parents remain in the aggregate manifest.
The next complete workflow/Clippy qualification is pending; this merge itself
does not qualify or publish the implementation.

The local hypercurve-selected-polynomial-replay-evidence-2026-10-28 branch ends at
2472aef555210a4222d04fe1b746c8c7ac015be3, adding docs/evidence/2026-10-28-hypercurve-closure-selected-polynomial-replay (145 files,
9,456,865 bytes), SHA-256 fb6e286a80355fe74b82833f708652cfdcdef185cb2394b1531693f7d1730316. It records strict shared scalar-graph
replay, eight witnesses plus a fresh-process child, eighteen predecessor
witnesses and 14 qualification groups. Geometric blockers, resource limits,
test-only serialization and parent alignment remain explicit. Geometric closure
and package delivery are not claimed. This record is unpushed.

The local `hypercurve-selected-polynomial-bounds-evidence-2026-10-28` branch ends at
`24ef19eea6278aaf9d75d8dec596c4129355a752`, adding `docs/evidence/2026-10-28-hypercurve-closure-selected-polynomial-bounds` (104 files,
6,609,652 bytes), SHA-256 `a0fe988c8ad9e07d71923244cc2ef91e9791f81100a283d1bf91f147f3047e83`. It records certified enclosures of
retained polynomial-field values, five new exact witnesses, thirteen predecessor
witnesses and 13 qualification groups. Geometric and point-consumer blockers,
resource limits and parent-alignment evidence remain explicit. No completed
geometric closure or package delivery is claimed. This record is unpushed.

The local `hypercurve-selected-polynomial-joins-evidence-2026-10-28` branch ends at
`84e540fa5f525e65d539cee6e82f588e33d2231c`, adding `docs/evidence/2026-10-28-hypercurve-closure-selected-polynomial-joins` (105 files,
6,495,209 bytes), SHA-256 `f6d1431299629a7604de444a7c4433db2d5faf3f886db8e45c75817bbe0890ff`. It records exact joins of selected
polynomial fields, five new composition witnesses, eight predecessor witnesses
and 13 qualification groups. The source attribution and parent alignment are
retained. Seven public geometric operation blockers remain explicit; no complete
geometric closure or resource bound is claimed. This record is unpushed.

The local `hypercurve-selected-polynomial-fields-evidence-2026-10-28` branch ends at
`bbeb520c52b1981ecab206e583963740aa1087b5`, adding `docs/evidence/2026-10-28-hypercurve-closure-selected-polynomial-fields` (466 files,
10,794,189 bytes), SHA-256 `6d899441a567c861c800cff052716fcdd7b26dbc7ef69c8b0b62ccb655815d46`. It records selected polynomial
fields, eight exact witnesses and 23 local/downstream qualification groups.
The elliptic scalar contact succeeds; seven public geometric operations remain
unsupported. Original failures and resource limits remain explicit. No completed
geometric chamfer, universal closure or package delivery is claimed. This record
is unpushed.

The local `hypercurve-polynomial-substitution-evidence-2026-10-28` branch ends at
`588685ed15c8b41c5c1690523b46fd3669f1f73e`, adding `docs/evidence/2026-10-28-hypercurve-closure-polynomial-substitution` (127 files,
8,537,302 bytes), SHA-256 `99744a5cbe56cd5f10eb2657d6a7cc46abacaeed0f81b079bba79e01785b69a1`. It records retained-field polynomial
substitution, five independent arithmetic witnesses, local solver qualification
and affected curve/model consumers. Original geometric operation blockers and
failed baseline compilation remain explicit. No completed implicit chamfer,
universal closure, resource bound or delivery is claimed. This record is unpushed.

The local `hypercurve-qualified-region-adapters-evidence-2026-10-28` branch ends at
`fedf350fcf2a4d555b95ffec6af213d52dedc68a`, adding `docs/evidence/2026-10-28-hypercurve-closure-qualified-region-adapters` (220 files,
17,313,387 bytes), SHA-256 `1a421bfa417a475db8cbdb9fd72a13751c2863685b63255386603d9a1489ae75`. It records the CSGRS membership
adapter's inherited topology receipt. The complete CSGRS suite, three new
fixtures in full and minimal feature configurations, affected consumer and
FFI suites, strict native/WASM Clippy and actual Node/TypeScript binding
consumers pass. The four-path scope, final parent qualification and receipt-only
transition are checked. Earlier failures remain archived. This record is
unpushed; general closure, complete resource bounds and delivery remain open.

The local `hypercurve-qualified-region-queries-evidence-2026-10-28` branch ends at
`c0f8d44e0741bffa646cc5c89bc73ca878db8e21`, adding `docs/evidence/2026-10-28-hypercurve-closure-qualified-region-queries`
(900 files, 43,595,026 bytes), SHA-256 `e53da03d4e3a828d095c5531aa534edf10504c0446e94343f5c74086c3c6256c`. It records
CurveRegion2 geometric queries, policy-aware native contour caching and native similarity lowering. All 2,931 Hypercurve cases, 67
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 271 fixtures pass; 34 existing ignored cases remain.
Every final implementation and acceptance byte is freshly qualified. Original
query failures, rejected guard behavior, exact/nonempty controls and zero-budget
instrumentation are archived separately. Parent 79ca1ae442 and the receipt-only
transition are recorded. This record is unpushed; full closure and package
delivery remain open.

The local `hypercurve-empty-native-replay-evidence-2026-10-28` branch ends at
`5396eff3cb4f6b019d664d83e835e0e041408743`, adding `docs/evidence/2026-10-28-hypercurve-closure-empty-native-replay`
(1,175 files, 46,419,497 bytes), SHA-256 `6b577979c9cbf9bbcc76d32a9fccc5622233d308c790a98788c8ef970478c346`. It records
CurveRegion2 normalization publication and replay, Boolean identity dependencies, affine transforms, finite projection and strict native SVG export. All 2,921 Hypercurve cases, 67
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 261 fixtures pass; 34 existing ignored cases remain.
Every final implementation and acceptance byte is freshly qualified. Exact
empty graphs, separate qualified topology receipts and subsequent consumers
are covered, including original failures and source-attributed zero-work
instrumentation. Parent 0fe42adbab and the receipt-only transition are
recorded. This record is unpushed; complete closure, resource bounds and package
delivery remain open.

The local `hypercurve-selected-circle-corners-evidence-2026-10-28` branch ends at
`a8894028434896af26584e04d3f0367d5787fe65`, adding `docs/evidence/2026-10-28-hypercurve-closure-selected-circle-corners`
(1,600 files, 92,870,882 bytes), SHA-256 `d39113527e8a6013bce19c9ada17faefc0be34fd2a8bd8e5adaeb85168b904b9`. It records
Hypercurve common point fields, selected-circle incidence and endpoint transport, corner reconstruction, arrangement side rays, finite projection and native circle definitions. All 2,911 Hypercurve cases, 66
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 251 fixtures pass; 34 existing ignored cases remain.
Every final source input is freshly qualified; the initial attempt with a
missing fixture snapshot is not credited. Source equality and parent
`2404957c1e` are recorded. This record is unpushed; complete closure, resource bounds and package
delivery remain open.

The local `hypercurve-circle-line-contacts-evidence-2026-10-28` branch ends at
`6115a1e02fe881250cda8eb537f67e5680e32a73`, adding `docs/evidence/2026-10-28-hypercurve-closure-circle-line-contacts`
(407 files, 9,463,829 bytes), SHA-256 `4f5b200100277b66bf2680c5c6992fa9966c7172f005142e9a8390a5de762ea5`. It records
Hypercurve common point-field rational queries, circle/chord incidence, chord orientation and overlap, and point equality using retained two-support evidence. All 2,896 Hypercurve cases, 65
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 239 fixtures pass; 34 existing ignored cases remain.
The recovered source is freshly qualified; lost pre-restart attempts are not
credited. Source equality and the transition from parent `437ebd05cd` are
recorded. This record is unpushed; complete closure, resource bounds and package
delivery remain open.

The local `hypercurve-region-replay-evidence-2026-10-28` branch ends at
`50903a9a13df1ad56c60251cfe7883f4c0fc8857`, adding `docs/evidence/2026-10-28-hypercurve-closure-region-replay`
(373 files, 7,191,304 bytes), SHA-256 `2890052379c7cb1997e3c1e57174c0101cde7f1e8fe4a6315a788a4f7e37917e`. It records
Hypercurve native SVG graph interchange, CurveRegion2 regularization-policy replay, and the CSGRS SVG adapter. All 2,888 Hypercurve cases, 65
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 231 fixtures pass; 34 existing ignored cases remain.
The recovered source is freshly qualified; lost pre-restart attempts are not
credited. Source equality and the transition from parent `60fcfc8526` are
recorded. This record is unpushed; complete closure, resource bounds and package
delivery remain open.

The local `hypercurve-selected-circle-regions-evidence-2026-10-28` branch ends at
`74c7a626ed6b4a7c88332336bf5d3e416cc94e60`, adding `docs/evidence/2026-10-28-hypercurve-closure-selected-circle-regions`
(362 files, 9,196,503 bytes), SHA-256 `fc6fe4b2838bd73c3b30eee19c1da57af1c2fabf612c29e8f8e1d71e5c225bc8`. It records
Hypercurve common point-field predicates, retained line representatives, circle endpoint contacts, minor-arc and line winding, and the existing CurveRegion2 arrangement. All 2,878 Hypercurve cases, 64
root cases, 3,012 consumer cases, strict Clippy, core builds, docs, 16 matrix
guards and 221 fixtures pass; 34 existing ignored cases remain.
The recovered source is freshly qualified; lost pre-restart attempts are not
credited. Source equality and the transition from parent `d75bb9ea3c` are
recorded. This record is unpushed; complete closure, resource bounds and package
delivery remain open.

The local `hypercurve-rational-point-images-evidence-2026-10-27` branch ends
at `01f2e7a9311b962f07e3b7ba6a71d9fcf38e2d63`, adding `docs/evidence/2026-10-27-hypercurve-closure-rational-point-images`
(772 files, 33,841,191 bytes), SHA-256 `79bf8acccb4cf36a8782e5288fa411c360f85b4d2c65a7d2777e5aead3a88c01`. It records shared
rational point images, original guards, native parameter predicates, qualification
and typed exhaustion. All 2,870 Hypercurve tests,64 root tests,3,012 consumer tests,
strict MSRV/fuzz Clippy, core builds, docs,16 matrix guards and213 fixtures pass;
34 existing ignored cases remain. Independent circle/inverse identities, cold
native line reconstruction, exact splitting, original poles and reached-limit
retries are asserted separately. Original failed witnesses and all attempts are
retained. Every non-receipt byte matches final qualification. This local record
is unpushed; complete algebraic operation closure, total resource bounds and
package delivery remain open.

The local `hypercurve-circle-differentials-evidence-2026-10-27` branch ends at
`5eeeec42d607ed5c3126fd396a2150df5be50668`, adding `docs/evidence/2026-10-27-hypercurve-closure-circle-differentials`
(972 files, 48,642,946 bytes), SHA-256 `551fcaa43f070caa54efa25e4591460d6d6fc27e94d1950edbb3a81be3618567`.
It records the shared rational derivative recurrence, selected-circle chart
inversion and exact joint vector images. All 2,859 Hypercurve tests, 64 affected
root tests, 3,012 consumer tests, strict MSRV/fuzz Clippy, core builds, API docs,
16 matrix guards and 205 capability fixtures pass; 34 existing ignored cases
remain. Independent derivative formulas, source-chart transport, inherited
qualification, fresh-process native consumers, true reached-limit retries and
successful uncharged exact queries are separately asserted. The original
Unsupported witness, capacity panic and all failed harness assumptions remain
in the archive. All non-receipt source bytes match final qualification. This
record is local and unpushed; other fields/frames, region admission, complete
operation closure, total resource bounds and package delivery remain open.

The local `hypercurve-selected-circle-points-evidence-2026-10-27` branch ends at
`9cd590ad47e73ca9e1b3fa3ce13307f4d40686c0`, adding `docs/evidence/2026-10-27-hypercurve-closure-selected-circle-points`
(596 files, 29,543,839 bytes), SHA-256 `51d30ce5818c61dfe18aee1f16cd950e00e5115f7b91cd79bac20adbc2156b68`.
It records selected circle-point angular authority, both radial coordinates in
one exact field, signed half selection, ordered close cuts, similarity transport
and native replay. All 2,846 Hypercurve tests, 64 affected root tests, 3,012
consumer tests, strict MSRV/fuzz Clippy, core builds, API docs, 16 matrix guards
and 193 capability witnesses pass; 34 existing ignored cases remain. Original
positive failures, the corrected replay regression, close-cut cost diagnostics,
policy and budget checks, and interrupted quota/session attempts are preserved. Every
non-receipt source byte matches final qualification. The record is local and
unpushed. Public selected-circle derivatives, correlated-center region admission,
general operation closure, total resource bounds and package delivery remain open.
The replay publication archive ends at `e784a8ea0b53cc24f3b56a61d63a13e66b9e7701` with 364 records.
Its new record `docs/evidence/2026-10-28-hypercurve-closure-replay-publication-validation` contains 13,491 files and 41,256,350 bytes,
SHA-256 `da99dd32d0146a81c5e18080e73a3058aa4305338b7b40a45831ccd6b85ea446`. It records fresh source-attributed success for all 328
pre-push command groups on `9d64f5dcab87ee79b5761a6db010d7e84e103b16`: 131 locally executable workflow groups,
196 strict Clippy groups and the CI security audit. All 13 ASan campaigns include
successful mutation evidence. Stable 1.99.1, MSRV 1.95.0, nightly-2026-10-27, both
release/package checks and the hosted-action review are retained. Lost runs from
before the host restart are not credited. All 364 record digests
verify, and four historical archive exclusions remain unchanged. Only three
receipt files differ from the qualified code; they are checked separately.
This supersedes earlier pending pre-push status for the included increments
through retained fiber-point order. Later selected-circle points, differentials,
rational point images, region/replay/contact and corner work are excluded.
Hosted actions and publication/deployment are not established by local checks.

The combined replay publication archive `hypercurve-replay-publication-evidence-2026-10-27` ends at
`7776fbc75fabcba93f1d5e0b1b07dded631b524f` with 363 records. It retains the previously
published pre-push evidence and all separately qualified increments through
common-field point and circle predicates. The combined code requires a fresh
complete pre-push run; the earlier publication receipt applies only to its
named source revision. No historical executions count toward this new run.

The local `hypercurve-fiber-point-order-evidence-2026-10-27` branch ends at
`33f87cb54308298a6a305b0fb6c6f8b133d477b8`, adding `docs/evidence/2026-10-27-hypercurve-closure-fiber-point-order`
(503 files, 19,569,574 bytes), SHA-256 `6f36bec9d2a26a7b7830fdd268ede65c1260e6bfd538218d3f818efe7c43f08a`.
It records shared-field linear/circle predicates, exact half-chart inversion of
proved constant displacements, and checked direction-definition replay. All
2,839 Hypercurve tests, 64 affected root tests, 3,012 dependent-package tests,
strict MSRV and fuzz Clippy, core builds, API docs, formatting, 16 matrix guards
and 188 capability witnesses pass; 34 existing ignored cases remain. Original
positive failures, corrected-source probes, exact geometric expectations,
source/cold/warm replay diagnostics, reached budgets and retry are retained.
Qualification and parent-transition records prove every non-receipt source byte
unchanged. This record is local and unpushed. General selected angular fields,
filled-region archives, total resource bounds, full pre-push qualification and
published-package delivery remain open.

The local `hypercurve-circle-replay-evidence-2026-10-27` branch ends at
`057b04a795ccca9ec8f60099cfcf5b06a01e4292`, adding `docs/evidence/2026-10-27-hypercurve-closure-circle-replay`
(590 files, 27,534,685 bytes), SHA-256 `f0d529e9c2ca5cb332cd43e9ae22f01c48bf6b665b2f5fddc946b2742631413d`.
It records exact selected circle frames, signed contacts, independent incident
selectors, checked chord unit directions and downstream point/region operations.
All 2,832 Hypercurve tests, 64 root tests, 3,012 dependent-package tests, strict
MSRV and fuzz Clippy, builds without optional features, API docs, formatting,
16 matrix guards and 181 capability witnesses pass. The suite retains the same
34 existing ignored cases. Original failures, prototype snapshots and fresh
process replay before further operations are preserved. This record is local
and unpushed. General circle charts, correlated-fiber midpoint ordering,
repeated-offset chart transport, filled-region archives, total resource bounds,
full pre-push checks and published-package delivery remain open.

The local `hypercurve-point-replay-evidence-2026-10-27` branch ends at
`9ed71e03ca34a38db6ec46f5194101181445fbfb`, adding `docs/evidence/2026-10-27-hypercurve-closure-point-replay`
(702 files, 27,425,737 bytes), SHA-256 `e5473caf511936bc1704e8d8ed1251b73bf1e2b17ab078623375a2d8b08f932b`.
It records exact selected point dependency graphs, source-preserving chord
displacements, shared map prefixes, checked point-depth limits and Hyperbrep
planar-point archive admission. All 2,820 Hypercurve tests, 63 root tests,
3,012 dependent-package tests, strict MSRV and fuzz-target Clippy, builds without
optional features, API documentation, formatting, 16 matrix guards and 170
capability witnesses pass. The Hypercurve suite retains 34 existing ignored
cases. Committed-parent failures, failed prototypes, source snapshots and
fresh-process replay before further operations are preserved. This record is
local and unpushed. Complete selected circles and broader generated/filled-region
replay, total resource bounds, full pre-push checks and published-package delivery
remain open.

The local `hypercurve-chord-replay-evidence-2026-10-27` branch ends at
`449fb13ca4931dbd460429664a30846e201dea0c`, adding `docs/evidence/2026-10-27-hypercurve-closure-chord-replay`
(555 files, 18,335,290 bytes), SHA-256 `79716febf65ec101df08a8ed67ed3d5411e2b003c616c4a02acc5c04575e1d67`.
It records exact selected-chord replay, shared point/source/root and endpoint
relations, validated support charts, and deterministic analytic endpoint
projection after exact queries. All 2,809 Hypercurve tests, 62 root tests,
3,010 dependent-package tests, strict MSRV Clippy, the build without optional
features, API documentation, formatting, 16 matrix guards and 160 capability
witnesses pass. The Hypercurve suite retains 34 existing ignored cases.
Committed-parent failures, failed prototypes, source snapshots and fresh-process
replay before further operations are preserved. This record is local and
unpushed. Broader generated and filled-region replay, total resource bounds,
full pre-push checks and published-package delivery remain open.

The local `hypercurve-parallel-replay-evidence-2026-10-27` branch ends at
`368840d8c3c4e3ca489870a56f8642c10122ff42`, adding `docs/evidence/2026-10-27-hypercurve-closure-parallel-replay`
(426 files, 13,660,333 bytes), SHA-256 `5b0a3f0c28cb0a70f0779f98ab2da108b2078fe5423008e50a92cd37f47f54fe`.
It records native procedural-parallel replay, retained source/range/normal-side
relations and finite projection on exterior or subdivided intervals.
All 2,799 Hypercurve tests, 61 root tests, 3,010 dependent-package tests,
strict MSRV Clippy, the build without optional features, API documentation,
formatting, 16 matrix guards and 151 capability witnesses pass. The Hypercurve
suite retains 34 existing ignored cases. Committed-parent export/projection
failures, failed prototypes, source snapshots and separate-process replay before
further region operations are retained. This record is local and unpushed.
Full pre-push checks, broader generated/filled-region replay, general algebraic
closure and total resource bounds remain open.

The local `hypercurve-native-replay-evidence-2026-10-27` branch ends at
`e0d8165fae971704511ba1f27fd6286eff408f6b`, adding `docs/evidence/2026-10-27-hypercurve-closure-native-replay`
(289 files, 8,114,777 bytes), SHA-256 `14268be5ef2fa70f0f4a0ff8f5879aa52885e5b2c335c3f42cab6b7ce99bf2dd`.
It records exact native source restrictions, selected-root and projective-chart
replay, retained-domain finite projection and root adapter error transport.
All 2,790 Hypercurve tests, 60 root tests, 3,010 dependent-package tests,
strict MSRV Clippy, the build without optional features, API documentation,
formatting, 16 matrix guards and 142 capability witnesses pass. The Hypercurve
suite retains 34 existing ignored cases. Original export/projection failures,
failed prototypes and qualification, source snapshots and fresh-process replay
evidence are retained. This record is local and unpushed. Full pre-push checks,
broader generated/filled-region replay and general algebraic closure remain open.

The local `hypercurve-algebraic-pair-evidence-2026-10-27` branch ends at
`8b0caa9be41bdacd1f40bd9f60b7b9da3f339962`, adding `docs/evidence/2026-10-27-hypercurve-closure-algebraic-pair`
(567 files, 23,612,610 bytes), SHA-256 `af8f3c08003180038a0db21aa3bbdf5e78fb86a7861c5901c5b43248e2e8703d`.
It records shared algebraic similarity incidence, selected-endpoint chord
source charts, projective common-fiber equations and strict selected-field
admission. All 1,233 Hypersolve tests, 2,781 Hypercurve tests, 43 root downstream
tests, 3,605 tests across nine dependent packages, strict MSRV Clippy,
formatting, 16 matrix guards and 133 capability witnesses pass. The Hypercurve
suite retains 34 existing ignored cases; standalone exactCore comparisons retain
19 existing oracle-defect cases. Original unsupported, undecided and timeout
witnesses, both failed qualifications, unchanged regression tests, the minimized
generated seed, and the repaired two-crate comparison inventory are archived.
Per-command source snapshots distinguish the metadata repair from the unchanged
kernel inputs. This record is local and
unpushed; full pre-push qualification, general algebraic closure and native
fresh-process replay remain open.

The local `hypercurve-similarity-policy-evidence-2026-10-27` branch
ends at `5c1494e708b19575a7616ee58443d4acf07350f1`, adding `docs/evidence/2026-10-27-hypercurve-closure-similarity-policy`
(242 files, 6,278,223 bytes), SHA-256 `db95b27ddd9dff1a5f89d3e234a663ea7e66140af57edd77b708c033fb95537c`.
It records policy retention for selected polynomial similarity points and
endpoints: locally certified outputs remain usable under STRICT, while nested
approximate dependencies keep their replay barriers. 2,773 Hypercurve tests,
43 root downstream tests, 3,010 tests across six dependent packages,
strict MSRV Clippy, formatting, 16 matrix guards and 125 capability witnesses
pass. Original failed witnesses, the unchanged generic policy-barrier check,
a consumed-receipt dependency witness and raw commands remain archived.
Point replay here is checked in memory. This record is local and unpushed;
full pre-push qualification and broader algebraic closure remain open.

The local `hypercurve-selected-integration-evidence-2026-10-27` branch
ends at `04eccb7d4eccb1928dc4e1c2998174cd4a6f27c4`, adding `docs/evidence/2026-10-27-hypercurve-closure-selected-composition`
(344 files, 13,220,752 bytes), SHA-256 `309880681d97f5bf8760866eea8009a38f0d1764bb4340f13e136d73e8da4499`.
It retains all three parent evidence histories and qualifies their combined
selected-branch compositions through exact face probes, original source/root/map
relations, transforms, disconnected components and further operation on empty
results. 1,223 Hypersolve tests, 2,770 Hypercurve tests, 43 root downstream tests,
3,010 dependent-package tests, strict MSRV Clippy, formatting, 16 matrix
guards and 123 capability witnesses pass. Failed attempts and rejected probe
changes remain archived. This record is local and unpushed; full repository
pre-push validation remains pending. General algebraic closure and native
region replay remain open.

The integration evidence branch now ends at `de84df74612abfd1e4d447e4827e42ef46f302a6`,
retaining 351 records including the independently qualified algebraic
similarity increment. Combined source and composition qualification
remain pending; these parent records do not establish that result.

The local `hypercurve-algebraic-similarity-evidence-2026-10-27` branch
ends at `1216d6408218a4746703408ea0e51832ae4fed62`, adding `docs/evidence/2026-10-27-hypercurve-closure-algebraic-similarities`
(202 files, 4,910,607 bytes), SHA-256 `cb775d3b839b79110b7dca2f73d291d91217f19dc11e1707d6d9ca10bdf99612`.
It records native-line sections through retained algebraic similarities with
original selected polynomial sources preserved. 2,745 Hypercurve tests,
43 root downstream tests, 3,009 tests across six dependent packages,
strict MSRV Clippy, formatting, 16 matrix guards and 98 capability witnesses
pass. Expanded equations, exact parameters, tangency, overlap continuation,
selected trims, distinct nearby supports and actual exhaustion/retry are covered.
Original failed witnesses and raw source-attributed commands remain archived.
This record is local and unpushed. General image and algebraic operation closure
and fresh-process native replay remain open; full pre-push qualification is
still required before publication.

The local `hypercurve-selected-integration-evidence-2026-10-27` branch
at `c81fae2f7fb6c17844f8d9eeb51d0971e5f8ffe6` combines 350 archived records from the
qualified region and horizontal-boundary increments. The individual
receipts below retain their original source scope. Combined integration
checks and repository pre-push validation remain pending.

The separate local `hypercurve-polynomial-region-evidence-2026-10-27`
branch ends at `aceb9add07aaaa060aabac0f31c98154d82a7d96`, adding
`docs/evidence/2026-10-27-hypercurve-closure-polynomial-regions` (564 files,
26,364,613 bytes), SHA-256
`6f2a9bdaf22a1b19e14126ff46325060a0532769608178d363979ea008f51c9b`.
It records selected polynomial supports in the shared filled arrangement:
2,754 Hypercurve tests, 43 root downstream tests, 3,009 tests across six
dependent packages, strict MSRV Clippy, formatting, 16 matrix guards and 107
capability witnesses pass. The unchanged H08 boundary and original failing
Boolean/open-clip cases remain, alongside holes, disconnected material,
reversal, mixed circles, exact selected trims, cusp/node incidence and actual
exhaustion/retry. Qualification used `5de7f0cb05` plus the frozen region
increment; non-receipt bytes remain unchanged. The original failed attempts and
the corrected quintic test oracle remain archived. General implicit operations,
nonzero offsets/corners and fresh-process replay remain open. This record is
local and unpushed; the separate complete repository publication run excludes
this increment, whose full pre-push validation remains pending.

The local `hypercurve-horizontal-boundary-evidence-2026-10-27` branch
ends at `84c4bd3d979ac681096e69a9c1a35f9e1d4ac681`, adding `docs/evidence/2026-10-27-hypercurve-closure-horizontal-boundaries`
(404 files, 16,033,938 bytes), SHA-256 `3f1bfd095d5be2dfeca96045eaa0815ac6b5544b4e045fcfb4be5af0c9547b52`.
It records complete closed horizontal boundary components in the existing
finite-fiber topology, with original equations and branch incidence retained.
1,223 Hypersolve tests, 2,744 Hypercurve tests, 43 root downstream tests,
3,010 dependent-package tests, strict MSRV Clippy, formatting, 16 matrix
guards and 97 capability witnesses pass. Selected trims, overlap and further
intersection, exact source replay in a fresh process, and the original BRep
boundary section through native/affine replay are covered. Earlier failed
attempts remain archived. Vertical/repeated components, algebraic coefficient
fields and general closure remain open. This record is local and unpushed;
the full repository pre-push validation of this increment remains pending.

The separate local `hypercurve-polynomial-location-evidence-2026-10-26`
branch ends at `98fd9084af3539fabee8dbedc1ffe300935f46e9`, adding
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-locations` (227 files,
6,750,543 bytes), SHA-256
`18441451082feb7c408e2951a88f80a9b84e2a05eb699cb35de72f0e8d1433d9`.
It records selected polynomial point incidence in original charts and invertible
similarities: 2,738 Hypercurve tests, 43 root downstream tests, 3,009 tests
across six dependent packages, strict MSRV Clippy, formatting, 16 matrix guards
and 91 capability witnesses pass. The three original Unsupported fixtures,
independent transformed equations, selected trims and exhaustion/retry remain
archived. Qualification used `09ce677746` plus frozen prerequisite and
point-location bytes, preserved when advancing the parent to `d7ae8ea218`.
General preimages, polynomial regions and fresh-process replay remain open.
This local record is unpushed; full repository pre-push qualification is pending.

The combined publication archive ends at `cafb43c3cf6a9ac5577d14fb97c7899b05fd6add` with 350 records.
Its new record `docs/evidence/2026-10-26-hypercurve-closure-validation-expanded` contains
14,368 files and 43,715,836 bytes, SHA-256
`6e43b5a833fff1a7c2f6c99e21741693681f7d91b4440a7061699f54e8e25cc3`. It records fresh, source-attributed
success for all 328 pre-push command groups on `b1c011a070e61970fc7ee15a0c206b2f3b84e5f0`:
131 locally executable workflow groups, 196 strict Clippy groups and the CI
security audit. All 13 ASan campaigns include successful mutation evidence.
Stable 1.99.1, MSRV 1.95.0, nightly-2026-10-27, both release/package checks and the
hosted-action review are retained. All 350 record digests verify; four historical
third-party archive exclusions remain unchanged. The three documentation files
carrying this receipt are checked separately; all other tracked bytes equal the
qualified revision. This completes the publication candidate's local pre-push
qualification and supersedes earlier pending status for its included increments.
Later point-location, polynomial-region, horizontal-boundary, algebraic-similarity
and integration changes are excluded. Hosted actions and actual
publication/deployment are not established by local validation.

The combined local `hypercurve-publication-evidence-2026-10-26` archive ends at
`15af9d0b78ec35c82bf00217eec4dd25c46c5918` and retains 349 records.

The complete command inventory was attempted against `904e5408e1`: 324 of
325 groups passed. The unchanged-value voxel assertion failed, so that revision
was not published. Both release/package checks and all 13 ASan campaigns passed;
the unsuccessful run remains evidence of an incomplete qualification attempt.
Its record, `docs/evidence/2026-10-26-hypercurve-closure-validation-attempt-904e`,
contains 12,786 files and 36,965,435 bytes, SHA-256
`94d4948defd834cd3b4984d64196a6063eab0038b9bde0454341a86c347951a4`.
The combined local archive retains this attempt and all independently qualified
follow-on records. A fresh 328-group pre-push run is required for the combined
code candidate; no previous command execution is counted toward that run.

The local `hypercurve-voxel-write-oracle-evidence-2026-10-26` branch ends at
`e5b2ee8217a3b9254b30130aacbbccbc0001ccc6`. Its new
`docs/evidence/2026-10-26-hypercurve-closure-voxel-write-oracle` record contains
24 files and 568,359 bytes, SHA-256
`32e6bc134c56648d70bd45a26e1032fd4cd07623b7b1c138cb9f74bfc26b7a4a`.
The full pre-push attempt at `904e5408e1` exposed an invalid random-write test
expectation for unchanged voxel values. The deterministic failing seed and the
original full random workload now use an independent dense-array change oracle.
All 116 nested workspace tests, 78 Hypervoxel consumer tests, strict Voxelis
Clippy, formatting and repository hygiene checks pass. This is targeted local
qualification; publication remains blocked pending the complete validation set.

The separate local `hypercurve-rational-section-evidence-2026-10-26` branch
ends at `a4abd246af4e572dbf62a7c139f127fd84bd085b`, adding
`docs/evidence/2026-10-26-hypercurve-closure-rational-sections` (442 files,
15,497,091 bytes), SHA-256
`c55babd43904c40db014bf6053f8aa1fceaad9a4d2a30fdc1d9f2156a8ab5e48`.
It records selected polynomial/rational finite sections and common-component
correspondences: 2,730 Hypercurve tests, 43 root downstream tests, 3,009
tests across six dependent packages, strict MSRV Clippy, formatting, 16 matrix
guards and 83 capability witnesses pass. Original Unsupported cases and the
subsequent spline, transform and cusp findings remain archived. Qualification
used `09ce677746` plus frozen abscissa and rational-section inputs, preserved
when advancing the parent to `80bcc08ae9`. Constant-abscissa common traces,
broader algebraic charts and polynomial regions remain open. This local record
is unpushed; full repository pre-push qualification remains pending.

The separate local `hypercurve-selected-abscissa-evidence-2026-10-26` branch
ends at `9008c6b87b51bbb644a9dc4a49d0fadfd52736e2`, adding
`docs/evidence/2026-10-26-hypercurve-closure-selected-abscissas` (213 files,
7,201,989 bytes), SHA-256
`1e45871c00e897e88d579812c2bb87e974e9ad9a356f6e36d6cae467992a4014`.
It records retained rational abscissa pullback and repeated output-coordinate
use: 1,213 Hypersolve tests, 2,710 Hypercurve tests, 43 root downstream tests,
3,009 tests across six dependent packages, strict MSRV Clippy for both owners,
formatting, 16 matrix guards and 66 capability witnesses pass. The original
generic-quintic evaluation failure and the output-coordinate follow-on failure
remain archived. Qualification used `00dbdb152a` plus frozen generated-line and
abscissa inputs, preserved when advancing the parent to `09ce677746`.
General dependent-fiber charts and polynomial regions remain open. This local
record is unpushed; full repository pre-push qualification remains pending.

The separate local `hypercurve-polynomial-chord-evidence-2026-10-26` branch
ends at `3f85a12099eeab4e6c8af0f14335701637a5d0e9`, adding
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-chords` (273 files,
9,941,360 bytes), SHA-256
`07ec6f93dc278782911c1756d62550e489a493df418ab3efdfdb75193f6cc35d`.
It records generated polynomial-line incidence, shared chord predicates and
retained parameter correspondence: 2,700 Hypercurve tests, 43 root downstream
tests, 3,009 tests across six dependent packages, strict MSRV Clippy,
formatting, 16 matrix guards and 56 capability witnesses pass. Original generated
blockers, independent point-location failure, transformed support failure and
source snapshots remain archived. Qualification used `f0fa18b7b8` plus frozen
pair and generated-line inputs, preserved when advancing the parent to
`00dbdb152a`. General selected supports, broader pair families and polynomial
regions remain open. The record is local and unpushed; full repository pre-push
qualification for this increment remains pending.

The separate local `hypercurve-polynomial-pair-evidence-2026-10-26` branch
ends at `aefddcc9b36a3efee20293d37f39e98606c2d6c2`, adding
`docs/evidence/2026-10-26-hypercurve-closure-polynomial-pairs` (165 files,
3,871,244 bytes), SHA-256
`3a1cc48424006b37ad2c2be99587216ef49b9d48e0db8acd31fcdea986ef7b13`.
It records selected polynomial-pair incidence and exact source correspondence:
2,690 Hypercurve tests, 43 root downstream tests, 3,009 tests across six
direct dependent packages, strict MSRV Clippy, formatting, 16 matrix guards and
46 capability witnesses pass. The original blockers, mixed-contact scalar
failure, incorrect negative-range prototype, shared rational transforms,
degree-37 continuation and exact source attribution are retained. Qualification
used `fd1313a23a9` plus the frozen predicate and pair inputs; their bytes were
preserved when advancing the parent to `f0fa18b7b8`. Broader images, filled
polynomial regions, general coefficient fields and fresh-process native replay
remain open. The record is local and unpushed; full repository pre-push
qualification for this increment remains pending.

The separate local `hypercurve-fiber-order-evidence-2026-10-26` branch
ends at `1761ca45750dd215a45dd150e08393a8b50c64ac`, adding
`docs/evidence/2026-10-26-hypercurve-closure-fiber-order` (116 files,
3,051,934 bytes), SHA-256
`a82d1aafbb3ce0ce84f909eb99da5e5ae35b9bf76c82c2382847d05ae64d4435`.
It records selected-fiber ordering and retained native evidence at `fd1313a23a9`
plus the predicate patch: 2,681 Hypercurve tests, 43 root downstream tests,
3,009 tests across six direct dependent packages, strict MSRV Clippy,
formatting, 16 matrix guards and 37 capability witnesses pass. Original counted
exhaustion, independent native-evidence regressions, source snapshots and
qualification commands are retained. Point replay remains in memory, and
counted work is not a total resource bound. The record is local and unpushed;
full repository pre-push qualification for this increment remains pending.

The separate local `hypercurve-algebraic-line-evidence-2026-10-26` branch
ends at `2d8749f6d70c8805fd96c0d4e1fefd83d59c55e2`, adding
`docs/evidence/2026-10-26-hypercurve-closure-algebraic-lines` (94 files,
2,666,477 bytes), SHA-256
`3404dc901761f282c45021ad7d399095a624a2f5961203a38cdc45734a88aec2`.
It records native algebraic-coordinate line sections and certified coefficient
decisions: 2,676 Hypercurve tests, 43 downstream tests, strict MSRV Clippy,
formatting, 16 matrix guards and all 34 capability witnesses pass. Original
blockers, a reproduced quartic false-empty prototype, exact source hashes and
final qualification are retained. Qualification used `45947259af` plus the
frozen component and algebraic-line inputs; source bytes were preserved when
advancing the parent to `9da58d3ac3`. Selected chord carriers, broader maps,
branch pairs and native archive replay remain gaps. This record is local and
unpushed. Full repository pre-push qualification remains pending; the frozen
`904e5408e1` run does not cover it.

The separate local `hypercurve-line-components-evidence-2026-10-26` branch
ends at `3f3a1ef5416f882095d3b3fa7b6d386d818d9f22`, adding
`docs/evidence/2026-10-26-hypercurve-closure-line-components` (84 files,
2,961,879 bytes), SHA-256
`9bc06b8764a2564c2ccd5f4ce42b3e952db4d0edc5e254116c273c3c0cfcf5c4`.
It records selected common-line overlaps and other-branch contacts against
`45947259af`: 2,669 Hypercurve tests, 43 downstream tests, strict MSRV Clippy,
formatting, 16 matrix guards and all 27 capability witnesses pass. Exact trim
correspondences, regular branch contacts at singular points, policy changes
and a reached-budget retry are qualified within the recorded native-line
domain. Original blockers, source snapshots and the reviewed selector prototype
are retained; the latter is not a reproduced runtime counterexample.
This record is local and unpushed. Full repository pre-push qualification
remains pending; the frozen `904e5408e1` run does not cover it.

The separate local `hypercurve-mapped-polynomial-evidence-2026-10-26` branch
ends at `f763399f1656e9596f9e63d13661feb89e3ee85b`, adding
`docs/evidence/2026-10-26-hypercurve-closure-mapped-polynomial` (76 files,
2,412,312 bytes), SHA-256
`81fd5fbcd665b34092c65e8e914400cdb35157a285d7545e84d3e45700f6f91d`.
It records rational-similarity polynomial/line sections and consumption of
their selected-point relations against `b1af6465b3`: 2,664 Hypercurve tests,
43 downstream tests, strict MSRV Clippy, formatting, 16 matrix guards and all
22 capability witnesses pass. The original translated quintic, selected-line
comparison failures, source snapshots and final qualification are retained.
Continuous line components remain unsupported at this increment; the separate
prototype is excluded. This record is local and unpushed. Full repository
pre-push qualification remains pending; the frozen `904e5408e1` run does not
cover this source.

The separate local `hypercurve-capability-matrix-evidence-2026-10-26` branch
ends at `a74b62c4fdd8dd3c361d9f175cfa947c7c1a5635`, adding
`docs/evidence/2026-10-26-hypercurve-capability-matrix` (98 files,
1,781,381 bytes), SHA-256
`355b93d081144f9633a9c16c202ef7d6c86e61b240982fd6278ddcf3ceae2f65`.
It records the initial 18-fixture M0 catalog, 16 evidence-guard tests and two
successful source-attributed invocations against `fa312fd849` plus the driver
patch. Unsupported, exhausted, invalid, blocked and qualified assertions remain
separate from exact completion. Formatting, typos, budget and the required
Zizmor command pass; Zizmor's online-only audits need an unavailable API token.
The hosted upload was inspected only. This record is local and unpushed; full
repository pre-push qualification for this increment remains pending.

The separate local `hypercurve-native-pair-evidence-2026-10-26` branch
ends at `b98f70fa0673c4fd165bd118a68d8ee1d9313097`, adding
`docs/evidence/2026-10-26-hypercurve-closure-native-pairs` (31 files,
2,029,711 bytes), SHA-256
`9c2aee95e8caaf702d91d45e2eed32cf5c6359baf543dcb4dcfa47a179479b54`.
It records native line/circle parallel pair consumption against `515e7647a9`:
2,660 Hypercurve tests and 43 downstream tests pass, with strict MSRV Clippy
and formatting. The unchanged arc-corner composition completes in a fresh
process in 13.64 s and is promoted to an ordinary test. The stationary-normal
counterexample, blocked baseline and source-equivalence proof are retained.
This record is local and unpushed; its full repository pre-push qualification
remains pending. The separate frozen `904e5408e1` run does not cover it.

The separate local `hypercurve-line-fragment-evidence-2026-10-26` branch
ends at `baf93f7321f69b0870da921b738f6549294fd8fb`, adding
`docs/evidence/2026-10-26-hypercurve-closure-line-fragments` (44 files,
521,732 bytes), SHA-256
`e57ae46ca6ccd5a0b405d96ee831fc61c9d6ab96c179f0b2215dc851390ceaf8`.
It records exact line support, orientation and signed offset transport against
`13c3e18375`: 2,655 Hypercurve tests and 43 downstream tests pass, with
formatting and strict MSRV Clippy. The isolated original erosion remains
blocked. The separately retained experimental completion includes native pair
changes outside this source increment. This record is local and unpushed;
its full repository pre-push qualification remains pending. The frozen
`904e5408e1` validation run does not cover this increment.

The separate local `hypercurve-contact-evidence-2026-10-26` branch ends at
`2783b4aa0e1e029b9dcc9d2200e246eb7f095444`, adding
`docs/evidence/2026-10-26-hypercurve-closure-native-contact-evidence`
(12 files, 329,667 bytes), SHA-256
`b2c6714712ce3b03bef818b23c7422b972e5dbe99f9e22166e5d0f04a9469075`.
It records native line/line and line/arc oriented contact evidence against
`904e5408e1`: 2,648 Hypercurve tests and 43 downstream tests pass, as do
formatting and strict MSRV Clippy. The original erosion remains blocked.
This increment is committed locally, has not been pushed, and awaits its own
full pre-push qualification. It is separate from the frozen `904e5408e1`
validation run and is not covered by that run.

The local `hypercurve-closure-2026-10-26` branch now ends at
`2550c23fd100d06fb2343323ae8273ea97a70164`, adding
`docs/evidence/2026-10-26-hypercurve-closure-line-endpoints` (13 files,
302,594 bytes), SHA-256
`ebd052c2f5a3bf22d9e31e016aa2245f86ef29eaeb07a72e1fd5451449e2c717`.
It records native line proof transport across equal endpoint substitution
against `07f1157275`: 2,647 Hypercurve tests and 43 downstream tests pass,
as do formatting and strict MSRV Clippy. The original erosion remains blocked,
and full pre-push qualification is pending. This record is committed locally
and has not yet been pushed.

Its local parent,
`38f2b160874308b674e02fc5bd53e9bb71484add`, adds
`docs/evidence/2026-10-26-hypercurve-closure-band-endpoints` (9 files,
301,329 bytes), SHA-256
`32eae2f8d8901d2c6d4ecc4f68a0dd3b0ffa81ed829072650e36424bd65bb0e0`.
This records exact endpoint sharing in offset bands against `532e517b07`,
with 2,646 Hypercurve tests and 43 downstream tests passing, plus formatting
and strict MSRV Clippy. The full original erosion case remains blocked. This
record is committed locally and has not yet been pushed.

Its local parent adds commit
`586bebeb9655773597eb3a0dd21d28b83ff88ea3` for
`docs/evidence/2026-10-26-hypercurve-closure-fillet-contacts` (42 files,
844,716 bytes), with SHA-256
`00ef55209fa9f09e862504fb1bec9613ef517b4f4d5d76b07c83d5437df92f0f`.
It records fillet contact publication, noncollapsed concentric evidence
transport, circular-source incidence and represented contact handoff against
base `4ace9a71a2`. The final Hypercurve release suite passes 2,645 tests
(35 existing ignored), all 43 downstream checks pass, and MSRV strict Clippy
and formatting pass. Failed diagnostic and resource attempts remain separate.
The full original arc-corner erosion remains blocked; no performance, full
closure or new pre-push completion claim is made. This record is committed
locally and has not yet been pushed.

Its published parent,
`6c50c8ac3f0cc4cb04a9f024a0677bd039d6eefb`, adds
`docs/evidence/2026-10-26-hypercurve-closure-validation` (12,153 files,
58,233,737 bytes), with SHA-256
`78c71e823c48c8f104fde6290654f66fa54452b8e50043be7380cb534b9fc137`.
It qualifies code `4b5bcdde506703e65f593a8006fde6e808e800a5`: all 129 locally
executable workflow steps and all 196 strict Clippy configurations completed,
with per-command source and dependency attribution. The full Hypercurve suite
passes 2,639 tests (35 existing ignored), and 43 downstream checks pass.
Four hosted configuration/upload/deployment actions were inspected but excluded;
local package checks do not establish registry publication or hosted deployment.
Failed sandbox/offline attempts and
cancelled duplicates remain separate from their successful replacements.
The sanitizer report's older nightly-alias metadata is corrected by the
archived Cargo invocation and executable compiler evidence. These executions
establish no performance improvement or complete curve-closure claim.

Its parent, `895722357d6acde2b82fbeee40b5e5856adb75c4`, adds `docs/evidence/2026-10-26-hypercurve-closure-native-circle` (10 files,
80,015 bytes), with SHA-256
`1e1bf512fe517ae7a60e3440ebe0115d4690c7bb03cbe600accad3c49829c68f`.
It records the corrected native-promotion failure, retained circular contact
transport, seven passing circle tests, Clippy and formatting. The original
arc-corner erosion remains blocked; full-suite and downstream qualification
is completed in the validation record above. The two discarded attempts using
an incorrectly initialized test certificate are explicitly separated from the valid baseline.

Its parent, `b70ae9defe0d86009bd8f5d16d28a0fa8ecab574`, adds
`docs/evidence/2026-10-26-hypercurve-closure-native-area` (9 files,
202,715 bytes), with SHA-256
`b73c39dff6499757d7d9e00ed4b8e275209b4bfe2f1de1c86d108912692cee3e`.
It records the storage-dependent CPArti assertion and extends its unchanged
geometry/area fixture through both native and retained views and following
Booleans. Focused checks pass; full-suite and pre-push validation are recorded
above. Production code is unchanged by this qualification extension.

Its parent, `770d49fd53a2bfec1943af54b417b2dface7a596`, adds
`docs/evidence/2026-10-26-hypercurve-closure-m1-circle` (12 files, 310,497 bytes),
with SHA-256
`52609b6ca3dc51023d2ecbec117bd91a9f249147851d81bb762d1cfcd7a39b38`.
It retains the missing-circle reproduction, transform repair, passing circle,
PCB and downstream checks, and the unchanged arc-corner erosion blocker.
The complete release suite was still running at this checkpoint. This local
record does not claim pre-push validation, performance improvement, or completed
closure; its parent is the retained-line record below.

The `hypercurve-closure-2026-10-26` branch adds commit
`8399bc8af813a5624f1db2152d40f7b7e1cb7e1c` for
`docs/evidence/2026-10-26-hypercurve-closure-m1` (10 files, 32,446 bytes), with
SHA-256 `0d07f14b2b32e5dea5b3488090871802157e57b40a6dbde74990a5642363d9a6`.
It records the retained-line transform repair, passing focused and downstream
checks, and the then-failing follow-on XOR. This incremental
record makes no full closure or pre-push validation claim. It adds only its own
record; its parent is the research closeout below.

The published [`research-2026-10-25` branch](https://github.com/timschmidt/csgrs-evidence/tree/research-2026-10-25)
ends at `cd8bf060aa577231f84fffd7cf21e15127c028b0`, adding
`docs/evidence/2026-10-26-research-closeout` (12,763 files, 187,318,255 bytes), with
SHA-256 `ef84e4aed934ad5014ed36b27a41951d58b2200ab82a5cee3e7ac39e2ba005ad`.
All eighteen research and closeout records verify; the earlier records and
historical `main` are unchanged.

The closeout record covers all 129 locally executable workflow steps and 196
strict Clippy checks, including the 256 Hypertri feature combinations,
stable/MSRV packaging and real external consumers, coverage, directed and
sanitizer campaigns, and release WASM demos. Its validation admission identifies
final code `5e7a430c6b93d6889a6b83ee88873b89d6caf5c1` and the unchanged compiled
inputs at earlier checkpoints. Four GitHub-hosted configuration, upload and
deployment actions were inspected but cannot run locally. Local packaging does
not establish registry publication or hosted docs.rs/deployment success.

R19 remains the final research experiment. The pre-push benchmark executions
are validation under concurrent load, not new performance-admission evidence.
The closure plans and agent instructions are committed requirements; they do
not establish implementation completeness. Earlier paragraphs below describe
the original local checkpoints; all are included in the published branch.

The preceding local follow-on commit was
`55d28790bdcf32b6454d34a2bfd5ed826f187d72`, adding
`docs/evidence/2026-10-25-research-r19` (16,755 files, 76,625,900 bytes), with
SHA-256 `e916615b99a96c59a6cc5f9d764271d17efcbe894a2fce762a5d7d2ee7fe0ea0`.
It records cold exact-domain continuation, 303 newly exact public consumer
cells, unchanged cold BRep archive replay, failed prototypes, resource limits,
all 20 passing final checks, and the measured adverse construction costs.
Performance-only admission fails. All seventeen research records verify at
this commit; historical paths and published `main` remain unchanged.

Its parent is
`e9b57b21802b18371d062a1a38303cbc8176acc5`, adding
`docs/evidence/2026-10-25-research-r18` (4,832 files, 42,607,582 bytes), with
SHA-256 `9d56eef5f86d80aef6059d8bf2dea42f5fcadeeec301885f11e6acdfb9f6f0f1`.
It records the scoped compound-rotation completion, independent predicate and
cube proofs, failed candidates, native and allocation comparisons, and all
20 passing final checks. The protected latency gate fails; unfavorable costs
remain in the record. Large text artifacts are losslessly compressed with
verified reassembly, and compiled binaries are excluded with digests. All
sixteen local research records verify at this commit; historical paths and
published `main` remain unchanged.

Its parent is
`6b9323f1aed17c0397d973cdb80b73541fb3a1d2`, adding
`docs/evidence/2026-10-25-research-r17` (3,915 files, 395,841,565 bytes), with
SHA-256 `d84deab1807baf7215662ae22f0aa517dbf449fcb139c9391fa3b90c0e00874f`.
It records the complete Nano allocation trace, ownership decoder and controls,
evaluation-cell census, rejected lazy-slot layout, native timing pairs and
passing retained-test validation. The original compressed trace is split into
32 MiB parts with verified reassembly; regenerable partial folded reports and
compiled binaries are excluded with digests. A compact exact multiset preserves
the independent Heaptrack weight comparison. All fifteen local research records
verify at this commit; historical paths and published `main` remain unchanged.

Its parent is
`cd833ffd7f8e04d75913cbd063c9009b5bd0662a`, adding
`docs/evidence/2026-10-25-research-r16` (11,064 files, 93,931,201 bytes), with
SHA-256 `b86159f76af7def0a1676370a48de2f73cae34ca1ffc79e666fca2f654f54565`.
It records exact selected-fiber captures, independent factor/embedding proofs,
both rejected quotient consumers, the controlled timing comparison, original
failed attempts and corrections, and retained test/documentation validation.
Shared graph exports are losslessly compressed; byte-exact replay verifies
the redundant expanded exports omitted from the archive. The fourteen earlier local research records verify at that commit.

Earlier commit `bdded6ef9d6761918276852e96c64c4b2a986e91` added
`docs/evidence/2026-10-25-research-r28` (4,248 files, 22,826,945 bytes), with
SHA-256 `65c784d87d3d6fea819f48c12de1d6100e1dc11a2489884426a90de31fe1e477`.
It retains the rejected circular line-fit shortcut, native/allocation pairs,
independent oracles, passing retained-change checks and diagnostic circle/Nano
traces. The two large Nano logs are losslessly compressed with verified digests.

The 25 October research record is committed locally as
`2f08663201ff859be36eeeabf5745d5916d2a555`, branch
`research-2026-10-25`, in the sibling `csgrs-evidence.git` repository. It adds
`docs/evidence/2026-10-25-research` (586 files, 2,269,593 bytes) without changing
any historical path. Its SHA-256 in `evidence-archive.tsv` is
`7e9cd5280eefa5160ee60801ff2cb4efd7d1a2ae90d300db0f7a74c42d800643`.
At this checkpoint the branch had not been pushed; the published historical
archive above remained at `b20000e377626a41b6ff8e6f87aee2b20ebd7865`.

This record contains the R01 mesh evaluation, the R02 production reproduction,
and recovered original H13–H15 material with new independent verification.

The same local branch now also contains commit
`1a77f07e8f166bb2455b5f3f5117e5c51a3219eb`, which adds
`docs/evidence/2026-10-25-research-r02-r25` (97 files, 1,187,293 bytes).
Its SHA-256 is
`3715724b34398e9443db0d38aed749c3f2eba130b210c5db5e924b531d17cb6e`.
It retains the rejected R02 variants, the R03 timeout, R04 mixed-profile
reproductions, and R25 build/run records and interruption checks.

Commit `a1c1182802aae8bd43ca825da95e67f262cd7a71` adds
`docs/evidence/2026-10-25-research-r04` (838 files, 16,892,204 bytes), with
SHA-256 `349cfa2256f7d156d1efd0629b1a3d4ef959b05734e8198a7e82c3aba5a67cb9`.
It retains the R04 implementation, eight baseline failures, final acceptance
and binding checks, and all paired exact rows and allocation measurements.
Commit `14d94946da060d8cb215cb161f37efbe6c9bb314` adds
`docs/evidence/2026-10-25-research-r11` (1,690 files, 16,013,219 bytes), with
SHA-256 `e558e4a5395098e82ae0a2796cc58013ace58676d33466b1eb902ad358fc8b2c`.
It retains the R11 baseline, both prototypes, 2,400 paired results, proof
traces, exact rows and local/binding/BRep/PCB acceptance checks.

Commit `c916f48bf254ab943d3a68268213af5d824924df` adds
`docs/evidence/2026-10-25-research-r05` (2,486 files, 8,861,617 bytes), with
SHA-256 `2bcd10864cf3bf00756c2199df8ebe7006226b29800556e7e77d3356e036e32b`.
It retains the R05 shared-cut baseline and implementation, the initial and
extended paired measurements, exact definitions and all affected acceptance
checks, including existing policy blockers and adverse controls.

Commit `5e749b3e83710a56eece75e6242cea7302107c1e` adds
`docs/evidence/2026-10-25-research-r08` (135 files, 2,815,808 bytes), with
SHA-256 `4df916c2722e2078566cf11d8f2d700e5036a05ffb10e95e69f613f48267bcd9`.
It retains the R08 typed policy failures, qualified affine continuation,
five board query outcomes, rejected native bypass, and local/downstream checks,
including the full seeded Hypercurve suite (2,607 passed, 35 existing ignored).

Commit `e72d70d97cd87a590d17a0d6604b7d016be5be41` adds
`docs/evidence/2026-10-25-research-r13` (38 files, 897,550 bytes), with
SHA-256 `2ed9e47b8d6d725d80826b67196711a3aee70a36013de1101e879d323658dc7f`.
It retains R13's original exploratory trace and the dedicated-build evaluation:
186 passing tests, 133 successor reconstruction sites, no single unmatched
pair before or after tangent construction, and no production change retained.

Commit `8e48e5ce1c98476c61bfa406304dc3ce7158b324` adds
`docs/evidence/2026-10-25-research-r12` (4,507 files, 46,627,949 bytes), with
SHA-256 `db661d469da8d93c2a870847904ef4f1e397ef511045775afa5f7efb0a64e304`.
It retains R12's rejected tangent-sharing experiment, independent oracles,
19,200 completed isolated comparisons, adverse controls, native/WASM attribution,
Callgrind traces and the passing 39-command frozen combined validation. The
production candidate was removed under the literal latency gate. R05 and R11
admission claims are likewise superseded by the reconciliation in
[research-program.md](research-program.md#hypercurve-work-package-coverage-and-admission-reconciliation).
Their original archive records remain unchanged.

All eight commits only add their own records; historical paths and published
`main` are unchanged. They were unpublished at this checkpoint. Verify the
current expanded manifest using:

```sh
python3 scripts/evidence_archive.py verify ../csgrs-evidence.git \
  cd8bf060aa577231f84fffd7cf21e15127c028b0
```

Commit `5f5b31ac82d32f32ec004a901080de54b8693d9a` adds
`docs/evidence/2026-10-25-research-r14` (96 files, 5,893,560 bytes), with
SHA-256 `d2a04ded4c46f0eaea3959153e7d85dbceae00903ad3095951d2dbf56e2ba393`.
It retains both unadopted R14 preparation variants, independent rational oracles,
phase diagnostics and the unchanged full refilleting completions. The branch
was unpublished at this checkpoint.

Commit `c0fd193305812e52a76a076b8bfce89ceb208e62` adds
`docs/evidence/2026-10-25-research-r04-followup` (4,544 files, 23,148,395 bytes), with
SHA-256 `ef789e5a1b616bf86510bd17dda8f2c5fa522a44fe665a6015371550f074ee85`.
It retains the R04 sampling/predicate/triangulation repairs, clean source
counterfactuals, 38,400 exact triangulation results and adverse timings,
withdrawal of R05/R11/R12 production candidates, and the complete passing
39-command validation of the final retained stack. The branch was unpublished
at this checkpoint.

Commit `ec280dbccf7ecb7e77c2f021efa1ea16963e7c18` adds
`docs/evidence/2026-10-25-research-r23` (20,898 files, 73,997,727 bytes), with
SHA-256 `9e409ecec0e53d725d217d5f0f17c029a898a4dfcac16922d19e9ecc493209d9`.
It retains both R23 candidates, independent extrema/source-range oracles,
two 67,200-outcome cost comparisons with adverse controls, pinned Alumina
consumer evidence, full private owner/downstream checks and final primary
validation. The branch was unpublished at this checkpoint; earlier records
and published `main` are unchanged.

Commit `d65d6245382cde6ef23b695608c2df7c9f9fef7b` adds
`docs/evidence/2026-10-25-research-r15` (19,191 files, 60,600,432 bytes), with
SHA-256 `1384647865ae24a1ae53535fcca46ee43a23f0a3e30237db1055414fc7296600`.
It retains the R15 current-primary counterfactual, independent comparison and
cache/resource/receipt gates, 26,880 native and 4,480 allocation samples,
adverse controls, source/build attribution and all eight final combined-primary
checks. The equivalent-field replay is kept for scoped exact completion;
performance-only admission fails. Its measured cost base predates R23, as the
record states. The branch was unpublished at this checkpoint; historical paths
and published `main` are unchanged.
