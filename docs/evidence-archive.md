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
