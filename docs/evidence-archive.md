# Evidence archive

Generated evidence does not live in this repository. That covers raw
command logs, benchmark samples, source snapshots, audit corpora, replay
bundles and checkpoint archives. It is kept in a separate evidence repository.
This repository keeps a digest for each archived record in
[`evidence-archive.tsv`](evidence-archive.tsv).

| | |
| --- | --- |
| Evidence repository | <https://github.com/timschmidt/csgrs-evidence> |
| Archived commit | `b20000e377626a41b6ff8e6f87aee2b20ebd7865` |
| Extracted from | CSGRS `7e3c36c1248823a8bb36b2b161068660cea2ebf3` (2026-10-20, before the archive split) |
| Records | 309 records, 552,370 files, 13.96 GB |

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

## Verifying a record

`evidence_archive.py` computes one SHA-256 per record from git objects, so
no working-tree checkout is needed. A record digest covers the content digest,
file type and relative path of every file in the record:

```sh
git clone --bare https://github.com/timschmidt/csgrs-evidence.git
python3 scripts/evidence_archive.py verify csgrs-evidence.git \
  b20000e377626a41b6ff8e6f87aee2b20ebd7865
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
