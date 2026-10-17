# Direct small-cache-hit evidence

This proposal is **rejected** by the preserved cold-batch cost gate. Read the
[decision](decision-v1.json), [evaluation](progress-v1.md) and
[contract](contract-v1.md). The reviewable patches are under `proposal/`.
No production cache change is applied.

The [lossless archive](raw-evidence-v1.tar.xz) retains helpers and preparation
under `work/`, raw executable checks and measurements under `run/`, and the
separate metadata checks under `metadata/`. The
[member index](archive-members-v1.json.gz) records each original path, byte
length and SHA-256. Every archived byte stream was verified independently.
List entries with `tar -tJf raw-evidence-v1.tar.xz`.

Prepared helper files do not establish execution; the decision identifies
unexecuted checks and the refused support preflight. Original snapshots,
binaries and raw artifacts remain physically retained. The
[checkpoint](checkpoint-v1.json) discloses the inherited missing paths.
