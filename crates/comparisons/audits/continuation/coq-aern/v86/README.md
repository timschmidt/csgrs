# Checkpoint 86

Read [findings](findings-v86.md) and [exact read records](read-records-v86.json).
The [protocol](protocol-v86.md) separates formal source, mathematical models and
native Hyper regression. Coq-aern coverage is now 30 complete files / 9,120 lines;
the broader audit and native donor qualification remain open.

`model-v86.mjs` checks precision/scaling/limit contracts with exact rationals.
`branches-v86.mjs` checks every admissible complex-root formula on a rational grid.
Neither is a Haskell implementation or a repair of admitted Coq proofs.

Exclusive command captures use `../../calcium/results/coq-aern-*-v86.*`.
`evidence-v86.mjs --record` creates an immutable manifest after qualifying current
sources, prior evidence and each classified gate. Without `--record` it verifies
the manifest; never overwrite a prior manifest or capture. Checkpoint 85's files
remain unchanged in the parent directory.
