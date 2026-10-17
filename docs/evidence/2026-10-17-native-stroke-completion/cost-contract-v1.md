# Stroke completion and cost comparison

Compare the b1373812 implementation and the final qualified candidate using
their completed HyperDRC build libraries, with matching feature/dependency
selection. Do not reopen either Cargo target. Retain compiler, hardware,
source, dependency, probe and executable identities and every raw sample.

Six exact source cases cover an axis, a 3–4–5 direction, an irrational
diagonal, symbolic translation, symbolic radius, and a rational small radius.
Exercise both butt and round caps. The end-to-end measured operation creates
the exact source/path and stroke, measures area and bounds, materializes its
native boundary, rebuilds the boundary as a region, and checks both regions
against an independently evaluated integer point-to-segment oracle on a
half-unit grid. Pure integer squared distances determine every expected point
classification, including boundary cases. Analytical area and extent formulas
provide separate scalar checks. A finite grid is not a universal set proof.

Count construction/query failures and undecided comparisons separately from
contradictory definite results. No contradiction is acceptable. A sample is a
certified completion only when all required results and comparisons are
certified. Do not report a baseline undecided case as a faster success.

Time source construction through complete diagnostic output materialization;
exclude process startup, JSON encoding/file IO and final model destruction.
Retain allocator call/request counts, retained requested bytes and peak extra
live requested bytes. These are not expression-depth or heap-bound proofs.
Each process performs a fresh load and a repeat after dropping model owners;
global scalar state can remain warm. Run five alternating baseline/candidate
pairs per input/cap. Never overlap measured pairs or original timing gates with
another validation process launched by this workstream.

Retain complete exact-scalar/native-segment snapshots, reconstruction results,
all point classifications and every artifact difference. Boundary subdivision,
ordering and equivalent scalar forms may change through the declared native
construction migration. Semantic checks remain separate. Costs are comparable
as completed contracts only where both variants complete every requirement.
Keep adverse latency, allocation, retention and binary-size observations.
Preflight runs validate the harness and are not benchmark samples.
