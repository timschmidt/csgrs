# Sparse representation acceptance

The next failure in `qualification-v2` is the final assertion in Hypersolve's
`every_real_certificate_crosses_solver_analysis_and_proof_surfaces`. It requires
at least one of the 22 positive coefficient cases to remain undecided. All
original inputs and preceding assertions ran unchanged.

`representation-policy-v1` reproduces that assertion failure with both the
preserved coherent 045621eba graph and the current E12 graph. The diagnostic
copy adds a count print and an exact-certainty assertion; it retains the failing
assertion. Both report 22 sparse successes, 22 pattern-preserving successes,
zero unknowns, and exact certainty in each sparse solver's dense replay.
The observation wrapper requires the original test exit 101 and those counts;
its exit zero is successful diagnosis, **not a passing original test**.

The invariant is owned by scalar proof and strict solver replay, P2/P3/P8/P9/
P35/P39. The systems are `v*x=v` with certified positive nonzero `v` in each
representation, so the independently known solution is one. Preserve every
fixture and require all 22 cases to succeed on both paths, retaining the
accepted-report and exact-solution checks and adding exact receipt inspection.
Any future Unknown on these now-certified cases must fail the test. The existing
separate sparse terminal-zero regression continues to require typed
`UnknownResidual`; the dense terminal coverage additionally checks that
approximate reuse cannot create a strict certificate.

No solver production code, policy, representation generator, or tolerance is
changed. Rerun the full downstream matrix after the final test edit in a fresh
independently copied target, retaining both failed qualification trials.
