This isolated candidate tests two guards around the original radical linear
loops: a stored zero opposite rational scale skips that loop. The rational
product, radical cross-products, both opaque loops, insertion order, budgets
and original cache fallback remain unchanged. It starts from the frozen
endpoint parent; rejected scaling variants and diagnostic counters are excluded.

The preceding [coverage diagnostic](../2026-10-13-coefficient-zero-contributions/decision-v1.json)
observed 9,488,190 such zero products. Rational multiplication already handles
zero before canonicalization and product-cache work, so coverage does not
predict a speedup. Two extra branches may regress nonzero and empty-vector cases.

Baseline and candidate receive the same seven regression functions and use
separate source, qualification and cost targets. Scalar qualification includes
the existing six cache-frontier regressions, full all-feature release library
tests and doctests, strict Clippy, formatting and warning-denied docs.

The fixed scalar cost protocol uses the three unchanged rational/quadratic/
integer probe suites, each in time and requested-allocation modes. Five blocks
run baseline/candidate, candidate/baseline, candidate/baseline,
baseline/candidate, baseline/candidate. All builds and preflight runs finish
before the 60 timed/allocation processes. No sample extension is allowed to
remove adverse rows. Unsupported scalar costs settle rejection; a supported
candidate still needs full native tests, fixed native controls, both original
limits, complete reports and binary-size evidence before retention.

[Scalar qualification](correctness-v1.json) passes: all seven regressions on
baseline and candidate, 870 candidate library results, 28 doctests, strict
scalar Clippy, formatting and warning-denied docs. Focused regressions overlap
the full library total.

[The fixed scalar costs](cost-assessment-v1.json) contain all 60 processes and
matching witnesses/inventories. Every allocation, retained and peak field is
identical in all five blocks. Eleven timing rows are faster in all blocks,
including quadratic multiradical-square warm/cloned ratios of 0.909/0.936.
`conjugate_then_affine_warm` is slower in all five blocks (1.022); five additional
rows are slower in four blocks, and nineteen have slower median paired ratios.
All ranges remain recorded. Four probe sizes are unchanged and two add 16 bytes.

[The scalar decision](scalar-decision-v1.json) advances this candidate to native
qualification with these tradeoffs open. The cost sample set is closed. No
statistical significance or application speedup is inferred.

Preparation and results are committed separately. No runtime change is admitted;
the endpoint parent's earlier tradeoffs and the full properties plan stay open.
