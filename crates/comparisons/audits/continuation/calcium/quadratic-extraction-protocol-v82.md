# Quadratic extraction — checkpoint 82 protocol

Read both pinned get_quadratic implementations and tests completely (four files,
644 lines). Exactness, normalization and selected embedding precede potential
cost improvements. This pass does not change any donor, Hyper or isolated candidate.

The native corpus has 775 inputs, each queried in modes 0/1/2 and twice on the same
value. Mode copies start from the same constructed enclosure. The second query
follows an explicit 1536-bit cache request and a 2048-bit output request from the
first query; this is not a precisely controlled cold/warm benchmark. Inputs are
(a+b*sqrt(d*s^2))/q, with known squarefree d, signed b and positive q. A Cartesian
grid covers real/complex/rational values and square factors through 65537; focused
cases cover +/-2^256, denominator 2^128 and close conjugates at denominator 2^1024.
Fifteen zero-radical-coefficient cases exercise rational promotion.

Independent BigInt checks prove reduced coefficients, positive denominator,
radicand sign, exact rational part, radical magnitude and selected sign. Both
original/reconstructed primitive polynomials and four component enclosures are
checked independently, with exact quadratic signs, ordered endpoints and width
at most 2^-96. Full factoring must yield the known squarefree d; Q(i) must use -1
in every mode. No minimality promise is imposed on heuristic smooth factoring.
Rational output must have b=c=0. No invalid-degree or undocumented output alias
calls are made. Mutation controls challenge values, embedding, normalization,
minimality, precision, ordering, identifiers and record counts.

Reuse the pinned current FLINT/GMP/MPFR libraries. Native collection is bounded
by 120 wall seconds, 90 CPU seconds and 1 GiB address space. Memcheck gets 180 wall
seconds without the address-space cap. A byte-identical output and no lost blocks
or errors are required. Whole-harness allocations are not a per-operation or peak
memory benchmark. Archive and upstream tests are read, not newly executed.

Hyper's bounded square extraction and 256-node surd parser are compared by source.
A separate public capability probe may be added; it must bind its own source,
metadata and debug/release results. No exact-real total equality claim follows
from a degree-2 algebraic extraction contract. No production transfer is retained
without a plausible benefit, differential correctness and matched cost evidence.
