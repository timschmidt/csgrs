# Native symmetric elliptic integrals

Baseline: `891e35453`. Owner: Hyperreal, P1–P7 and P17–P20.
Hyperspatial E2 needs elliptic integrals for exact ellipsoidal rectification;
the baseline has no native elliptic integral API. The missing acceptance test
is `tests/elliptic_integrals.rs`, initially expected to fail compilation.

Contract: `Real::elliptic_rf(x,y,z)` represents Carlson's integral RF for real
nonnegative arguments with at most one zero. `Real::elliptic_rd(x,y,z)`
represents RD for nonnegative x,y with at most one zero, and strictly positive z.
Negative arguments and divergent endpoints return `Problem::NotANumber`;
unresolved domain proofs return `Problem::Exhausted`. Neither method evaluates
through machine floating point. Their outputs remain refinable native values,
usable by arithmetic, predicates and serialization.

The defining integrals are in [DLMF 19.16](https://dlmf.nist.gov/19.16).
With lambda = sqrt(xy)+sqrt(yz)+sqrt(zx) and each new argument (v+lambda)/4,
RF is unchanged; RD equals 3/(sqrt(z)*(z+lambda)) plus one quarter of the new RD.
See [DLMF 19.26](https://dlmf.nist.gov/19.26#iii).

## Representation and enclosure proof

The optional zero is permuted to the first argument (RD only exchanges its
first two arguments). A node stores `[sqrt(x), ln(y), ln(z)]` and evaluates
`[square(root_x), exp(log_y), exp(log_z)]`. Every finite payload therefore has
the required domain even after serialization. RF and RD have separate appended
enum variants, preserving earlier binary enum indices; their boxed payloads
do not enlarge the common expression carrier. All-equal arguments collapse to
`1/sqrt(x)` or `1/(x*sqrt(x))` after domain validation. The integral's strict
positivity is retained as a structural fact. An undecided zero is never admitted
as a zero endpoint.

At an even, negative grid precision q, integer endpoint pairs enclose each
argument in units of `2^q`. Integer square roots bound each product's square
root; lower endpoints round down and upper endpoints round up. Addition and
division by four preserve this inclusion at each duplication. RD's positive
summands are separately enclosed on the same grid, including their successive
`4^-j` factors. No unrounded rational-denominator chain or nested duplication
expression is retained.

Let m be the smallest lower argument endpoint and M the largest upper endpoint
in physical units. When m > 0, monotonicity of the defining integrands gives
`1/sqrt(M) <= RF <= 1/sqrt(m)` and
`1/M^(3/2) <= RD <= 1/m^(3/2)`. For RD, add the retained sum and multiply the
remaining-integral enclosure by `4^-j`. Directed integer quotients convert
these bounds to units of `2^(p-3)` for requested output precision p. Return only
when the integer bounds differ by at most two. Rounding their midpoint to units
of `2^p` introduces at most another half unit, keeping the total error strictly
below one output unit. These bounds cover the entire remaining integral;
there is no unaccounted Taylor remainder.

The initial grid uses twice the requested fractional accuracy plus guards,
because the integer square-root endpoint bounds halve grid accuracy. A restart
doubles grid accuracy if denominators remain unresolved or the scheduling
iteration limit is reached. Those scheduling choices cannot authorize a return;
only the enclosure-width check does. For fixed admitted finite arguments the
exact duplication makes the arguments converge, while arbitrarily fine grids
make enclosure rounding arbitrarily small. This is a mathematical convergence
argument, not a bound on native execution time or heap usage.

Each iteration and restart checks the inherited abort signal. Public enclosure
queries preserve the existing unavailable/cancelled outcome; an aborted zero
approximation is not a represented integral value. No new retained cache is
introduced: native immutable DAG/cache and cold/warm serialization contracts
apply. Precision integers, allocation limits and operand-refinement work remain
the existing scalar limits. Arbitrarily huge mathematical domains do not imply
that every input finishes within a fixed resource budget.

## Acceptance and consumers

`tests/elliptic_integrals.rs` checks diagonal identities, circular limits,
negative/divergent/undecided domains, permutation symmetry, homogeneity, the
independent cyclic RD identity, refinement order, cancellation recovery,
concurrent readers, JSON/CBOR replay and uniform scales `1e-320` and `1e320`.
An independent MPFR integrand expansion checks 512-bit agreement for all 22
certificate recipes. Its transformed inputs have series ratios at most 3/4;
2048 terms bound the analytical tail below `2^-840`, with 1024-bit MPFR arithmetic
and a wider acceptance margin. MPFR AGM separately checks complete RF with
complements down to `1e-160`. These are development oracles, never runtime
dependencies or proofs of the native implementation.

Hyperspatial's shared `dev/elliptic.rs` covers four ordinary/endpoint/scale
regimes, all 484 ordered scalar-family pairs, cold and display-warmed inputs,
and construction of validated ellipsoids and ECEF outputs from the new scalars.
The same workload feeds independent-bucket lexicase selection and ASan mutation.
The coverage script measures the three dedicated production implementation
files separately and gates their line coverage at 95%; shared dispatch and
structural matching remain outside that reported subset. The 100% and connected
formal-verification targets remain open.

`benches/elliptic_integrals.rs` records cold construction/enclosure, warm
enclosure, and independent MPFR AGM point timings with per-trial completion and
96-bit acceptance. `benches/gmp_api.rs` additionally exercises evaluated RF/RD
circular limits against independent MPFR circular identities. The GMP API audit
remains unchanged. These different output contracts do not support an
exact-kernel versus point-arithmetic speed ratio.

Final commands, results and source hashes are recorded in the
[verification bundle](../../hyperspatial/verification/2026-10-19-e2-elliptic/README.md).
This prerequisite does not claim a completed ellipsoidal Transverse Mercator
projection, Jacobi functions, complex continuation, new bindings or a published
release. Existing earlier capabilities and the complete development plan remain
in scope.
