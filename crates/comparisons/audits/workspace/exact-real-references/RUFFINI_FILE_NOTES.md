# Ruffini source audit — OPEN

Source: [jonas-lj/Ruffini](https://github.com/jonas-lj/Ruffini), pinned at
82d552fee22d92e493936183fab8672517694e56. The public repository page was checked
against the supplied reference before acquisition. Original checkout clean;
all245tracked files inventoried separately from reading credit. No AGENTS.md.
The inventory checks exact Git blob bytes as well as SHA256 and counts physical
lines including unterminated final lines. Merely hashing a file gives no read
credit. Most repository source remains UNREAD.

Read in the opening slice:

- README.md: module architecture, generic algebra algorithms, constructive-real
  demo provenance, build/documentation commands and contributor/citation data.
- Parent pom.xml: ten modules, Java16source/target; inherited JUnit4.13.1,
  CommonsMath3.6.1 andGuava32.0.0-jre. GPG signing and Maven Central staging are
  configured; no publish/sign/deploy task is authorized by this audit. Native
  qualification should use local package/test phases and explicitly avoid
  external release operations.
- LICENSE: MIT, JonasLindstrøm2019. No donor code copied into Hyper.
- CITATION.cff and.gitignore: metadata and generated/tool-output exclusions.
- Both GitHub workflows: JDK17package/Javadoc and external publication actions;
  audit will not execute deployment or dependency-submission actions.
- reals/pom.xml: inherits parent and depends on common module.
- ConstructiveReal.java: all source read. Procedure IntFunction<BigInteger>,
  eager16bit initial cache, arithmetic estimator closures and eager expression
  strings. Cached coarsening uses arithmetic right shift without an added error
  budget. Multiplication magnitude estimation uses signed a.apply(0)+2 rather
  than absolute magnitude, with exception fallback3. Reciprocal searches only
  for estimates >=3 and is evaluated during construction, suggesting a negative
  input cannot finish. equals uses a17bit near-equality test while hashCode hashes
  estimator identity; these cannot be assumed to satisfy exact equality/hash
  contracts. of(double) uses the shortest decimal value via BigDecimal.valueOf,
  not exact binary import. estimate(m) truncates a decimal scaling factor before
  multiplication. Precision-counter overflow, signed/nearzero multiplication,
  reciprocal partiality, cache error and decimal-output behavior require native
  probes before failure claims. No native result or Hyper transfer yet.

Next read the other real-module files and the common/integer dependencies,
then establish a source-pinned Java build and independent rational/MPFR oracles.
All245files remain in the full audit scope, not only the scalar module.

## Scalar/shared continuation checkpoint — 2026-09-06

Current verified coverage: **76 of 245 files, 3,810 of 26,234 physical lines**.
The opening section above is historical; its native hypotheses are resolved or
explicitly left open below. All nine real-module files are read. The remaining
169 files are still in scope. Compiling a module gives no reading credit.

### Additional file-by-file notes

- `ConstructiveReals.java`: Field wrapper delegates all arithmetic and equality
  to ConstructiveReal; zero and identity also eagerly evaluate at 16 bits.
- `RealNumbers.java`: singleton Field<Double>, ordinary floating operations,
  Double.equals semantics. No exact-real domain/error certification.
- `ComplexNumber.java`: pair of public final doubles and eager formatted view;
  no object equality override.
- `ComplexNumbers.java`: bilinear multiply and unscaled x²+y² reciprocal;
  primitive component equality differs from Double.equals on NaN/signed zero.
  Overflow/underflow and dimension contracts are not yet natively qualified.
- `RealCoordinateSpace.java`: Euclidean norm from double inner product.
- `ComplexCoordinateSpace.java`: conjugates the second operand, then takes the
  square root of the real component of its self-inner-product.
- `RungeKutta.java`: mutable classical RK4 over double scalars, not certified
  integration. Stream.iterate ignores its passed prior state and mutates this
  integrator; interleaved stream behavior remains a source observation.
- `common/pom.xml`: dependency/build inheritance only.
- `AdditiveGroup.java`: generic signed scaling delegates to Multiply; exact
  isZero depends on the underlying Set equality contract.
- `CommutativeMonoid.java`: add/zero; balanced four-operand helper.
- `EuclideanDomain.java`: quotient/remainder and exact-divide contract, norm,
  ordering-dependent abs and divides; no guarantee added by wrappers.
- `Field.java`: division by embedded integer; inherits group/ring contracts.
- `Group.java`: inverse and multiply-by-inverse division.
- `InnerProductSpace.java`: scalar-valued inner product interface.
- `Module.java`: scalar multiplication and scalar-ring access.
- `Monoid.java`: identity test and integer powers through Power.
- `NormedVectorSpace.java`: norm returns double, not a certified real.
- `OrderedSet.java`: four comparisons around a Comparator.
- `Ring.java`: combines additive group and semiring.
- `SemiRing.java`: integer embedding and scalar-multiply helpers.
- `Semigroup.java`: multiply plus three/four-operand association helpers.
- `Set.java`: explicitly requires equals true iff equality in the set. This is
  the direct donor contract against which constructive equality was checked.
- `VectorSpace.java`: Module constrained to a Field.
- `Pair.java`: public mutable first/second, component equality/hash and function
  application. No thread-safety claim established for the constructive cache.
- `Multiply.java`: recursive doubling, but negative int scalars converge to
  zero and negative BigInteger scalars recur at -1. Both natively confirmed.
- `Power.java`: repeated squaring with group inverse for negative exponents;
  int MIN_VALUE recurs on itself. BigInteger MIN_VALUE-equivalent control works.
- `IntegerRingEmbedding.java`: binary positive embedding and negative recursion;
  int MIN_VALUE recurs because unary negation does not change it.
- `Sum.java`: sequential list/range/varargs folds; Iterable overload explicitly
  parallel. No certified balancing or precision scheduler.
- `Product.java`: sequential fold from identity.
- `DotProduct.java`: dimension checked only by assertion, zipped streams then
  parallel multiply/reduce. Truncation with disabled assertions and interaction
  with unsynchronized scalar caches remain qualification questions.
- `AbstractModule.java`: group delegation and retained scalar ring.
- `AbstractVectorSpace.java`: only specializes AbstractModule to Field.
- `VectorSpaceOverField.java`: map-based scaling, DotProduct delegation.
- `VectorGroup.java`: fixed-n coordinate operations; equality delegates to the
  vector's supplied predicate, without additional dimension checks.
- `BaseVector.java`: eager map, lazy view/pad; equality walks only this.size(),
  so unequal-length behavior merits qualification before importing this API.
- `ConcreteVector.java`: ArrayList storage; some constructors populate in
  parallel; ArrayList constructor retains caller storage, List copies it.
- `ConstructiveVector.java`: nonmemoizing callback view with assertion bounds;
  mapped views compose callbacks rather than materializing values.
- `Vector.java`: factories, borrowed array view, zip-based coordinate operations,
  parallel anyMatch. Representation-specific map strictness is significant.
- `SamePair.java`: Pair specialization plus two-element stream.
- `ConstructiveRealsDemo.java`: sqrt2, positive quadratic root and BBP pi demo;
  commented Gram-Schmidt experiment also read. Entire 95-line file rerendered
  after an earlier combined search was truncated; no credit from that search.
- `QuadraticEquation.java`: one quadratic-formula root, characteristic !=2
  required, catches square-root exceptions and discards their cause; neither
  degree-degenerate handling nor certified real-root isolation.
- `integers/pom.xml`: common/polynomials dependencies, duplicate polynomials
  declaration; not a new scalar implementation.
- `BigIntegers.java`: BigInteger operations, canonical nonnegative remainder
  even for negative divisors. Unlike the test helper, signed division is sound
  by inspection; a broader integer-native grid remains open.
- `BigIntegersModuloN.java`: Barrett reducer plus BigInteger.modPow override;
  reducer's noncanonical negative multiples break quotient equality natively.
- `BigRationals.java`: singleton specialization of FieldOfFractions<BigInteger>.
- `Integers.java`: unchecked Java int operations; division's a-residue can
  overflow, and abs(MIN_VALUE) gives a negative norm. Not yet natively probed.
- `IntegersModuloN.java`: int QuotientRing wrapper; inherits overflow limits.
- `Rationals.java`: FieldOfFractions<Integer>, inherits bounded integer limits.
- `BarrettReduction.java`: precomputes reciprocal at 2*bitLength, performs one
  correction. Positive x<2^(2k) controls pass; negative multiples return m and
  general oversized inputs can exceed the canonical range. See native grid.
- `BinaryGCD.java`: positive subtraction/valuation algorithm; (0,1) reaches a
  nonterminal fixed point, confirmed with bounded public-process probes.
- `BitLength.java`: halving recursion omits a maxBitLength<=1 base case;
  bitLength(1,64) natively overflows the Java stack.
- `EuclideanAlgorithm.java`: recursive scalar/list gcd; iterative extended gcd;
  list extended-gcd accumulates coefficients through recursive sublists. No
  half-GCD or bounded coefficient growth technique new to Hyper.
- `Fraction.java`: raw numerator/denominator record, permissive slash parser;
  representation equality is not field equality and denominator validity is
  not enforced by construction.
- `FieldOfFractions.java`: cross-multiply then reduce with extended gcd, even
  when Bezout coefficients are not needed. No cross-cancellation fast path.
  Inverting zero natively returns 1/0 instead of a domain error.
- `QuotientRing.java`: equality compares reduced representatives; therefore a
  noncanonical reduction map is insufficient even if it preserves congruence.
- `Calculator.java`: convenience folds, linear repeated addition for int scale,
  nullable ring/field depending constructor; cannot assume all methods valid
  on an additive-group-only instance. No precision-aware algorithm.
- `MultiOperator.java`: sequential nonempty fold, emptiness assertion only.
- `NullSafeRing.java`: null used as zero sentinel; multiply and negate may
  return null while add(null,null) materializes ring.zero(). This is not an
  exactness/unknown representation suitable for Hyper.
- `PerformanceLoggingRing.java`: synchronized increments but unsynchronized
  reads/reset; additionally performs null/zero/identity simplifications. Native
  constructive cases show this is not semantics-neutral instrumentation.
- `PerformanceLoggingField.java`: inverse skips isIdentity operands; inherits
  equality partiality and logging-ring behavior. No Hyper transfer retained.
- `ChineseRemainderTheorem.java`: pairwise Bezout merge then recursive vector
  rebuilding. Requires compatible coprimality assumptions not checked here;
  no empty-input handling. Current donor positive-prime tests pass.
- `Projection.java`: divides by u's self-inner-product; zero u delegated to
  scalar domain behavior. Operand orientation needs review for complex spaces.
- `TestUtils.java`: int and BigInteger test rings plus modular test field.
  BigInteger negative-remainder correction adds one to quotient where positive
  divisor requires subtracting one; native existing tests use nonnegative data.
  This helper must not serve as an independent signed oracle.
- `SamplingUtils.java`: rejection sampling, decreasing sequence and probabilistic
  factor sampler. Zero bound loops; sequence can end at zero, and double-based
  acceptance ratio can lose huge-integer information. Not source of certified
  scalar algorithms; these observations not yet natively qualified.
- `JacobiSymbol.java`: implementation interprets first argument as positive odd
  modulus, contrary to its test call's order; original test fails at i=2.
- `BigLegendreSymbol.java`: Euler criterion via modPow; prime modulus is an
  assumed precondition, not a primality certificate.
- `AlgorithmsTests.java`: all eleven tests read and run unchanged in both JVM
  modes. Positive-domain arithmetic fixtures omit signed/minimum boundaries;
  three sampling/counting tests are weak/no-oracle checks. Ten pass, one fails.

### Qualified conclusions and pending transfer work

Native and Hyper artifact details are in `ruffini-qualification/README.md`.
Confirmed requested-precision, equality/hash, signed-recursion, formatting and
shared-wrapper results must not be generalized into nonconvergence claims for
every real expression. of(double)'s decimal semantics differ deliberately from
Hyper's exact binary import; this alone is not classified as a defect.

The initial cache-coarsening suspicion is **rejected** for ordinary nonoverflowing
integer precision differences. Let K=2^delta and M=floor(A/K), with A integer
and |A-y|<=1 at the fine scale. KM<=A<=K(M+1)-1, hence
M-1/K<=y/K<=M+1, so |M-y/K|<=1. Signed arithmetic right shift therefore preserves
the requested bound. Both floor and ceiling estimator controls pass all 300
cases in each execution mode. This proof does not establish concurrency safety
or overflow safety of Java int precision arithmetic.

Hyper already has exact/partial certified comparison, sign-independent magnitude
planning, signed inverse division, exact float import, and a synchronized finest
cache. Its public matching 1,575-case corpus plus 584 additional checks pass in
debug/release. Do not transfer finite-bit equality, eager expression strings,
logging-driven simplification, or exception-based negative magnitude fallback.

One **unimplemented, unqualified** performance candidate remains: compare a
borrowed integer rescale of Hyper's cached value against its current clone then
rounded rescale, preserving the enclosure and concurrency contracts. The donor
floor-coarsening proof motivates testing, not automatic adoption. Existing cache
rounding behavior may be preferable; only isolated before/after tests, unbiased
benchmarks, allocation counts, and downstream validation can justify a change.
Remaining source, shared matrix/numerical contracts, precision-counter limits,
native demo and broader performance qualification remain OPEN.

## Matrix, transform, utility and remaining integer continuation — 2026-09-06

All 47 files below were read completely in numbered, untruncated chunks. With
the 25 parser/permutation/finite-field files in the next section, the explicit
coverage is now 148/245 files and 8,122/26,234 physical lines. The inventory TSV
records each file's full range and SHA256. A compiled file is not thereby read.
`ruffini-qualification/MATRIX_CHECKPOINT.md` separates executed results from
source observations; “native” below means both JIT and interpreter qualification.

### Common Fourier transforms and matrix algorithms

- `DiscreteFourierTransform.java`: radix-two split into even/odd transforms,
  powers precomputed at each recursive level; odd sizes fall back to an n²
  table populated with repeated generic Power calls. Input is padded/truncated
  to the configured n. There is no positive-n guard; n=0 recursively constructs
  size-zero transforms by inspection. All 69 valid finite-field transform and
  inverse-roundtrip probes pass natively. This is neither a new exact scalar
  representation nor a demonstrated alternative to Hyper's integer kernels.
- `InverseDiscreteFourierTransform.java`: reverse nonzero indices of the
  forward transform and scale by inverse(n). Assumes a full-length input and
  an invertible n in the field. The same 69 native valid-root probes pass;
  malformed length, characteristic and zero-size inputs remain unqualified.
- `Determinant.java`: recursive cofactor expansion, with no fraction-free or
  pivot policy. The lazy 0×0 case returns zero natively, not the empty product
  one. All 120 nonempty integer cases pass. Hyper's Bareiss implementation
  already handles empty determinant one, certified pivots and a pivot-free
  fallback; importing this factorial expansion would not improve it.
- `GaussianElimination.java`: parallel row updates and `field.isZero` pivot
  decisions. Once a pivot eliminates its column, exact zeros prevent subsequent
  rows becoming additional pivots in that column. ConstructiveReal's finite-bit
  equality makes that assumption unsafe as an exact rank certificate. This is
  a source-level dependency observation, not a newly executed rank corpus.
- `GramMatrix.java`: triangular dot products for square A*Aᵀ; rectangular
  block/pad/Strassen route. All 128 native integer cases pass. This Ring API is
  bilinear and does not promise complex Hermitian conjugation; lack of a
  conjugate is not itself reported as a violated contract.
- `GramSchmidt.java`: repeatedly projects and subtracts; dependent/zero input
  delegates division behavior to its field. The shared Projection orientation
  is incompatible with the complex inner-product convention: projecting i onto
  1 gives -i natively. No certified rank or orthogonality fallback to extract.
- `GramSchmidtOverRing.java`: clears projection denominators using products
  of self-inner-products, then gcd-normalizes each vector. Fraction-free work
  is relevant in principle but already present in Hypersolve. Empty/dependent
  vectors can reach empty accesses or zero divisions by inspection; no claim
  that the complete algorithm has been natively qualified.
- `KroneckerProduct.java`: constructs a block view and multiplies b_ij*a_ij.
  This reverses the conventional order for a noncommutative Ring; unqualified
  source observation pending a noncommutative oracle. No new layout benefit
  demonstrated against Hyper's fixed-size scalar/matrix representation.
- `MatrixAddition.java`: validates matching shapes, then materializes entries
  via the common Matrix factory. No sparsity-aware or symbolic combination.
- `MatrixInversion.java`: augmented elimination, then Java Object.equals
  rather than the supplied field equality to recognize the identity. A 1×1
  complex identity is rejected natively; a real-number identity control passes.
  This reinforces keeping exact scalar predicates separate from object identity.
- `MatrixMultiplication.java`: checks the inner dimension, materializes output
  using parallel entry construction and parallel DotProduct. All 1,296 native
  rectangular integer cases pass. Prepared donor-only timing includes exact
  result verification and is not a Hyper performance comparison.
- `QRDecomposition.java`: forms a square Q using a list of only the input's
  column count. A 3×2 tall input indexes a missing column vector natively.
  No general rank-revealing or exact certified QR algorithm is supplied.
- `StrassenMultiplication.java`: recursive copied quadrants and padded null
  cells; null-safe additions do not make the ordinary multiplication base case
  null-safe. Native grid: 186/264 pass, 78 padded cases throw; default cutoff1
  passes the tested sizes, while larger cutoffs expose null padding. Power-of-two
  timing is qualified separately. No backend transfer retained from this code.

### Common matrix representation and structures

- `BaseMatrix.java`: predicate equality loops rows up to width and columns up
  to height; all 24 nonsquare equality cases throw natively. Object.equals also
  restricts implementation class and compares entry references. `mutable()`
  dispatches through the Matrix-typed copy constructor; the public copy
  independence control passes, so an initial aliasing suspicion is not retained
  for that path.
- `ConcreteMatrix.java`: nested ArrayLists, first-row width lookup, eager map,
  transpose and submatrix. `view()` returns this rather than a lazy MatrixView,
  so callers cannot assume it removes those materializations. Row access wraps
  retained row storage; empty widths need care. No flat contiguous layout or
  structural-DAG sharing improvement over Hyper is demonstrated.
- `Matrix.java`: array-backed factories copy entries; ArrayList-row factory
  retains row lists. `fromBlocks` assumes uniform block shapes. `collapseColumns`
  uses height as its column iteration bound: of 32 cases, 8 square pass, 12 wide
  give wrong answers and 12 tall throw. Factory population is parallel by
  default; lazy-view semantics depend on the implementation as noted above.
- `MatrixView.java`: nonmemoized callback composition for transpose, submatrix,
  extension and map; no independent dimension/index validation. Useful
  separation of view and storage is already familiar, but repeated callback
  evaluation is not automatically better for expensive exact scalar nodes.
- `MutableMatrix.java`: public Matrix copy is independent natively. A separate
  package-private ConcreteMatrix overload aliases rows by inspection; that is
  not evidence that the tested public mutable-copy path aliases.
- `SparseMatrix.java`: HashMap storage keyed by a row/column value object,
  returning ring zero for absent entries. Builder.build retains the builder's
  mutable map: an already built matrix changes after another builder.add
  natively. Hyper immutable values must not inherit that snapshot behavior.
- `GeneralLinearGroup.java`: delegates group operations to generic matrix
  multiplication/inversion; no independent invertibility certificate.
- `MatrixRing.java`: fixed-dimension Ring wrapper; matrix-specific toString
  returns null. It adds no exact arithmetic representation or dispatch scheme.

### Common utilities, functional interfaces and exceptions

- `ArrayUtils.java`: array append/remove/union and recursive subset generation.
  The list ksubsets overload passes (size,k) to a routine expecting (k,size);
  native list failure and array control confirm the argument reversal.
- `MultiDimensionalArray.java`: recursively nested arrays with linear index
  descent; sparse variant stores shapes/keys without defensive immutability and
  enumerates the dense coordinate space for some operations. Width/populator
  construction records d=width instead of child dimension+1: native 3×2 shape
  reports dimension3, while the explicit-shape control passes. Integer size
  products have no overflow check. No exact tensor-storage improvement retained.
- `ArgMax.java`: nonempty validation, first maximum wins ties; ordinary fold.
- `DataConversionPrimitives.java`: names/comments for I2OSP/OS2IP are reversed.
  Signed BigInteger.toByteArray can add a leading sign byte, so positive128 in
  one unsigned byte can be rejected; negative inputs are not explicitly ruled
  out. Source-only observation, not a newly run serialization corpus.
- `EncodingUtils.java`: signed BigInteger byte export followed by unsigned
  import cannot preserve negative values without a sign convention. Domain
  requirements are not clearly documented here; no native failure claimed.
- `MathUtils.java`: bit-manipulation helpers assume positive nonoverflowing
  ints; zero, negatives and high shifts are unguarded. Arithmetic right shift
  makes negative binaryExpansion reach a fixed point by inspection. These are
  not a replacement for Hyper's checked precision/magnitude operations.
- `MatrixIndex.java`: value equality/hash on row and column; ordinary key type.
- `PermutationUtils.java`: direct list/array swaps, no distinct algorithm.
- `StreamUtils.java`: Iterable wrapper returns the same supplied Iterator,
  hence is one-shot rather than a repeatable sequence factory.
- `StringUtils.java`: repeated regex digit replacement; product formatting
  removes empty strings from its supplied list and accesses an empty result
  without a special case. Source observations only; no scalar exactness idea.
- `Triple.java`: immutable record-like accessors, no numerical machinery.
- `InvalidParametersException.java`: parameter-error wrapper only.
- `NotASquareException.java`: failure marker and message constructors only.
- `NotInvertibleException.java`: inversion-error constructors only; no partial
  comparison or explicit unknown-result protocol.
- `IntBinaryFunction.java`: primitive-index two-argument callback interface.
- `TernaryOperator.java`: homogeneous three-argument callback interface.
- `TriFunction.java`: heterogeneous three-argument callback interface.

### Remaining integer files

- `IntegerPolynomial.java`: regex-based signed term splitting, builder.set
  per exponent, implicit coefficient parsing. Repeated powers appear to
  overwrite rather than accumulate, and implicit -x handling needs a native
  probe after reading Polynomial.Builder. Not yet compiled in the matrix slice.
- `CongruenceSolver.java`: extended-gcd input is [m,p0,p1,...] but q_i uses
  coefficient i rather than i+1. Native one-variable exhaustive small-prime
  grid: 36 zero-RHS controls pass, all 408 nonzero RHS cases fail the equation.
- `EulersTotientFunction.java`: factors first, then applies product(p-1)/p;
  no improved factor-free algorithm or independently certified factor result.
- `ModularSquareRoot.java`: implements p mod8=3,5,7 only; mod8=1 is accepted
  by construction then reports not implemented. Native square-input grid:
  58 supported cases pass; all 17 cases for p=17 throw. Nonresidue and negative
  representative handling remain separately unqualified.
- `factorize/Factorize.java`: probable-prime certainty1000 and recursive
  splitting. factor(1) can repeatedly split unchanged1 by inspection. Failed
  factor selection propagates rather than yielding a certified partial answer.
- `factorize/PollardRho.java`: bounded restart count, fixed x²+1 iteration and
  sqrt(m).longValue iteration limit, which can truncate for large m. Random
  representative selection is modulo-biased. These are source observations,
  not timing or failure-rate measurements; no Hyper algorithm retained.
- `structures/limbs/BigElement.java`: retained mutable list of limbs without
  normalization/copy, no compact limb representation to extract.
- `structures/limbs/BigElements.java`: final carry condition tests ring.zero
  against zero and therefore never appends a carry; multiply returns null.
  Both failures are native. Negation is per-limb modulus-minus-limb without
  full carry propagation; source observation. Avoid this incomplete backend.
- `TestIntegers.java`: two positive-domain tests, both pass unchanged in both
  modes. Supplemental native probes also confirm the previously noted
  Integers abs(MIN_VALUE) negative norm and overflowing division intermediate;
  the two original tests do not cover these boundaries.

## Parser, permutation and finite-field reads — 2026-09-06

All 25 files were fully read, not merely searched or compiled. The observations
below are **not yet natively qualified**. Dependencies in polynomials remain
unread, so apparent extension-field failures must be tested before promotion to
executed findings. These modules stay within the full-file audit even where
their direct scalar payoff is low.

### Parser (all eight files)

- `pom.xml`: common dependency and finite-field test dependency; no new runtime
  precision or exactness policy.
- `EvaluationException.java`: checked evaluation-error constructors.
- `Evaluator.java`: RPN stack evaluator with arity/identifier validation,
  separate ring/field operator factories and supplied function/variable maps.
  The default convenience evaluation catches parse/evaluation errors, prints
  them and returns null; other runtime failures need not follow that path.
- `MultiOperator.java`: configurable arity, guarded by assertions; not a
  statically validated expression type.
- `NumberParser.java`: functional string-to-value parser; exactness belongs to
  the caller's implementation, not the parser itself.
- `Parser.java`: shunting-yard variant; ordered operators [+,-,*,/,^] give
  separate precedences, exponent processing is left-associative, and unary
  minus is rewritten to 0-minus only in selected positions. Function tokens
  are not popped immediately at a closing parenthesis, suggesting f(2)+3 can
  evaluate f(2+3); malformed delimiters can reach null stack accesses. Textual
  spacification also requires identifier-prefix/scientific-notation tests.
  No production parsing change follows from these unqualified observations.
- `Token.java`: token text/type carrier; no typed exact expression DAG.
- `TestParser.java`: one composite real/finite-field expression fixture; cannot
  establish precedence, associativity, unary or malformed-input completeness.

### Permutations (all five files)

- `pom.xml`: common module dependency, no scalar arithmetic implementation.
- `RandomDerangement.java`: randomized construction with recursively computed
  double derangement counts and no memoization; n=1 can return an identity
  even though no derangement exists. Sampling correctness, recursive resource
  growth and endpoint behavior require native checks before quantified claims.
- `Permutation.java`: varargs constructor retains the caller's array; cycle
  and vector operations assume valid indices. Claimed uniform random sampling
  uses nextInt(n-i) without shifting by i, suggesting biased Fisher-Yates
  selection. Exact finite draw enumeration would be a better test than timing.
- `SymmetricGroup.java`: composition a(b(i)), inverse fills the reverse map;
  input bijectivity is assumed rather than verified. Matrix orientation should
  be compared using this documented composition convention, not guessed.
- `PermutationTests.java`: a fixed-seed size13 derangement example; one such
  case does not test distribution or invalid sizes.

### Finite fields (all twelve files)

- `pom.xml`: common, integer and polynomial dependencies plus integer test
  dependency; unchanged module lifecycle not yet qualified.
- `AlgebraicFieldExtension.java`: polynomial quotient over a supplied minimal
  polynomial, irreducibility assumed. Inversion obtains Bezout coefficient
  then divides by the constant gcd, with a degree assertion. That normalization
  is the right general shape but not a new certified algebraic-number layer.
- `BigFiniteField.java`: extension quotient and polynomial extended gcd;
  returns a Bezout coefficient without the explicit constant-gcd normalization
  seen above. Whether PolynomialRing already normalizes must be read/tested.
- `BigPrimeField.java`: Barrett-backed quotient and modular inverse, checks
  integer gcd==1 but not primality. Inherited reducer rejects very small moduli
  and has the already qualified signed/oversized representative limitations.
- `FiniteField.java`: bounded-int field order, hardcoded irreducibles for
  selected small characteristics/degrees, otherwise null. Same apparent
  inverse-normalization question as BigFiniteField; dependent source unread.
- `GaussianRationals.java`: constructs a quadratic polynomial with its linear
  coefficient represented as Fraction(0,0), not 0/1. The representation is
  visibly invalid for a rational; downstream consequences still need a native
  extension-field test rather than assuming how polynomial trimming treats it.
- `PrimeField.java`: bounded int modular arithmetic with unchecked products;
  extended-gcd inverse and a degree-one polynomial quotient conversion. Not
  arbitrary-size certified modular arithmetic.
- `QuadraticField.java`: same Fraction(0,0) linear coefficient; supplied
  quadratic parameter does not itself certify irreducibility or real embedding.
- `algorithms/BerlekampRabinAlgorithm.java`: shifts f to f_k but raises a
  polynomial modulo a quotient constructed from the original f, suggesting a
  mismatch. Recursive factor/root calls restart their local iteration budget;
  no global resource bound is inferred. Pending polynomial/source/native work.
- `algorithms/BigTonelliShanks.java`: rejects zero using its nonresidue path,
  and builds powers via int shifts even with BigInteger primes. Large two-adic
  exponents can therefore wrap; characteristic/order preconditions and native
  tests must distinguish supported nonzero cases from missing boundary cases.
- `algorithms/TonelliShanks.java`: order computed in int; characteristic-two
  branch uses 1<<(q/2) rather than q/2 as the exponent, which needs a finite
  exhaustive oracle. Random zero uses Object.equals; candidate nonresidue and
  retained bPower state raise additional source questions. Do not equate these
  untested hypotheses with a proven general failure of Tonelli-Shanks itself.
- `AlgorithmTests.java`: one small prime-field square-root example, not an
  exhaustive field/radical/zero/large-two-adicity suite. Native execution pending.

### Transfer disposition

The useful matrix ideas are fraction-free updates, explicit views, sparse
storage, and avoiding unnecessary scalar work. Hyper already has fraction-free
solves, fixed-size exact matrix kernels, shared scalar DAGs and explicit
certified/unknown decisions. Ruffini supplies no qualified exactness or
completeness improvement over those paths here. Null-as-zero padding, mutable
builder snapshots and Object.equals pivots are rejected. Benchmark evidence is
scoped to this donor implementation and these small prepared integer workloads;
it cannot rule out Strassen for other sizes, scalar costs or storage designs.
No new production changes, memory/binary-size wins or code-size wins are claimed.

## Polynomial module complete read — 2026-09-06

All 29 files/2,585 physical lines were read completely, including commented
code and tests. A truncated combined rendering was discarded and all affected
files rerendered before read credit. Current coverage:177/245 files,
10,707/26,234 lines. `polynomial-analysis.json` qualifies the native findings
below in both JIT and interpreter modes; source-only observations are labeled.

### Representation and ring interfaces

- `polynomials/pom.xml` (22 lines): depends only on common; no independent
  numeric backend or exactness policy. Direct compilation is not Maven lifecycle
  qualification.
- `elements/Polynomial.java` (268): sorted sparse integer-power map; absent
  coefficients are null. Builder removes zeros and keeps one constant zero,
  but varargs construction retains explicit leading zeros and differentiation
  of a constant creates an empty map. Native zero/degree predicates then throw.
  Builder.build wraps its mutable SortedMap without copying; both built values
  and the copy constructor change with the builder natively. Evaluation walks
  ascending powers and reuses successive exponent gaps, rather than Horner;
  all420 exact integer evaluation checks pass. Map/scale/reverse do not enforce
  canonical zero degree. Reverse uses actual stored degree, not a caller-supplied
  reversal width, which matters to FastDivision. Integer exponent overflow and
  thread safety are not newly qualified.
- `structures/PolynomialRingOverRing.java` (163): sequential pair-of-terms
  product, synchronized builder accumulation and parallel addition. All640
  product cases pass. Equality reverses the absent-coefficient zero condition:
  x differs from explicit0+x but equals1+x natively. Explicit leading-zero
  degree also breaks equality. Monic division is correct on225 constructed
  divisibility controls but stalls when a stored leading coefficient is zero.
  A1025-observation prefix verifies its unchanged1+0x state; public calls hit
  a2second process cap. No null-as-zero or builder-view transfer retained.
- `structures/PolynomialRing.java` (43): field division inverts the leading
  coefficient, then delegates to the ring division. It does not monic-normalize
  the gcd returned by generic EuclideanAlgorithm; this resolves the previously
  pending finite-field inverse question. Ordinary nonmonic division control
  passes. Its degree norm depends on valid canonical polynomial representation.
- `structures/PolynomialRingKaratsuba.java` (34): routes products above two
  coefficients through Karatsuba; smaller cases use ordinary multiplication.
  Its divide override calls FastDivision without adapting the field-leading
  coefficient to that algorithm's constant-one inverse precondition. A nonmonic
  field division fails natively while the ordinary PolynomialRing control passes.
- `structures/PolynomialRingFFT.java` (134): fixed-n Fourier representation,
  pointwise products and conversions. This has cyclic-product behavior when
  degree reaches n, not unrestricted K[x] closure as the class description
  suggests. Conversion of oversized input truncates rather than reducing it
  consistently modulo x^n-1. Both contract-boundary probes fail; ordinary small
  products in the donor tests pass. Transforming zero back produces an empty
  polynomial whose isZero predicate throws. Size/root validity, operand-owner
  mismatch and n=1 monomial vector size remain source-only questions. No claim
  that cyclic convolution itself is mathematically wrong.
- `elements/Monomial.java` (159): exponent-array key, componentwise product,
  division and lcm, total-degree sum and repeated-power evaluation. The public
  array and varargs constructor retain mutable exponents; native mutation
  changes a monomial. This can also invalidate sorted/hash key assumptions by
  inspection. Negative/overflowing exponents are not generally validated.
- `elements/MultivariatePolynomial.java` (268): sorted exponent-vector map,
  padding to a larger variable count, sparse arithmetic and differentiation.
  leadingMonomial uses lastKey but parameterless leadingCoefficient uses
  firstKey; native2+3x returns the wrong leading coefficient. Builder retains
  its mutable map, confirmed natively. DEFAULT_ORDERING is globally mutable.
  Custom-order arithmetic and constant±1 formatting have further source-only
  limitations; no certified monomial-order abstraction is supplied.
- `structures/MultivariatePolynomialRingOverRing.java` (99): delegates sparse
  arithmetic; equality's second pass fetches ai from b rather than a, then can
  call scalar equality with a missing coefficient. Source-only observation;
  the native same-support equality control passes and does not qualify different
  support cases. No new exactness or allocation technique is demonstrated.
- `structures/MultivariatePolynomialRing.java` (124): repeats the equality
  issue, stores a monomial ordering for division and uses total degree as an
  asserted Euclidean norm. For multiple variables this is not an ordinary
  Euclidean-domain division contract: x divided by y may leave x, whose total
  degree is not below y's. This is a source/mathematical contract observation,
  not a newly run x/y test. The x/x native progress failure is separate.

### Univariate algorithms

- `algorithms/BatchPolynomialEvaluation.java` (30): reusable subproduct-tree
  wrapper, a relevant general multipoint idea but not a new exact-real scalar
  representation. Native sparse x² evaluation throws at1,2,4,8 points, while
  constant-zero controls pass. Three-point input is explicitly rejected by the
  underlying power-of-two tree; the wrapper does not advertise that restriction.
- `algorithms/BinaryTree.java` (110): recursively stores product labels; downward
  remainders and upward weighted combinations use the same tree. Each descent
  builds and concatenates result lists. Only power-of-two leaf counts are
  accepted. No proof of benefit for Hyper's common few-point/root-isolation
  workloads follows from the asymptotic comment; persistent tree storage and
  coefficient growth would need matched measurement.
- `algorithms/Remainder.java` (48): monic dense division working from nullable
  getCoefficient results, without filling sparse holes. Its monic check uses
  Object.equals rather than field equality. The null handling explains native
  sparse evaluation/interpolation exceptions. Returning an empty-map zero can
  propagate inconsistent polynomial degree. No remainder-tree implementation
  from this code is retained.
- `algorithms/PolynomialInterpolation.java` (45): reusable subproduct tree,
  constructs the derivative-like weight polynomial by combining all-one leaves,
  evaluates weights, divides values by those weights, then combines upward.
  Native square-value examples pass at1,2 points but throw at4,8. Retains the
  caller's x list while tree labels are precomputed, a source-only mutation
  consistency issue. Repeated nodes/zero weights require field domain handling.
- `algorithms/LagrangePolynomial.java` (52): direct product-of-linear-factors
  interpolation, no precomputed barycentric weights, input-size or repeated-node
  validation. All four native control sizes1,2,4,8 pass. No performance transfer
  inferred merely because it is simpler than the failing tree path.
- `algorithms/Inversion.java` (59): Newton recurrence2g-fg² with power-of-two
  truncation and triangular symmetric product reuse. It assumes f(0)=1;
  all480 valid-precondition truncated inverse checks pass. It returns enough
  terms for the enclosing power-of-two target, which still satisfies the stated
  modulo-x^l contract. Generic Ring admits noncommutativity despite the symmetric
  product expansion; large int shift/exponent endpoints remain source-only.
- `algorithms/FastDivision.java` (44): reverses divisor, computes a truncated
  reciprocal, convolves a reversed dividend and reverses the quotient. All225
  constructed controls pass, but those quotient constants are nonzero. The
  dedicated x²/x case returns the wrong quotient because actual-degree reverse
  loses the required width. The identity a=bq+r alone cannot validate this:
  the implementation defines r by subtraction even when its degree is too high.
  Both quotient and remainder are checked against independent expected values.
- `algorithms/KaratsubaAlgorithm.java` (82): splits at half the maximum size,
  three recursive products and add/subtract recombination; loops over degree
  ranges and allocates fresh builder maps at each level. All640 matched dense/
  sparse small product cases pass. A separately qualified larger dense/sparse
  donor benchmark is in progress; no Hyper backend transfer is yet justified.
- `algorithms/Modulus.java` (56): entirely commented-out prototype for retained
  transformed divisor/inverses, including incompatible duplicate apply stubs.
  Fully read but not an executable feature. Hyper already has a retained
  CertifiedPolynomialDivisor with exact leading inverse for repeated reductions;
  no unfinished commented code is imported.
- `algorithms/CharacteristicPolynomial.java` (34): builds xI-A as polynomial
  entries then invokes the recursive determinant. Shares factorial cofactor
  cost and empty-matrix boundary issues, rather than giving a fraction-free or
  division-free characteristic-polynomial improvement. Source only in this
  slice; prior native determinant controls remain separately recorded.

### Multivariate algorithms and ordering

- `algorithms/MultivariatePolynomialDivision.java` (87): finds a divisible
  leading monomial and computes a positive quotient coefficient, but adds the
  multiple instead of subtracting it. Native x/x overF5 reaches the repeated
  leading-coefficient cycle1,2,4,3;1025 callback observations verify that prefix
  and the uninstrumented call hits the process cap. It also uses parameterless
  divisor leadingCoefficient instead of the selected-order coefficient. No
  correct multivariate reduction algorithm is extracted from this path.
- `algorithms/GröbnerBasis.java` (84): textbook pair/S-polynomial loop with
  recursive restart on each new basis member, rescanning old pairs and printing
  progress. S-polynomial leading terms use the requested ordering, but its
  division helper is created with the default ordering. Inherits the qualified
  reduction problem; the Gröbner algorithm itself has not been run here. No
  pair criteria, F4/F5 reduction, modular lifting or replay certificate to extract.
- `ordering/MonomialOrdering.java` (9): marker Comparator interface only.
- `ordering/LexicographicalOrdering.java` (21): componentwise lexicographic
  comparison, equal variable counts asserted. No partial/unknown scalar order.
- `ordering/GradedLexicographicalOrdering.java` (34): total int degree then lex;
  degree-sum overflow can compromise ordering by inspection. No wide checked
  exponent representation or packed-monomial optimization.

### Recursive representation and tests

- `elements/recursive/Polynomial.java` (170): coefficients recursively contain
  lower-dimensional polynomials. Creation is mutable, some arithmetic results
  use immutable sorted maps, and untouched child coefficients are shared.
  Native adding a mutable polynomial to zero yields a result later changed by
  mutating the original. set on arithmetic-produced maps can also be unsupported
  by inspection. Recursive division ignores the coefficient-division remainder
  when eliminating the leading term, so multivariable completeness/progress is
  not established. No safe persistent-tree benefit demonstrated.
- `elements/recursive/Constant.java` (102): zero-variable scalar leaf initialized
  to null until explicitly set. Constant division accepts a Field or unit
  divisor in a Ring; direct scalar errors propagate. Null-as-uninitialized is
  not an exact zero or explicit unknown protocol.
- `elements/recursive/Monomial.java` (38): term record and formatting over a
  retained exponent list; StringUtils handles products. No arithmetic kernel.
- `src/test/java/PolynomialTests.java` (166): seven original tests pass unchanged
  in both JVM modes. Inversion, FastDivision and Fourier-division tests print
  values without asserting their core algebraic guarantees; FastDivision is
  compared with the same override. The batch test uses TestField(51), a composite
  modulus, with the logging wrapper masking some null/zero cases. Passing this
  suite is not a certificate of general field or polynomial correctness.

### Previously read dependent modules: native follow-up

Both modes now qualify the pending parser and finite-field observations:
f(2)+3 evaluates25 and f(2)*3 evaluates36 for f(t)=t²; 1*-2 is rejected,
while parenthesized negation works. The default parser evaluates2^3^2 as64,
which is a left-associativity/conventional-notation mismatch, not an explicitly
documented right-associativity promise. Unmatched close/comma raise NPE, unmatched
open is caught as an evaluation error. IntegerPolynomial.parse(x+x) overwrites
one term; parse(-x) throws. Original parser test still passes.

Exhaustive scripted uniform draws for sizes2..7 give2,6,24,120,720,5040 paths,
but only2,4,11,40,162,788 distinct permutations; size7 output multiplicities
range1..34. These are exact distribution counts, not a noisy Monte Carlo claim.
The constructor array-alias probe also fails. Original permutation test passes;
RandomDerangement edge cases/counting cost remain unqualified.

Degree-one finite extensions provide valid small test fields: constant inverses
pass only for1 (6/68 int and5/66 BigInteger cases). Base-prime inverse controls
pass68/68, and the normalized AlgebraicFieldExtension path passes66/66. Thus
the missing constant-gcd normalization is not hidden in PolynomialRing. Squaring
the Gaussian/quadratic generator throws ArithmeticException from the invalid0/0
linear coefficient; no claim about every extension element is made. The original
finite-field square-root test passes. Nonconstant extension inverses and the
pending Tonelli-Shanks/Berlekamp-Rabin boundaries remain open.

Hyper controls independently pass1,520 checks in debug and release, with a
stable201-file source snapshot: exact evaluation, monic/nonmonic/sparse division,
gcd normalization, bivariate division, zero/domain cases and radical identities.
Hyper's dense vectors and strict trimming make missing/trailing zeros explicit;
retained divisors avoid repeated certification, and balanced eval_poly prevents
long nonrational Horner chains. These are inspected existing capabilities, not
new transfers. No characteristic-p finite-field, parser or random-permutation
equivalence is claimed for APIs that Hyper does not expose here.

## Class-group module complete read — 2026-09-06

All four files/423 lines were fully read while the isolated polynomial benchmark
ran. Coverage now181/245 files,11,130/26,234 lines;64 files remain in scope.
These are source findings pending native qualification, not executed results.

- `class-group/pom.xml` (29): common and integers compile dependencies, no custom
  test-source directory. Tests live outside Maven's ordinary src/test/java tree.
- `quadraticform/ClassGroup.java` (112): negative discriminant group wrapper,
  coefficient equality, inverse by negating b, and composition delegation.
  Constructor checks negativity but not discriminant congruence0/1 mod4 before
  integer division in the principal form; invalid inputs can silently produce
  a different discriminant. Sampling narrows to prime |D| with D=1 mod4, searches
  primes a=3 mod4, then uses modular roots and reduction. No finite sampling
  budget; small bounds can reach the previously noted zero-bound sampler.
  Probable primality is not an exact certificate or a scalar relation oracle.
- `quadraticform/QuadraticForm.java` (180): exact quadratic evaluation, cached
  discriminant, normalization/reduction and Bezout-based composition. Input
  positivity/primitivity is not generally enforced. Object.equals includes
  the mutable optional discriminant cache and ordering object, yet no matching
  hashCode override; warming one cache can affect equality by inspection. The
  Group equality wrapper is a separate coefficient comparison and must not be
  conflated with that object contract. No NUCOMP-like bounded intermediate
  scheme or new exact-real separation certificate is apparent here.
- `test/java/QuadraticFormTests.java` (102): five tests cover reduction,
  composition, two small class groups and sampling. Imports reference the
  obsolete finitefields.quadraticform package rather than the current
  dk.jonaslindstrom.ruffini.quadraticform package. Source and location issues
  require a direct build diagnostic; do not claim these tests ran unchanged.
  Composition/sampling examples largely print rather than assert algebraic
  invariants, and Object.equals/cache behavior may affect other assertions.

Class-group native follow-up now confirms the source questions:146probes per
mode,80pass/66wrong, identical outputs. All64valid discriminant controls pass;
all64invalid-congruence negative inputs are accepted. Object equality changes
after warming just one discriminant cache and recovers after warming both;
equal objects have different identity hashes in these runs. Separate Group
coefficient equality, original reduction values and order3/order7 controls pass.
Unchanged original test compilation fails on the obsolete imports; no original
JUnit run is claimed. See POLYNOMIAL_CHECKPOINT.md and classgroup-analysis.json.

Polynomial timing follow-up now completes all12families and27,753,408verified
measured products. Polynomial-level Karatsuba wins only with costly dense
coefficients in this screen: CPU ratios0.596and0.447at length64/128 with the
2048bit coefficient parameter. Every sparse family regresses, sometimes severely;
all small-coefficient families regress too. The initial calibration-cap failure
is preserved separately. A density-and-cost-aware **Hyper experiment remains
unimplemented/unqualified**, now a concrete next action ahead of any production
transfer. Native donor timing is not a Hyper speed claim; correctness, symbolic
behavior, allocation and code-size gates still apply.

## Elliptic module complete source read — 2026-09-06

All17files/1,424physical lines read completely, including constants and tests.
Coverage198/245files,12,554/26,234lines;47files remain UNREAD. Findings here are
source analysis, not native execution or cryptographic qualification.

- `elliptic/pom.xml` (34): common/finite-fields dependencies and Commons Codec
  1.15 for the hex-decoding test. No new native test execution this checkpoint.
- `elements/AffinePoint.java` (36): immutable record, null/null infinity,
  conversion preserves infinity; generic apply does not bypass null coordinates.
- `elements/EdwardsPoint.java` (14): immutable pair with coordinate rendering;
  identity convention belongs to the owning group, not an extra point tag.
- `elements/JacobianPoint.java` (28): infinity by Z=0; affine conversion divides
  both X and Y by Z². Conventional Jacobian Y requires Z³; coordinate-scaling
  invariance needs qualification before borrowing any formula.
- `elements/ProjectivePoint.java` (27): homogeneous X/Z,Y/Z conversion and Z=0
  infinity predicate. Explicit representation validity is not enforced.
- `structures/ShortWeierstrassCurveAffine.java` (104): discriminant assertion,
  coordinate field equality, addition identity/inverse cases and affine slopes.
  Negation lacks the addition routine's explicit infinity branch. Exact-real
  division elision is attractive, but domain proof must remain explicit.
- `structures/ShortWeierstrassCurveProjective.java` (180): division-free
  Bernstein–Lange-style addition/doubling and cross-product equality. Addition
  has no identity/exceptional-point dispatch; invalid all-zero triples can
  satisfy cross-product equality vacuously. No assumption of complete formulas
  or transferable certified geometry predicates is warranted without testing.
- `structures/EdwardsCurve.java` (88): rational addition formulas, explicit
  (0,1) identity, x-negation, birational Montgomery map. Its v-coordinate is
  twice the conventional ratio and B is correspondingly scaled; do not flag
  that consistent convention as a factor-of-four defect. Denominator-zero
  exceptional maps and completeness conditions are not enforced.
- `structures/MontgomeryCurve.java` (123): affine formulas and birational
  Weierstrass conversion. Distinct-x branch uses Object.equals instead of field
  equality; jInvariant reuses the short-Weierstrass parameter expression even
  though A,B have Montgomery meaning. Infinity is not special-cased in negate
  or conversion. Assertion-only nonsingularity is not an always-on contract.
- `structures/Curve25519.java` (85): BigInteger field parameters, scalar clamp,
  coordinate decoding and compressed/uncompressed point reconstruction.
  Encoders/decoders have different size conventions, compression-sign handling
  needs review, and public parameters are mutable. This audit records numerical
  representation concerns only, not a production cryptographic assessment.
- `test/java/TestCurve25519.java` (27): one test with two fixed x-coordinate
  scalar-multiplication comparisons; it does not assert full signed-point,
  infinity, conversion or general group-law coverage. Not run this checkpoint.
- `algorithms/MillersAlgorithm.java` (108): bit-scheduled repeated squaring,
  affine tangent/chord rational functions, sparse polynomial evaluation. The
  tangent numerator code has the correct +lambda*px-py constant despite the
  adjacent misleading comment. Identity/vertical-tangent domain cases and
  m preconditions are not generally checked. No new exact-real scalar method.
- `algorithms/OptimalAtePairing.java` (102): signed-digit loop, twists, generic
  target-field powering. Tangent formula specializes to curve a=0 despite the
  generic curve parameter. g1embedding is stored but unused. Inputs, digit
  conventions and final-exponent divisibility are assumed, not certified.
- `algorithms/WeilPairing.java` (42): four Miller evaluations plus quotient;
  auxiliary-point exclusions are documented but unchecked. Retain explicit
  partial-domain thinking, not a claim that arbitrary exact equality is decidable.
- `structures/bls12381/BLS12381.java` (124): all parameters/generators read,
  quadratic/cubic/quadratic extension tower, twists and signed-digit pairing
  wrapper. Deep generic polynomial nesting can multiply allocation overhead;
  no verified replacement for Hyper's retained algebraic evidence appears here.
- `structures/bn254/BN254.java` (132): analogous extension tower and constants,
  different twist and curve parameter. Shared labels do not establish standard
  parameter interoperability; no external standards conformance claimed.
- `structures/bls12381/Serialization.java` (170): mask/coordinate extraction,
  compressed flags, G1/G2 conversion. Loop offset combines an increment of48
  with multiplication by48; compressed state, infinity lists, coefficient
  presence and sign conventions require validation. Do not transfer unchecked
  representation boundaries. No exploit or third-party probing performed.

Hyper comparison: existing homogeneous rational curve representations and
strict reciprocal/predicate policies already separate retained coordinates from
division-domain certification. The elliptic source provides useful warnings
about representation invariants, but no qualified production transfer yet.

## Remaining executable demo source complete read — 2026-09-06

All14remaining non-data demo files/1,030lines read completely. Overall coverage
212/245files,13,584/26,234lines. The32constant/matrix data files and root SVG
remain UNREAD: generated data is not excluded from the user's requested scope.
These are source findings, not claims of native execution.

- `demos/pom.xml` (41): finite-fields/reals/elliptic/common dependencies, no
  dedicated benchmark framework or correctness test configuration.
- `demo/PolynomialMultiplication.java` (73): seeded size18 finite-field DFT
  multiplication, fixed roots-of-unity table, operation logger and printed
  equality. Input lengths9and10 fit the transform's18coefficient output without
  cyclic wrap. No rigorous elapsed-time benchmark or asserted oracle here.
- `demo/StrassenDemo.java` (44): seeded8×8small-integer comparison and operation
  counts; printed equality. Prior isolated timing, not multiplication count
  alone, showed donor Strassen's overhead. No new Hyper transfer.
- `demo/AKS.java` (85): polynomial quotient witness construction and parallel
  searches. Perfect-power preprocessing is absent, order search/int conversion
  and witness bound use bounded/native approximations, and n<=r returns false.
  Trial division can include n itself. Do not treat this demo as a qualified
  exact primality certificate or move approximate proof bounds into Hyper.
- `demo/BellPolynomials.java` (56): exact BigInteger binomial recurrence and
  sparse multivariate construction; computes B0throughB15 with15printed steps.
  Division in the binomial recurrence is exact by its combinatorial invariant.
  Reusable values, not recomputed expression trees, are the useful existing idea.
- `demo/DeterminantFormula.java` (34):6×6symbolic matrix over36variables, full
  determinant expansion and printing. No normalization/performance improvement
  over Hyper's retained Bareiss/interpolation machinery established.
- `demo/MultivariatePolynomials.java` (25): recursive polynomial mutation and
  rendering only; no arithmetic equality or independent expected result.
- `demo/CauchyMatrix.java` (76): fixed-node Cauchy matrix-vector action expressed
  via interpolation and multipoint evaluation. Precomputes f,f', reciprocal
  factors and trees before counting only the repeated action. Potentially useful
  for repeated structured solves, but node-distinctness/denominator domains and
  the previously exposed interpolation boundaries must be certified. No current
  Hyper Cauchy-workload evidence; candidate remains unqualified, not retained.
- `demo/ECMDemo.java` (71): inversion failure intended as factor evidence.
  The k recurrence starts at1 and multiplies by gcd(i,k), leaving k=1 throughout;
  it does not construct the intended smooth scalar. Returned exception element
  is not independently validated as a nontrivial divisor. No execution/attack
  against any target, and no cryptographic qualification claimed.
- `demo/BLS12381.java` (20): three bilinear expressions printed without an
  equality assertion; shares the previously read extension/pairing stack.
- `demo/WeilPairingDemo.java` (57): fixed small field/points, printed bilinearity
  comparisons. Exclusions for auxiliary points are assumed, not a general test.
- `demo/HadamardMatrix.java` (234): Williamson search, exact integer/quotient-
  polynomial identities and final H H^T assertion. Nested parallel streams and
  simultaneous mutable polynomial-builder accumulation require determinism
  scrutiny; builder issues were already exposed in the polynomial audit.
  FormalRing is explicitly modulo x^n-1, not unrestricted polynomial arithmetic.
  No parallel scheduling transfer justified by source alone.
- `demo/poseidon/Constants.java` (100): complete path maps for16constant and
  16matrix files, fixed round table, decimal parsing and row-major dimensions.
  Readers are not closed; paths depend on working directory and resources are
  reloaded for each construction. No source read credit granted to the referenced
  data files merely because their paths and sizes have been inventoried.
- `demo/poseidon/Poseidon.java` (114): all8fixed examples read, generic field
  fifth powers and matrix rounds. Internal mutable state makes instance lifetime
  significant; every round prints. Loader/round state and I/O dominate what
  would otherwise be a scalar benchmark. Not run or cryptographically certified.

## First four matrix resources — complete read, 2026-09-06

All entries of matrix00(4lines,2×2), matrix01(9lines,3×3), matrix02(16lines,4×4),
and matrix03(25lines,5×5) were individually read with numbered file boundaries.
An initial combined rendering joined EOF-without-newline boundaries; each file
was rerendered separately before credit. All54entries are plain nonnegative
decimal integers for the loader's row-major convention. All lie below the
advertised modulus. Bareiss and independent Laplace expansion agree on every
one of344nonempty square minors, and every determinant is a unit modulo that
modulus. Another120small integer controls cross-check the determinant routines.
No primality or cryptographic qualification is inferred, and these tables offer
no new scalar algorithm. Other numeric resources remain UNREAD.
Coverage216/245files,13,638/26,234physical lines;29files remain in scope.

Elliptic numerical follow-up:1,672probes each in normal/interpreted JVM modes,
byte-identical;1,434PASS,232WRONG,6NullPointerExceptions. Independent small-field
affine/Montgomery addition, homogeneous conversion and projective doubling
controls pass. Conventional Jacobian scaling exposes152wrong conversions;
projective addition has74wrong results, the all-zero triple compares equal to
valid points in3probes, and3Montgomery invariant checks disagree with the
independently derived value. Infinity negation throws in both affine models.
Only7unchanged donor coordinate/curve files compiled for this bounded probe;
existing dependency/source/class hashes revalidated. No original Curve25519
JUnit or whole elliptic/cryptographic qualification is claimed. Hyper already
retains explicit homogeneous/domain boundaries; no production transfer kept.

## Remaining matrix resources — complete read, 2026-09-06

All twelve files were rendered individually with numbered boundaries, with no
truncation. Each line is one decimal integer in the loader's row-major table;
these data files contain no additional scalar algorithm or evaluation policy.
Per-file read coverage: matrix04(36lines,6×6), matrix05(49,7×7),
matrix06(64,8×8), matrix07(81,9×9), matrix08(100,10×10),
matrix09(121,11×11), matrix10(144,12×12), matrix11(169,13×13),
matrix12(196,14×14), matrix13(225,15×15), matrix14(256,16×16),
matrix15(289,17×17). All1,730lines read, not inferred from parsing or hashes.
The prior exhaustive344minor check covers only matrix00–03; it is not extended
to these larger matrices by source reading. No cryptographic claim or Hyper
production transfer follows from opaque constants.
Coverage228/245files,15,368/26,234physical lines;17files remain UNREAD:
the16round-constant resources and root SVG. Full ecosystem goal remains open.

Separate post-read structural checks on matrix04–15 pass:1,730canonical decimal
entries of the expected dimensions, all within the advertised modulus and units
modulo it;70,438two-by-two minors and12full determinants are units too. Integer
Bareiss and modular elimination agree on the full determinants. Both algorithms
also agree with independent Laplace expansion on120small integer controls.
Every modular pivot is explicitly proved invertible by extended GCD; no modulus
primality assumption is hidden in the checker. These checks do not establish
all larger minors or cryptographic properties. Evidence is separately versioned
in `ruffini-qualification/remaining-matrices-analysis.json`; the original
four-matrix all-minor evidence is unchanged.

## First two round-constant resources — complete read, 2026-09-06

- constants00: all128physical lines read individually; one nonnegative decimal
  constant per line, matching2coordinates×(8full+56partial)rounds.
- constants01: all195physical lines read individually; same representation,
  matching3coordinates×(8full+57partial)rounds.

No truncation or inferred read credit. These opaque constants contain no new
exact-real representation, normalization or scheduling algorithm; no performance
or cryptographic conclusion follows from reading them. The round table and
loader were reread to establish dimensions, not to credit the other data files.
Coverage230/245files,15,691/26,234physical lines;15UNREAD remain in scope.

The separately scoped round-table checker passes all323entries for canonical
decimal syntax, advertised-modulus range and expected round dimensions. This
does not validate the generating procedure or any cryptographic property.

## Next two round-constant resources — complete read, 2026-09-06

- constants02: all256physical lines rendered with line numbers, no truncation;
  decimal data matching4coordinates×(8full+56partial)rounds.
- constants03: all340physical lines rendered with line numbers, no truncation;
  decimal data matching5coordinates×(8full+60partial)rounds.

These opaque numeric tables add no scalar representation or performance idea.
Read coverage is now232/245files,16,287/26,234physical lines;13UNREAD remain:
constants04–15 and root SVG. No generating-procedure or cryptographic claim.

## Fifth round-constant resource — complete read, 2026-09-06

- constants04: all408physical lines read in numbered, untruncated ranges
  1–140,141–280,281–408. Canonical decimal data, no final newline, matching
  6coordinates×(8full+60partial)rounds from the previously read loader.

No scalar algorithm or performance/completeness transfer in this numeric table.
Coverage233/245files,16,695/26,234physical lines;12UNREAD remain in scope:
constants05–15 and root SVG. Range/dimension checks are separate from reading;
no generating-procedure or cryptographic qualification is claimed.

## Sixth round-constant resource — complete read, 2026-09-06

- constants05: all497physical lines read in numbered, untruncated ranges
  1–170,171–340,341–497. Decimal data with no final newline; the loader expects
  7coordinates×(8full+63partial)rounds. No new scalar algorithm is present.

Coverage234/245files,17,192/26,234physical lines;11UNREAD remain in scope:
constants06–15 and root SVG. The separately scoped constants02–04 checker
passes all1,004entries and15format/range predicate controls; it does not qualify
constants05, generating procedures, cryptographic properties or unread files.

## Finite-field native boundaries — 2026-09-06

34bounded JVM processes complete with3,206probes per mode, byte-identical default
JIT/interpreter outputs. New Java harnesses use the unchanged, fingerprinted donor
classes and dependency jars. Independent Node checks use carry-free packed BigInt
multiplication, polynomial reduction and exact unit/square replay. The four large
odd moduli are proved prime by exhaustive integer trial division, not a probable
prime test. No timeout or256draw harness budget was reached.

- `FiniteField`/`BigFiniteField`: of682canonical nonconstant inverse probes,
  546return a wrong field value (276int/270BigInteger);136pass. Missing constant-gcd
  normalization remains visible with monic and nonmonic irreducible moduli.
  `AlgebraicFieldExtension`'s normalization passes all332corresponding canonical
  BigInteger probes. Separately, all164nonzero inputs stored with trailing zero
  coefficients raise NotInvertibleException; do not conflate this representation
  boundary with the canonical nonconstant inverse defect. All34zero inputs are
  rejected. Fields of orders4,8,9,25,49; all nonzero elements are independently
  certified units, and every nonzero scaling of each modulus is included.
- `TonelliShanks`: exhaustive GF(2^n),n=1..8 covers510elements; only the16zero/one
  cases pass, all494others return a wrong square root. The characteristic-two
  branch uses `1 << (q/2)` where q=2^n, instead of the Frobenius inverse exponent
  2^(n-1); machine shift wraparound compounds that wrong exponent. Separate
  seeded odd-prime tests pass all549cases over3,5,7,13,17,41,97, including
  nonresidue rejection. The private Random is seeded by reflection without
  changing arithmetic; this is not exhaustive random-stream or extension-field
  qualification of the odd-characteristic algorithm.
- `BigTonelliShanks`:660supported small-prime representative cases give636passes
  and24rejections of exact zero (including ±modulus). Six known squares at each
  of four exact primes withv2(p−1)=30,31,32,35 give15passes/9StackOverflowErrors.
  At s=32,9mod184683593729 is one rejected-by-overflow square; explicit root3
  independently certifies it. The `1 << m` int exponent reaches MIN_VALUE and
  the existing `Power.apply(Integer)` negative-exponent recursion cannot negate
  that value. No claim about every large prime or square is made.
- `BerlekampRabinAlgorithm`: all monic quadratics over3,5,7 at three seeds,
  eight iterations per recursive call, give133verified roots,110unresolved
  searches and6NullPointerExceptions.102unresolved inputs truly have no roots;
  eight have roots but bounded randomized exhaustion is not itself an incorrect
  answer. All six exceptions have a root. No wrong returned root was observed,
  and the source's shifted-modulus concern is not claimed as a reproduced wrong
  answer. A256draw harness cap and external process caps are explicit safeguards.

The first harness incorrectly admitted BigPrimeField(2/3), violating its explicit
Barrett constructor restriction; its failed run is retained and not qualified.
V2 separates two unsupported-constructor checks, restricts BigInteger arithmetic
to supported moduli, and adds the order49extension. A sandbox javac EPERM was
retried with authorization; host output classes match the earlier unqualified
bytes. Original source/classes/logs remain preserved. No donor or Hyper edit.

Current Hyper comparison is source inspection, not a new finite-field equivalence
test: Real::sqrt explicitly handles exact zero and negative inputs; rational
powers use unsigned_abs/BigInt magnitude and bounded eager expansion with exact
fallback; Hypersolve trims only certified zero coefficients and retains explicit
unknown/nonzero checks before inversion. Hyper is characteristic zero here, so
the donor's Frobenius/finite-field routines are not direct replacement candidates.
Existing strict domain/storage boundaries should be retained. No production
transfer or new benchmark claim follows from this correctness-only slice.

## Seventh round-constant resource — complete read, 2026-09-06

- constants06: all576physical lines read in numbered, untruncated ranges
  1–144,145–288,289–432,433–576. One decimal value per line; no final newline.
  Expected shape8coordinates×(8full+64partial)rounds. No scalar algorithm.

Coverage235/245files,17,768/26,234physical lines;10UNREAD remain in scope:
constants07–15 and root SVG. Reading is not inferred from parser/hash checks.

The separate constants05–06 checker passes all1,073entries for dimensions,
canonical decimal syntax and advertised-modulus range. No generating-procedure
or cryptographic qualification. Exception-diagnostic follow-up replays all249
Berlekamp cases per JVM mode: all110unresolved cases have the actual iteration-
limit message; all six null errors originate in Integers.negate. A separate
high-square replay confirms StackOverflowError in each mode. JIT top frames
are Power.apply, interpreter top frames are Integer boxing; an initial validator
assumption that these frames must match was corrected after inspecting both.
Numerical outputs agree; these diagnostic stack traces do not byte-match.

## Root architecture SVG — decoded-source and visual asset review, 2026-09-06

`abstractions.svg` is a generated 75,401-byte/12-physical-line asset, SHA256
770dec9140a009a8436024ed8181d911c5023cbf9f8042f45a6b9751aa2bbe2c.
It wraps a base64 SVG and a deflated, URI-encoded diagrams.net model containing
another copy of that SVG. The image payloads are byte-identical. Full outer,
mxfile and model envelopes were read; all decoded SVG source was read in bounded
presentation ranges1–80,81–160,161–240,241–327 (103decoded physical lines),
including its entire PlantUML comment. The 554×926 PNG was rendered and viewed.

The versioned decoder and verify-abstractions.mjs preserve and validate exact
source/payload/artifact hashes, complete chunk reconstruction, all14interface
labels,15method labels and16inheritance edges. Model UML and SVG-comment UML
agree after CRLF and XML-comment-safe arrow normalization. The current Java
interfaces retain those16edges and add Set<-OrderedSet. The diagram is stale:
old math.algebra package, getZero/getIdentity rather than zero/identity,
divisionWithRemainder rather than divide, Integer rather than BigInteger norm,
and no OrderedSet/default helper surface. Inkscape completed with style-property
warnings; visual inspection agrees with the checked labels/arrows. Rendering
does not establish implementation correctness.

Disposition: no scalar algorithm or new transfer. Runtime algebra dictionaries
and a total-looking equality interface are already compared in the earlier
abstractions audit. The figure adds no exactness, approximation, scheduling,
cache, memory, or performance evidence. Hyperlimit's current module contract
still distinguishes certified results and explicit uncertainty. No production
change or numerical benchmark is justified by this diagram.

Per the root asset convention, this is REVIEWED_ASSET, not fabricated manual
base64/deflate source-read credit. Source coverage stays235files/17,768lines;
one additional asset reviewed, nine UNREAD resources remain (constants07–15).
The original donor is untouched. Initial decoder metadata output truncation
received no credit; the corrected bounded decoded-source reads are complete.

## Eighth and ninth round-constant resources — complete reads, 2026-09-06

- constants07: all639physical lines read in untruncated numbered ranges
  1–160,161–320,321–480,481–639. Shape9coordinates×(8full+63partial)rounds.
- constants08: all680physical lines read in untruncated numbered ranges
  1–170,171–340,341–510,511–680. Shape10coordinates×(8full+60partial)rounds.

Both contain canonical decimal data with no final newline. Their complete bytes,
line counts and SHA256s match the pinned inventory. The new scoped checker reads
the fingerprinted loader's round schedule and validates all1,319entries using
independent BigInt and lexical range predicates. All18positive/negative predicate
controls pass. This is shape/format/range validation only, not a claim about the
generating procedure, cryptographic properties or unread resources. No scalar
algorithm or additional performance/completeness candidate occurs in these data.

Current coverage237source/data files,19,087physical lines; one generated asset
reviewed separately. Seven UNREAD resources remain: constants09–15 (7,135lines).
No prior frozen coverage validator was altered to accept the newer count.

## Tenth and eleventh round-constant resources — complete reads, 2026-09-07

- constants09:814physical lines, read in untruncated numbered ranges1–165,
  166–330,331–495,496–660,661–814. Loader shape11coordinates×(8full+66partial).
- constants10:816physical lines, read in untruncated numbered ranges1–204,
  205–408,409–612,613–816. Loader shape12coordinates×(8full+60partial).

These are decimal data tables, not arithmetic implementations; no new scalar
algorithm or transfer candidate. Source coverage now239files/20,717physical
lines, plus one separately reviewed generated asset. Five tables remain UNREAD:
constants11–15, totaling5,505physical lines. Format/range qualification is
separate and does not establish generation or cryptographic properties.

## Twelfth round-constant resource — complete read, 2026-09-07

constants11: all949physical lines read in untruncated numbered ranges1–240,
241–480,481–720,721–949. Loader shape13coordinates×(8full+65partial)rounds.
No algorithm in this decimal data table. Coverage240source/data files,
21,666physical lines, plus one separately reviewed generated asset. Four tables
remain UNREAD: constants12–15, totaling4,556physical lines.

The scoped constants09–10 validation passes all1,630entries and18predicate
controls; both files lack a final newline. No generation/cryptographic claim.

The combined constants09–11 check also passes all2,579entries and the same18
predicate controls. constants11 has no final newline. The240-file resource
checkpoint passes inventory/range fingerprints and the397-file Hyper snapshot
with zero drift; it is not a new native arithmetic/performance run.

## Thirteenth round-constant resource — complete read, 2026-09-07

constants12: all1,092physical lines read in untruncated numbered ranges1–273,
274–546,547–819,820–1092. Loader shape14coordinates×(8full+70partial)rounds.
This is decimal data without a scalar algorithm or new transfer candidate.
Coverage241source/data files/22,758physical lines, plus one separately reviewed
generated asset. constants13–15 remain UNREAD, totaling3,464physical lines.

constants12 shape/format/range checks pass all 1,092 entries and 18 predicate
controls. No final newline; no generating-procedure or cryptographic claim.

## Fourteenth round-constant resource — complete read, 2026-09-07

constants13: all 1,020 physical lines read in untruncated numbered ranges
1–255, 256–510, 511–765, 766–1020. Loader shape: 15 coordinates × (8 full +
60 partial) rounds. Decimal data, no scalar algorithm or new transfer candidate.
Coverage: 242 source/data files, 23,778 lines, one separately reviewed generated
asset. constants14–15 remain UNREAD (2,444 lines). Production code unchanged.

The combined constants12–13 check passes all 2,112 entries and the same 18
predicate controls; both files lack a final newline.

## Final two round-constant resources — complete reads, 2026-09-07

- constants14: all 1,152 physical lines read in untruncated numbered ranges
  1–288, 289–576, 577–864, 865–1152. Shape: 16 coordinates × (8 full + 64 partial).
- constants15: all 1,292 physical lines read in untruncated numbered ranges
  1–323, 324–646, 647–969, 970–1292. Shape: 17 coordinates × (8 full + 68 partial).

Both are decimal data tables with no scalar algorithm or new transfer candidate.
Ruffini read coverage is complete: 244 source/data files, 26,222 physical lines,
plus the separately reviewed generated SVG (12 outer physical lines, decoded
source and visual review documented above). No UNREAD files. This does not turn
bounded numerical probes into universal correctness or cryptographic claims.
The broader ecosystem audit remains open; no production change from these data.

Final resource qualification passes: all 4,556 entries in constants12–15 and
all 10,854 entries across constants00–15 satisfy the pinned loader dimensions,
canonical-decimal syntax and advertised modulus range. Independent BigInt and
lexical predicates agree; all 18 positive/negative controls pass. Every table
lacks a final newline. No claim about generation or cryptographic suitability.
Evidence is preserved in read-round-constants-12-13-14-15-analysis.json and
read-round-constants-00-01-02-03-04-05-06-07-08-09-10-11-12-13-14-15-analysis.json;
the root ledger records their hashes. resource-checkpoint-244-files.json passes
all coverage/inventory fingerprints and the 397-file Hyper snapshot without
drift. Historical native numerical and performance findings retain their
original scope; this checkpoint does not rerun or broaden them.
