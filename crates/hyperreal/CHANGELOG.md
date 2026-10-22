# Changelog

## [0.14.0] - Unreleased

### Breaking

- With the `serde` feature, `Real::from_json`, `Real::from_bytes`, conversion
  from a CBOR `Value` and the `Deserialize` impl for `Real` now validate the
  decoded value instead of trusting it. Invalid named certificates, an opaque
  value without its computable payload, a payload that contradicts its class,
  and known domain violations return `Problem::ParseError`; a certificate or
  payload relationship that cannot be proved returns the new
  `Problem::UnknownCertificate`. Every partial operation in a computable
  payload must have its operand domain proved, so an undecided domain also
  returns `Problem::UnknownCertificate`. The wire layout is unchanged.
- Removed `Computable::sign`, which reported both zero and an unresolved sign
  as `Sign::NoSign`. Use `Computable::sign_until`, which returns `None` when
  bounded refinement cannot decide the sign.
- `RealOrderingCertificate` has a new `StructuralFacts` variant, so exhaustive
  matches must handle it. The implicit discriminants of
  `DifferenceStructuralFacts` and `BoundedRefinement` increased by one.
- `Rational::powi` returns `Problem::Exhausted` when the exponent's magnitude
  exceeds 255 and the result would exceed about 2^16 bits of numerator plus
  denominator; 0.13.0 rejected only exponents of 1000 or more bits.
  `Real::powi` keeps such powers exact as lazy values rather than eager
  rationals. Zero raised to a negative power now returns
  `Problem::DivideByZero` from `Rational::powi` instead of zero.
- With the `dispatch-trace` feature, `dispatch_trace::RationalTraceStats` has a
  new public field, `gcd_operand_widths`, so struct literals must set it.

### Added

- Elementary and numerically stable `Real` functions: `cbrt`, `root_n`,
  `powi_i64`, `pow_rational`, `exp2`, `exp10`, `expm1`, `ln_1p`/`log1p`,
  `ln_1m`/`log1m`, `softplus`, `sigmoid`, `logit`, `logaddexp`, `logsubexp`,
  `cot`, `sin_pi`, `cos_pi`, `tan_pi`, `cot_pi`, `sinc`, `sinc_pi`, `cosc`,
  `try_atan2`, `sqrt1pm1`, `sqrt1m1`, `hypot_minus`, `hypot2`, `hypot3`,
  `mul_add`, `sum_products`, `diff_of_products`, `eval_poly` and
  `eval_rational_poly`.
- Probability and special functions on `Real`: the error-function family
  (`erf`, `erfc`, `erfcx`, `erfinv`, `erfcinv`), standard and parameterized
  normal densities, distribution functions, quantiles, log forms, hazards,
  Mills ratios and moments (`dnorm`, `pnorm`, `qnorm`, `normal_cdf`,
  `normal_quantile` and related methods), `gamma`, `lgamma`, `beta`,
  `ln_beta`/`lbeta`, the regularized incomplete beta and gamma functions, and
  `chi_square_cdf`/`chi_square_sf`. Their domains and implemented parameter
  ranges are documented per method.
- Elliptic and geodetic functions: Carlson `elliptic_rf` and `elliptic_rd`;
  Legendre `elliptic_k`, `elliptic_e_complete`, `elliptic_f` and `elliptic_e`;
  `jacobi_amplitude`, `inverse_elliptic_e`, `jacobi_sncndn`, `jacobi_epsilon`
  and `jacobi_sncndn_epsilon` (parameter m < 1); and
  `inverse_isometric_latitude`.
- Decimal text: `Rational` and `Real` parse scientific notation (for example
  `"1200e-2"`) exactly, without going through floating point, and `Real`
  retains extreme powers of ten lazily. `Real::try_scientific` with
  `ScientificCase` and `Real::try_decimal` return certified rounded text or a
  typed `Problem::UnknownZero` or `Problem::Exhausted`;
  `Real::diagnostic_decimal` and `Real::fixed_decimal` provide display views.
- Certified enclosures and rounding: `certified_rational_interval`,
  `cached_rational_interval`, `certified_dyadic_interval`,
  `Rational::to_f64_enclosure`, `Real::to_f64_exact_dyadic`, the directional
  `floor_certified`, `ceil_certified`, `round_certified`, `trunc_certified`,
  `fract_certified` and `rem_euclid_certified`, and the total `near_integer`.
- Exact sign and structure queries: `immediate_equality`, `immediate_sign`,
  `is_structural_negation_of`, `definitely_irrational`,
  `exact_quadratic_normal_form`, `quadratic_tower_sign` and
  `radical_tower_sign` (signs decided exactly in iterated square-root
  towers).
- Certified geometric determinant signs that avoid building the exact
  determinant (`Real::certified_affine_det2_sign`, `certified_affine_det3_sign`,
  `certified_incircle2_sign`, `certified_insphere3_sign`,
  `certified_linear_form3_sign`), exact 3x3 and 4x4 rational matrix inverses,
  and signed product-sum kernels for downstream geometry crates.
- `Rational` implements `Eq`, `Hash` and `Product` (owned and borrowed) and
  gains `numerator`, `denominator`, `is_positive`, `is_negative`,
  `perfect_nth_root` and `is_perfect_power`. `Real` implements `Product` and
  gains `Real::integer` and `Real::average_pair`. `Problem` implements `Eq`.
  `Computable` gains the error and normal-distribution kernels, `expm1`,
  `near_integer`, `try_atan2_until` and `try_compare_to_until`.
- The `simple` expression language and `hyperreal` calculator binary accept
  the new elementary, probability and special functions.
- `Real::representation_size` reports the stored size of a value as a
  `RepresentationSize`: distinct expression-graph nodes, the longest path and
  the largest stored rational in bits, visiting at most a caller limit of
  nodes. It reads stored fields only and is meant for observing growth across
  repeated operations, not as a cost bound.

- `Real::exact_rational_normal_form` proves exact zeros whose leaves are
  rationals, `sin(pi * r)` and `cos(pi * r)` for rational `r`, `sqrt(2)`,
  `sqrt(3)` and square roots of rationals with square-free part dividing 6.
  These values lie in one cyclotomic field, so relations between different
  angles, such as `cot(pi/16) = tan(7 pi/16)`, now decide. The proof reduces
  the expression modulo primes at every conjugate and bounds its norm; it
  never approximates, proves only zero, and declines beyond 4096 nodes, field
  order 16384, degree 1024 or 2^25 work charges. It runs right after the
  polynomial attempt.
- `Real::exact_rational_normal_form` applies `sin(a)^2 + sin(b)^2 = 1` and
  `cos(a)^2 + cos(b)^2 = 1` when `a + b` is proved to be exactly `pi / 2`.
  Exact `cos(pi * r)` is stored as `sin(pi * (1/2 - r))`, so this is the form
  in which the sine and cosine of one rational turn meet; before, only a
  sine and a cosine of one argument were related. Turns beyond the
  cyclotomic order limit now decide, such as `sin^2 + cos^2 - 1` at 1/10000,
  1/360 or 12.345 degrees, and so do Booleans that need it: a cube rotated by
  12.345 degrees and back, or rotated by 1/10000 degree and subtracted from
  itself, was `PredicateUndecided` under strict policy.

### Changed

- `Real` and `Computable` are now `Sync` and `RefUnwindSafe`; they were
  neither in 0.13.0. Cloning a `Rational` or `Computable` shares its storage
  instead of copying it.
- More results stay exact: `sin` and `cos` at multiples of π/12 return
  `(√6 ∓ √2)/4` forms; square roots of quadratic surds denest when the
  conjugate norm is a rational square (for example
  `sqrt(x + y + 2*sqrt(x*y))`); and sine, cosine and cotangent of recognized
  `atan(x) + k*π/2` forms are exact, so `sin(atan(4/3) + π/2)` is `3/5`.
- Rational arithmetic, GCD, comparison and approximation paths were
  optimized throughout. Per-change measurements and their limits are recorded
  in [`PERFORMANCE.md`](PERFORMANCE.md).
- License is now `MIT OR Apache-2.0` (was `Apache-2.0`).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.13.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Fixed

- `Real` import accepted a computable payload whose partial operation had an
  operand outside its domain whenever bounded rational admission could not
  refute it, such as `sqrt(-pi)`, `asin(pi)`, `1/(sqrt(2)*sqrt(2) - 2)` or
  `tan(pi/2)`. Strict queries on such a value then certified a sign
  (`asin(pi) > 0`), certified zero under a zero factor or scale, panicked, or
  did not terminate; a Hyperbrep archive with such coordinates validated as a
  trusted model. Import now proves each operand domain, operands first, from
  an exact identity, the existing binary64 filter, or the certified sign
  query that native `Real` operations use. A refuted domain returns
  `Problem::ParseError` and an undecided one `Problem::UnknownCertificate`.
  `Computable` decoding keeps its rejection-only checks, like its unchecked
  native constructors.
- Parsing a decimal with more digits than fit a machine word left its
  fraction unreduced without recording that, although stored parts are read
  as reduced. Equal values then hashed differently and kept dyadic facts
  hidden; the exact binary64 spelling of 10/3 stored a 170-bit denominator
  instead of 52 bits. Such decimals now parse to reduced rationals.
- `Real::to_f64_lossy` and `Real::to_f32_lossy` round exact rationals to the
  nearest value, ties to even, whatever their representation. A rational with
  operands wider than 53 bits, such as a long decimal parsed over a power of
  ten, was rounded twice and could land one ulp away, so equal values could
  convert differently. A value just above a binary32 midpoint could narrow to
  the lower neighbour. A nonzero rational whose denominator exceeds the
  binary64 range returned `0.0`.
- Integer powers of a negative base that fall back to the lazy
  `exp(k * ln x)` construction took the logarithm of the negative base. They
  now use its magnitude and restore the sign by parity.
- `Computable::compare_absolute` could report a perturbation as within
  tolerance from an uncertified magnitude estimate. It now requires a
  certified magnitude.
- A deserialized constant product with a zero power of π evaluated as π
  instead of 1.
- Binary-exponent arithmetic in approximation, shift rewrites and magnitude
  bounds is overflow-checked at the extremes of the `i32` precision range,
  and a nonzero rational whose exponent lies outside that range is no longer
  bounded as zero.

For 0.13.0 and earlier, see [crates.io](https://crates.io/crates/hyperreal/versions).
