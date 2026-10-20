# Changelog

## [0.14.0] - Unreleased

### Breaking

- With the `serde` feature, `Real::from_json`, `Real::from_bytes`, conversion
  from a CBOR `Value` and the `Deserialize` impl for `Real` now validate the
  decoded value instead of trusting it. Invalid named certificates, an opaque
  value without its computable payload, a payload that contradicts its class,
  and known domain violations return `Problem::ParseError`; a certificate or
  payload relationship that cannot be proved returns the new
  `Problem::UnknownCertificate`. The wire layout is unchanged.
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

- `Real::to_f64_lossy` returned `0.0` for a nonzero exact rational whose
  denominator exceeds the binary64 range. It now returns a finite
  approximation within one ulp.
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
