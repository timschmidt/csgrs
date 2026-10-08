# Checkpoint 81 — reciprocal inverses, rounding and binary import

Read both pinned donors file by file and line by line: six implementation/test
pairs (asec_pi, acsc_pi, floor, ceil, set_d, set_re_im_d) and nine scalar helpers
(get_fmpq, get_fmpz, numerator, denominator, height, height_bits, swap, inlines,
phi). Forty-two new files / 1,902 new lines: 979 archived, 923 current. Manuals
and the recorded Hyper source ranges are rereads, not new donor credit.

Do not broaden a failed inverse recognition into negative mathematical proof.
Only inspect successful float/inverse output values. Non-finite import is a
documented failure case; failed destinations are unspecified, not required to
remain unchanged. Signed zero imports as the mathematical value zero. Floor and
ceiling of a complex donor value refer to its real part. Its denominator is the
leading coefficient of the minimal polynomial, not a minimal integral multiplier.

Bind the unchanged live/candidate maps, selected donor hashes, pinned native
libraries/configuration, collector, oracle and protocol before building one
small executable in a new temporary directory. Reuse the existing libraries.

Native corpus: 19,923 rows plus terminal, including:

- 18,432 scalar imports: every binary32/binary64 exponent, both signs and four
  significands (zero, one, midpoint bit and all ones). Widening finite f32 to f64
  is exact. Infinity and multiple quiet/signaling NaN bit patterns return failure.
- 256 complex imports from a full 16-by-16 boundary cross product, including
  signed zeros, subnormal/normal boundaries, huge finite values and non-finites.
- 576 reciprocal-inverse angle rows, plus twelve rational/imaginary/zero controls.
  All pi/12 residues, three periods, unreduced factors 1/3, initial/256-bit cached
  states. Check full input polynomials, both components and exact principal output.
- 644 floor/ceiling rows: seven integer origins through +/-2^256, 23 rational or
  sqrt(2) offsets including +/-2^-1024 and half-boundary offsets, real and added
  i*sqrt(3), initial/1536-bit cached states. Query rounding before serializing a
  2048-bit enclosure. Check primitive minimal polynomial, exact sign inequalities,
  coefficient height, denominator and numerator polynomial (separate/in-place).
- Three phi/swap/integer/rational extraction records.

Independent Node oracles decode IEEE bits into integer ratios; use exact
Q(sqrt(2),sqrt(3)) field signs and Galois products for reciprocal trig; derive
complex quadratic/biquadratic polynomials by exact conjugate products. Check
both component containment, enclosure widths and complete ordered membership.
The numerator polynomial is independently obtained by an invertible rational
change of variable and primitive normalization. No same-library round trip or
approximate overlap serves as the correctness oracle. Native and Memcheck streams
must match. Corruption controls and strict failure-output membership are required.

Compare live Hyper exact dyadic imports, certified floor/ceiling and multivalued
near_integer, plus explicitly derived acos(1/x)/asin(1/x) (not new asec/acsc APIs).
Preserve Exhausted/Unknown; no approximate integer may masquerade as certified.
Any reduced-candidate rounding optimization remains a hypothesis until isolated
correctness, full regression and matched-cost gates justify retention. This
source/capability collector is not a benchmark or a new production transfer.
