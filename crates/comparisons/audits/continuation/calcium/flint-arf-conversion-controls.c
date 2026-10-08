/* Checkpoint 45: bounded finite public conversions, no MPFR output oracle.
 * Exact GMP grid rounding; independent BigInt bit-order search checks stdout.
 * No malformed input, raw mutating limb access, huge exponent or crash probe.
 */
#include "arf.h"
#include <assert.h>
#include <fenv.h>
#include <float.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static const arf_rnd_t modes[] = {ARF_RND_DOWN, ARF_RND_UP, ARF_RND_FLOOR,
                                  ARF_RND_CEIL, ARF_RND_NEAR};
static const unsigned widths[] = {1,  2,  3,  31,  32,  33,  52,  53,  54,
                                  63, 64, 65, 127, 128, 129, 255, 256, 257};
static const slong peaks[] = {-1100, -1076, -1075, -1074, -1023, -1022, -1021,
                              -1,    0,     1,     52,    53,    54,    970,
                              971,   972,   1022,  1023,  1024,  1100};
static const slong grids[] = {-1074, -1022, -53, 0, 971};
static unsigned long fixtures, checks, failures, ties, aliases, imports,
    si_checks;
static mpz_t mag, quotient, remainder, denominator, expected, actual, scratch;

static void check(int ok, const char *what) {
  checks++;
  if (!ok) {
    failures++;
    fprintf(stderr, "FAIL fixture %lu check %lu: %s\n", fixtures, checks, what);
  }
}
static void canonical(mpz_t m, slong *e) {
  if (mpz_sgn(m) == 0) {
    *e = 0;
    return;
  }
  mp_bitcnt_t v = mpz_scan1(m, 0);
  mpz_tdiv_q_2exp(m, m, v);
  *e += (slong)v;
}
static void decode(mpz_t m, slong *e, const arf_t x) {
  assert(arf_is_finite(x));
  if (arf_is_zero(x)) {
    mpz_set_ui(m, 0);
    *e = 0;
    return;
  }
  assert(!COEFF_IS_MPZ(ARF_EXP(x)));
  nn_srcptr p;
  slong n;
  ARF_GET_MPN_READONLY(p, n, x);
  mpz_import(m, (size_t)n, -1, sizeof(ulong), 0, 0, p);
  if (ARF_SGNBIT(x))
    mpz_neg(m, m);
  *e = ARF_EXP(x) - n * FLINT_BITS;
  assert(*e > -4000 && *e < 4000);
  canonical(m, e);
}
static void emit(const arf_t x, const mpz_t m, slong e) {
  slong ae;
  mpz_set(expected, m);
  canonical(expected, &e);
  decode(actual, &ae, x);
  check(ae == e && mpz_cmp(actual, expected) == 0, "full dyadic result");
  gmp_printf("[\"%Zx\",%ld]", actual, (long)ae);
}
/* Signed rounding of m*2^e to an integer, with exact remainder/halfway tests.
 */
static int integer_oracle(const mpz_t m, slong e, unsigned mode) {
  const int negative = mpz_sgn(m) < 0;
  mpz_abs(mag, m);
  mpz_set_ui(denominator, 1);
  if (e >= 0)
    mpz_mul_2exp(mag, mag, (mp_bitcnt_t)e);
  else
    mpz_mul_2exp(denominator, denominator, (mp_bitcnt_t)-e);
  mpz_fdiv_qr(quotient, remainder, mag, denominator);
  const int inexact = mpz_sgn(remainder) != 0;
  int up = mode == 1 || (mode == 2 && negative) || (mode == 3 && !negative);
  if (mode == 4) {
    mpz_mul_2exp(scratch, remainder, 1);
    const int c = mpz_cmp(scratch, denominator);
    ties += c == 0;
    up = c > 0 || (c == 0 && mpz_odd_p(quotient));
  }
  if (inexact && up)
    mpz_add_ui(quotient, quotient, 1);
  if (negative)
    mpz_neg(quotient, quotient);
  return inexact;
}
static uint64_t double_oracle(const mpz_t m, slong e, unsigned mode) {
  if (!mpz_sgn(m))
    return 0;
  const int negative = mpz_sgn(m) < 0;
  const uint64_t sign = negative ? UINT64_C(0x8000000000000000) : 0;
  const uint64_t inf = UINT64_C(0x7ff0000000000000);
  slong top = (slong)mpz_sizeinbase(m, 2) - 1 + e;
  if (top > 1023) {
    const int up = mode == 4 || mode == 1 || (mode == 2 && negative) ||
                   (mode == 3 && !negative);
    return sign | (up ? inf : inf - 1);
  }
  slong grid = top < -1022 ? -1074 : top - 52;
  integer_oracle(m, e - grid, mode);
  mpz_abs(quotient, quotient);
  assert(mpz_fits_ulong_p(quotient));
  uint64_t q = (uint64_t)mpz_get_ui(quotient);
  if (top < -1022) {
    assert(q <= (UINT64_C(1) << 52));
    return sign | q;
  }
  if (q == (UINT64_C(1) << 53)) {
    q >>= 1;
    top++;
  }
  if (top > 1023)
    return sign | inf;
  assert(q >= (UINT64_C(1) << 52) && q < (UINT64_C(1) << 53));
  return sign | ((uint64_t)(top + 1023) << 52) | (q - (UINT64_C(1) << 52));
}
static void bits_dyadic(mpz_t m, slong *e, uint64_t bits) {
  const unsigned be = (unsigned)((bits >> 52) & 2047);
  assert(be != 2047);
  mpz_set_ui(m, (ulong)(bits & UINT64_C(0xfffffffffffff)));
  if (be)
    mpz_setbit(m, 52);
  *e = be ? (slong)be - 1075 : -1074;
  if (bits >> 63)
    mpz_neg(m, m);
  canonical(m, e);
}
static int sign(int v) { return (v > 0) - (v < 0); }
static void set_q(mpq_t q, const mpz_t m, slong e) {
  mpq_set_z(q, m);
  if (e >= 0)
    mpq_mul_2exp(q, q, (mp_bitcnt_t)e);
  else
    mpq_div_2exp(q, q, (mp_bitcnt_t)-e);
}
static void run_case(const mpz_t m, slong e, arf_t previous, mpq_t pq) {
  arf_t x, z;
  arf_init(x);
  arf_init(z);
  fmpz_t f, fe;
  fmpz_init(f);
  fmpz_init(fe);
  fmpz_set_mpz(f, m);
  arf_set_fmpz(x, f);
  arf_mul_2exp_si(x, x, e);
  mpq_t q, aq, apq, power;
  mpq_inits(q, aq, apq, power, NULL);
  set_q(q, m, e);
  mpq_abs(aq, q);
  mpq_abs(apq, pq);
  gmp_printf("{\"id\":%lu,\"m\":\"%Zx\",\"e\":%ld,\"input\":", fixtures, m,
             (long)e);
  emit(x, m, e);
  printf(",\"double\":[");
  for (unsigned mode = 0; mode < 5; mode++) {
    if (mode)
      printf(",");
    const uint64_t eb = double_oracle(m, e, mode);
    const double d = arf_get_d(x, modes[mode]);
    uint64_t bits;
    memcpy(&bits, &d, sizeof(bits));
    check(bits == eb, "complete binary64 bits");
    printf("[\"%016" PRIx64 "\",", bits);
    if (((bits >> 52) & 2047) == 2047)
      printf("null");
    else {
      slong de;
      bits_dyadic(scratch, &de, bits);
      arf_set_si(z, 7);
      arf_set_d(z, d);
      emit(z, scratch, de);
      imports++;
    }
    printf("]");
  }
  printf("],\"integer\":[");
  for (unsigned mode = 0; mode < 5; mode++) {
    if (mode)
      printf(",");
    const int ie = integer_oracle(m, e, mode);
    fmpz_set_si(f, 7);
    const int ret = arf_get_fmpz(f, x, modes[mode]);
    fmpz_get_mpz(actual, f);
    check(ret == ie && mpz_cmp(actual, quotient) == 0, "integer value/inexact");
    gmp_printf("[\"%Zx\",%d,", actual, ret);
    if (mpz_fits_slong_p(quotient)) {
      slong si = arf_get_si(x, modes[mode]);
      check(si == mpz_get_si(quotient), "proved-in-range signed conversion");
      si_checks++;
      printf("\"%ld\"", (long)si);
    } else
      printf("null");
    printf("]");
  }
  printf("],\"integralOps\":[");
  for (unsigned op = 0; op < 3; op++) {
    const unsigned mode = op + 2;
    integer_oracle(m, e, mode);
    void (*fn)(arf_ptr, arf_srcptr) = op == 0   ? arf_floor
                                      : op == 1 ? arf_ceil
                                                : arf_nint;
    for (unsigned alias = 0; alias < 2; alias++) {
      if (op || alias)
        printf(",");
      if (alias) {
        arf_set(z, x);
        fn(z, z);
        aliases++;
      } else {
        arf_set_si(z, 7);
        fn(z, x);
      }
      emit(z, quotient, 0);
    }
  }
  slong top = mpz_sgn(m) ? (slong)mpz_sizeinbase(m, 2) + e : 0;
  printf("],\"frexp\":[");
  for (unsigned alias = 0; alias < 2; alias++) {
    if (alias)
      printf(",");
    fmpz_set_si(fe, 7);
    if (alias) {
      arf_set(z, x);
      arf_frexp(z, fe, z);
      aliases++;
    } else {
      arf_set_si(z, 7);
      arf_frexp(z, fe, x);
    }
    check(fmpz_equal_si(fe, top), "frexp exponent");
    printf("[");
    emit(z, m, e - top);
    printf(",%ld]", (long)fmpz_get_si(fe));
  }
  slong bottom = e;
  mpz_set(scratch, m);
  canonical(scratch, &bottom);
  const int isint = !mpz_sgn(m) || bottom >= 0;
  const int gotint = arf_is_int(x);
  check(gotint == isint, "integrality");
  printf("],\"isInt\":%d,\"bounds\":[", gotint);
  const slong le = !mpz_sgn(m) ? 0 : top - (mpz_cmpabs_ui(scratch, 1) == 0);
  arf_abs_bound_le_2exp_fmpz(fe, x);
  check(fmpz_equal_si(fe, le), "nonstrict exponent bound");
  printf("%ld,", (long)fmpz_get_si(fe));
  arf_abs_bound_lt_2exp_fmpz(fe, x);
  check(fmpz_equal_si(fe, top), "strict exponent bound");
  printf("%ld,", (long)fmpz_get_si(fe));
  slong bsi = arf_abs_bound_lt_2exp_si(x);
  check(bsi == top, "bounded signed exponent");
  printf("%ld],\"powers\":[", (long)bsi);
  const slong pe[] = {-1074, -1, 0, bottom - 1, bottom, bottom + 1, top};
  for (unsigned j = 0; j < 7; j++) {
    if (j)
      printf(",");
    mpz_set_ui(scratch, 1);
    set_q(power, scratch, pe[j]);
    const int ii = arf_is_int_2exp_si(x, pe[j]);
    const int c = sign(arf_cmp_2exp_si(x, pe[j]));
    const int ca = sign(arf_cmpabs_2exp_si(x, pe[j]));
    check(ii == (!mpz_sgn(m) || bottom >= pe[j]), "scaled integrality");
    check(c == sign(mpq_cmp(q, power)) && ca == sign(mpq_cmp(aq, power)),
          "power comparisons");
    printf("[%ld,%d,%d,%d]", (long)pe[j], ii, c, ca);
  }
  const int c = sign(arf_cmp(x, previous)), ca = sign(arf_cmpabs(x, previous));
  check(c == sign(mpq_cmp(q, pq)) && ca == sign(mpq_cmp(aq, apq)),
        "previous-value comparisons");
  printf("],\"previousCmp\":[%d,%d],\"preserved\":", c, ca);
  emit(x, m, e);
  printf("}\n");
  arf_set(previous, x);
  mpq_set(pq, q);
  fixtures++;
  mpq_clears(q, aq, apq, power, NULL);
  fmpz_clear(f);
  fmpz_clear(fe);
  arf_clear(x);
  arf_clear(z);
}
int main(void) {
  assert(FLINT_BITS == 64 && sizeof(double) == sizeof(uint64_t));
  assert(DBL_MANT_DIG == 53 && DBL_MAX_EXP == 1024 &&
         fegetround() == FE_TONEAREST);
  mpz_inits(mag, quotient, remainder, denominator, expected, actual, scratch,
            NULL);
  mpz_t m, base;
  mpz_inits(m, base, NULL);
  arf_t previous;
  arf_init(previous);
  mpq_t pq;
  mpq_init(pq);
  for (unsigned w = 0; w < sizeof(widths) / sizeof(*widths); w++)
    for (unsigned v = 0; v < 3; v++) {
      mpz_set_ui(base, 0);
      mpz_setbit(base, widths[w] - 1);
      if (v == 1) {
        mpz_mul_2exp(base, base, 1);
        mpz_sub_ui(base, base, 1);
      }
      if (v == 2) {
        mpz_setbit(base, widths[w] / 2);
        mpz_setbit(base, 0);
      }
      for (unsigned p = 0; p < sizeof(peaks) / sizeof(*peaks); p++)
        for (unsigned s = 0; s < 2; s++) {
          mpz_set(m, base);
          if (s)
            mpz_neg(m, m);
          run_case(m, peaks[p] + 1 - (slong)mpz_sizeinbase(m, 2), previous, pq);
        }
    }
  for (unsigned g = 0; g < sizeof(grids) / sizeof(*grids); g++)
    for (unsigned v = 0; v < 8; v++) {
      if (v < 3)
        mpz_set_ui(base, v);
      else {
        mpz_set_ui(base, 1);
        mpz_mul_2exp(base, base, v < 6 ? 52 : 53);
        if (v == 3 || v == 7)
          mpz_sub_ui(base, base, 1);
        if (v == 5)
          mpz_add_ui(base, base, 1);
        if (v == 6)
          mpz_sub_ui(base, base, 2);
      }
      mpz_mul_2exp(base, base, 2);
      for (unsigned d = 1; d <= 3; d++)
        for (unsigned s = 0; s < 2; s++) {
          mpz_add_ui(m, base, d);
          if (s)
            mpz_neg(m, m);
          run_case(m, grids[g] - 2, previous, pq);
        }
    }
  mpz_set_ui(m, 0);
  run_case(m, 0, previous, pq);
  mpq_clear(pq);
  arf_clear(previous);
  mpz_clears(m, base, NULL);
  mpz_clears(mag, quotient, remainder, denominator, expected, actual, scratch,
             NULL);
  flint_cleanup();
  printf("{\"summary\":true,\"fixtures\":%lu,\"checks\":%lu,\"failures\":%lu,"
         "\"ties\":%lu,\"aliases\":%lu,\"finiteImports\":%lu,"
         "\"signedConversions\":%lu}\n",
         fixtures, checks, failures, ties, aliases, imports, si_checks);
  return failures ? 1 : 0;
}
