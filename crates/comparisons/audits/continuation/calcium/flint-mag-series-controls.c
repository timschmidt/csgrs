/* Checkpoint 41. Public bounded numerical inputs only. Exact combinatorial
 * targets use GMP recurrences, not FLINT factorial/binomial/Bernoulli code.
 * Positive polylog tails include an independent eventual geometric remainder.
 * Valid string roundtrips only: no malformed parser or I/O-failure tests. */
#include "arf.h"
#include "fmpq.h"
#include "mag.h"
#include <assert.h>
#include <fenv.h>
#include <math.h>
#include <mpfr.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned long exact_checks, directed_checks, preservation, aliases;
static unsigned long roundtrips, infinite_polylog, failures;
static mpq_t target, decoded, scratch;
static mpfr_t low, high, val, xref;

static void check(int ok, const char *kind, unsigned long id) {
  if (!ok) {
    failures++;
    fprintf(stderr, "FAIL %s id=%lu\n", kind, id);
  }
}
static void mag_q(mpq_t q, const mag_t x) {
  assert(!mag_is_inf(x) && !COEFF_IS_MPZ(MAG_EXP(x)));
  long e = MAG_EXP(x);
  assert(e >= -100000 && e <= 100000);
  if (mag_is_zero(x))
    mpq_set_ui(q, 0, 1);
  else {
    assert(MAG_MAN(x) >= MAG_ONE_HALF && MAG_MAN(x) < (1UL << MAG_BITS));
    mpq_set_ui(q, MAG_MAN(x), 1);
    if (e >= MAG_BITS)
      mpq_mul_2exp(q, q, e - MAG_BITS);
    else
      mpq_div_2exp(q, q, MAG_BITS - e);
  }
}
static void mag_mpfr(mpfr_t y, const mag_t x) {
  if (mag_is_inf(x)) {
    mpfr_set_inf(y, 1);
    return;
  }
  mag_q(decoded, x);
  assert(mpfr_set_q(y, decoded, MPFR_RNDN) == 0);
}
static void emit(const mag_t y) {
  printf("[%lu,%ld,%d]", MAG_MAN(y), (long)MAG_EXP(y), mag_is_inf(y));
}
static void exact_bound(const mag_t y, int lower, const char *kind,
                        unsigned long id) {
  mag_q(decoded, y);
  exact_checks++;
  int c = mpq_cmp(decoded, target);
  check(lower ? c <= 0 : c >= 0, kind, id);
  emit(y);
}
static void directed_bound(const mag_t y, const char *kind, unsigned long id) {
  mag_mpfr(val, y);
  directed_checks++;
  check(mpfr_cmp(val, high) >= 0, kind, id);
  emit(y);
}

static void combinatorial(void) {
  mag_t y;
  mag_init(y);
  mpz_t factorial, binomial, power;
  mpz_inits(factorial, binomial, power, (mpz_ptr)0);
  mpz_set_ui(factorial, 1);
  for (unsigned long n = 0; n <= 4096; n++) {
    if (n)
      mpz_mul_ui(factorial, factorial, n);
    mpq_set_z(target, factorial);
    mag_set_ui(y, 7);
    mag_fac_ui(y, n);
    printf("{\"kind\":\"fac\",\"n\":%lu,\"results\":[", n);
    exact_bound(y, 0, "fac", n);
    printf(",");
    mpq_inv(target, target);
    mag_set_ui(y, 7);
    mag_rfac_ui(y, n);
    exact_bound(y, 0, "rfac", n);
    printf("]}\n");
  }
  for (unsigned long ix = 0; ix < 262; ix++) {
    const unsigned long extra[] = {511, 512, 513, 1024};
    unsigned long n = ix < 258 ? ix : extra[ix - 258];
    mpz_set_ui(binomial, 1);
    printf("{\"kind\":\"binomial\",\"n\":%lu,\"results\":[", n);
    for (unsigned long k = 0; k <= n + 1; k++) {
      if (k > n)
        mpz_set_ui(binomial, 0);
      else if (k) {
        mpz_mul_ui(binomial, binomial, n - k + 1);
        assert(mpz_divisible_ui_p(binomial, k));
        mpz_divexact_ui(binomial, binomial, k);
      }
      mpq_set_z(target, binomial);
      mag_set_ui(y, 7);
      mag_bin_uiui(y, n, k);
      if (k)
        printf(",");
      exact_bound(y, 0, "binomial", n * 2048 + k);
    }
    printf("]}\n");
  }
  const unsigned long ms[] = {
      1,   2,   3,   7,    31,           32,           33,
      255, 256, 257, 1024, 1073741823UL, 1073741824UL, 1073741825UL};
  const unsigned long ns[] = {0,  1,   2,   3,   7,   31, 32,
                              33, 127, 255, 256, 257, 512};
  for (unsigned long i = 0; i < sizeof(ms) / sizeof(*ms); i++)
    for (unsigned long j = 0; j < sizeof(ns) / sizeof(*ns); j++) {
      unsigned long m = ms[i], n = ns[j];
      mpz_ui_pow_ui(mpq_numref(target), m + 1, n);
      mpz_ui_pow_ui(mpq_denref(target), m, n);
      mpq_canonicalize(target);
      mag_set_ui(y, 7);
      mag_binpow_uiui(y, m, n);
      printf("{\"kind\":\"binpow\",\"m\":%lu,\"n\":%lu,\"result\":", m, n);
      exact_bound(y, 0, "binpow", i * 13 + j);
      printf("}\n");
    }
  /* Akiyama-Tanigawa exact rational recurrence; B1 has the positive
   * convention, which is immaterial after absolute value. */
  mpq_t a[129], mult;
  for (unsigned long i = 0; i < 129; i++)
    mpq_init(a[i]);
  mpq_init(mult);
  mpz_set_ui(factorial, 1);
  for (unsigned long n = 0; n <= 128; n++) {
    if (n)
      mpz_mul_ui(factorial, factorial, n);
    mpq_set_ui(a[n], 1, n + 1);
    for (unsigned long j = n; j > 0; j--) {
      mpq_sub(a[j - 1], a[j - 1], a[j]);
      mpq_set_ui(mult, j, 1);
      mpq_mul(a[j - 1], a[j - 1], mult);
    }
    mpq_abs(target, a[0]);
    if (n == 0)
      assert(mpq_cmp_ui(target, 1, 1) == 0);
    if (n == 1)
      assert(mpq_cmp_ui(target, 1, 2) == 0);
    if (n == 2)
      assert(mpq_cmp_ui(target, 1, 6) == 0);
    if (n == 4)
      assert(mpq_cmp_ui(target, 1, 30) == 0);
    mpz_mul(mpq_denref(target), mpq_denref(target), factorial);
    mpq_canonicalize(target);
    mag_set_ui(y, 7);
    mag_bernoulli_div_fac_ui(y, n);
    printf("{\"kind\":\"bernoulli\",\"n\":%lu,\"result\":", n);
    exact_bound(y, 0, "bernoulli", n);
    printf("}\n");
  }
  for (unsigned long i = 0; i < 129; i++)
    mpq_clear(a[i]);
  mpq_clear(mult);
  mpz_clears(factorial, binomial, power, (mpz_ptr)0);
  mag_clear(y);
}

static void geometric(void) {
  const unsigned long mans[] = {
      0,           536870912UL, 536870912UL,  536870912UL,
      805306368UL, 939524096UL, 1073741822UL, 1073741823UL};
  const long exps[] = {0, -59, -3, 0, 0, 0, 0, 0};
  const unsigned long ns[] = {0, 1, 2, 3, 7, 31, 64, 127, 255, 256, 257};
  mag_t x, y, z, original;
  mag_init(x);
  mag_init(y);
  mag_init(z);
  mag_init(original);
  mpq_t xq, den;
  mpq_inits(xq, den, (mpq_ptr)0);
  for (unsigned long i = 0; i < 8; i++) {
    mag_zero(x);
    if (mans[i]) {
      MAG_MAN(x) = mans[i];
      fmpz_set_si(MAG_EXPREF(x), exps[i]);
    }
    mag_q(xq, x);
    mag_set(original, x);
    mpq_set_ui(den, 1, 1);
    mpq_sub(den, den, xq);
    for (unsigned long j = 0; j < 11; j++) {
      unsigned long n = ns[j], id = i * 11 + j;
      mpz_pow_ui(mpq_numref(target), mpq_numref(xq), n);
      mpz_pow_ui(mpq_denref(target), mpq_denref(xq), n);
      mpq_div(target, target, den);
      mag_set_ui(y, 7);
      mag_geom_series(y, x, n);
      preservation++;
      check(mag_equal(x, original), "geom-input", id);
      mag_set(z, x);
      mag_geom_series(z, z, n);
      aliases++;
      check(mag_equal(y, z), "geom-alias", id);
      printf("{\"kind\":\"geom\",\"i\":%lu,\"n\":%lu,\"results\":[", i, n);
      exact_bound(y, 0, "geom", id);
      printf(",");
      exact_bound(z, 0, "geom-alias-bound", id);
      printf("]}\n");
    }
  }
  mpq_clears(xq, den, (mpq_ptr)0);
  mag_clear(x);
  mag_clear(y);
  mag_clear(z);
  mag_clear(original);
}

/* Positive sum z^k log(k)^d/k^sigma, N<=31, |sigma|<=8, d<=4.
 * All k beyond K=N+512 satisfy log(k)>=1. Hence the subsequent ratio
 * is at most z*(1+1/(K+1))^(max(-sigma,0)+d). Use that upper ratio
 * to enclose the complete tail following the independently summed prefix. */
static void poly_reference(const mpfr_t x, long sigma, unsigned long d,
                           unsigned long n, mpfr_t *logs_l, mpfr_t *logs_h,
                           long prec) {
  mpfr_t pl, ph, tl, th, kl, kh, den, q;
  mpfr_inits2(prec, pl, ph, tl, th, kl, kh, den, q, (mpfr_ptr)0);
  mpfr_pow_ui(pl, x, n, MPFR_RNDD);
  mpfr_pow_ui(ph, x, n, MPFR_RNDU);
  mpfr_set_zero(low, 1);
  mpfr_set_zero(high, 1);
  unsigned long end = n + 512;
  for (unsigned long k = n; k <= end + 1; k++) {
    mpfr_pow_ui(kl, logs_l[k], d, MPFR_RNDD);
    mpfr_pow_ui(kh, logs_h[k], d, MPFR_RNDU);
    mpfr_mul(tl, pl, kl, MPFR_RNDD);
    mpfr_mul(th, ph, kh, MPFR_RNDU);
    mpfr_set_ui(den, k, MPFR_RNDN);
    assert(mpfr_pow_ui(den, den, (unsigned long)(sigma < 0 ? -sigma : sigma),
                       MPFR_RNDN) == 0);
    if (sigma >= 0) {
      mpfr_div(tl, tl, den, MPFR_RNDD);
      mpfr_div(th, th, den, MPFR_RNDU);
    } else {
      mpfr_mul(tl, tl, den, MPFR_RNDD);
      mpfr_mul(th, th, den, MPFR_RNDU);
    }
    if (k <= end) {
      mpfr_add(low, low, tl, MPFR_RNDD);
      mpfr_add(high, high, th, MPFR_RNDU);
    }
    mpfr_mul(pl, pl, x, MPFR_RNDD);
    mpfr_mul(ph, ph, x, MPFR_RNDU);
  }
  mpfr_set_ui(q, end + 1, MPFR_RNDN);
  mpfr_ui_div(q, 1, q, MPFR_RNDU);
  mpfr_add_ui(q, q, 1, MPFR_RNDU);
  mpfr_pow_ui(q, q, (sigma < 0 ? (unsigned long)-sigma : 0) + d, MPFR_RNDU);
  mpfr_mul(q, q, x, MPFR_RNDU);
  assert(mpfr_cmp_ui(q, 1) < 0);
  mpfr_ui_sub(den, 1, q, MPFR_RNDD);
  mpfr_div(th, th, den, MPFR_RNDU);
  mpfr_add(high, high, th, MPFR_RNDU);
  assert(mpfr_cmp(low, high) <= 0);
  mpfr_clears(pl, ph, tl, th, kl, kh, den, q, (mpfr_ptr)0);
}

static void analytic_tails(long prec) {
  mpfr_t logs_l[545], logs_h[545], term;
  mpfr_init2(term, prec);
  for (unsigned long k = 0; k < 545; k++) {
    mpfr_init2(logs_l[k], prec);
    mpfr_init2(logs_h[k], prec);
    if (k >= 2) {
      mpfr_set_ui(term, k, MPFR_RNDN);
      mpfr_log(logs_l[k], term, MPFR_RNDD);
      mpfr_log(logs_h[k], term, MPFR_RNDU);
    }
  }
  const unsigned long mans[] = {0,           536870912UL, 536870912UL,
                                805306368UL, 939524096UL, 1006632960UL};
  const long exps[] = {0, -2, 0, 0, 0, 0}, sigmas[] = {-3, -1, 0, 1, 3, 8};
  const unsigned long ds[] = {0, 1, 2, 4}, ns[] = {2, 3, 8, 31};
  mag_t x, y, z, original;
  mag_init(x);
  mag_init(y);
  mag_init(z);
  mag_init(original);
  unsigned long id = 0;
  for (unsigned long i = 0; i < 6; i++) {
    mag_zero(x);
    if (mans[i]) {
      MAG_MAN(x) = mans[i];
      fmpz_set_si(MAG_EXPREF(x), exps[i]);
    }
    mag_set(original, x);
    mag_mpfr(xref, x);
    for (unsigned long si = 0; si < 6; si++)
      for (unsigned long di = 0; di < 4; di++)
        for (unsigned long ni = 0; ni < 4; ni++) {
          long sigma = sigmas[si];
          unsigned long d = ds[di], n = ns[ni];
          poly_reference(xref, sigma, d, n, logs_l, logs_h, prec);
          mag_set_ui(y, 7);
          mag_polylog_tail(y, x, sigma, d, n);
          preservation++;
          check(mag_equal(x, original), "polylog-input", id);
          mag_set(z, x);
          mag_polylog_tail(z, z, sigma, d, n);
          aliases++;
          check(mag_equal(y, z), "polylog-alias", id);
          if (mag_is_inf(y))
            infinite_polylog += 2;
          printf("{\"kind\":\"polylog\",\"precision\":%ld,\"id\":%lu,\"i\":%lu,"
                 "\"sigma\":%ld,\"d\":%lu,\"n\":%lu,\"results\":[",
                 prec, id++, i, sigma, d, n);
          directed_bound(y, "polylog", id);
          printf(",");
          directed_bound(z, "polylog-alias-bound", id);
          printf("]}\n");
        }
  }
  for (unsigned long s = 2; s <= 32; s++)
    for (unsigned long a = 1; a <= 17; a++) {
      mpfr_zeta_ui(low, s, MPFR_RNDD);
      mpfr_zeta_ui(high, s, MPFR_RNDU);
      for (unsigned long k = 1; k < a; k++) {
        mpfr_set_ui(term, k, MPFR_RNDN);
        assert(mpfr_pow_ui(term, term, s, MPFR_RNDN) == 0);
        mpfr_ui_div(val, 1, term, MPFR_RNDU);
        mpfr_sub(low, low, val, MPFR_RNDD);
        mpfr_ui_div(val, 1, term, MPFR_RNDD);
        mpfr_sub(high, high, val, MPFR_RNDU);
      }
      assert(mpfr_sgn(low) > 0 && mpfr_cmp(low, high) <= 0);
      mag_set_ui(y, 7);
      mag_hurwitz_zeta_uiui(y, s, a);
      printf("{\"kind\":\"hurwitz\",\"precision\":%ld,\"s\":%lu,\"a\":%lu,"
             "\"result\":",
             prec, s, a);
      directed_bound(y, "hurwitz", s * 32 + a);
      printf("}\n");
    }
  mag_clear(x);
  mag_clear(y);
  mag_clear(z);
  mag_clear(original);
  mpfr_clear(term);
  for (unsigned long k = 0; k < 545; k++) {
    mpfr_clear(logs_l[k]);
    mpfr_clear(logs_h[k]);
  }
}

static void double_conversion(uint64_t bits, unsigned long id) {
  double d;
  memcpy(&d, &bits, sizeof d);
  assert(isfinite(d));
  const long scales[] = {-1024, -31, 0, 31, 1024};
  mag_t y;
  mag_init(y);
  fmpz_t e;
  fmpz_init(e);
  mpq_t base;
  mpq_init(base);
  mpq_set_d(base, fabs(d));
  printf("{\"kind\":\"double\",\"id\":%lu,\"bits\":\"%016llx\",\"results\":[",
         id, (unsigned long long)bits);
  for (unsigned long i = 0; i < 5; i++) {
    long scale = scales[i];
    fmpz_set_si(e, scale);
    mpq_set(target, base);
    if (scale >= 0)
      mpq_mul_2exp(target, target, scale);
    else
      mpq_div_2exp(target, target, -scale);
    if (i)
      printf(",");
    printf("[");
    mag_set_ui(y, 7);
    mag_set_d_2exp_fmpz(y, d, e);
    exact_bound(y, 0, "set-double-scaled", id);
    printf(",");
    mag_set_ui(y, 7);
    mag_set_d_2exp_fmpz_lower(y, d, e);
    exact_bound(y, 1, "set-double-scaled-lower", id);
    if (scale == 0) {
      printf(",");
      mag_set_ui(y, 7);
      mag_set_d(y, d);
      exact_bound(y, 0, "set-double", id);
      printf(",");
      mag_set_ui(y, 7);
      mag_set_d_lower(y, d);
      exact_bound(y, 1, "set-double-lower", id);
    }
    printf("]");
  }
  printf("]}\n");
  mpq_clear(base);
  fmpz_clear(e);
  mag_clear(y);
}

static void conversions(void) {
  const uint64_t small[] = {
      0, 1, 2, 3, UINT64_C(0x000ffffffffffffe), UINT64_C(0x000fffffffffffff)};
  const long es[] = {-1022, -1000, -970, -31, -30, -1, 0,   1,    29,
                     30,    31,    52,   53,  63,  64, 999, 1000, 1023};
  const uint64_t fractions[] = {
      0, 1, 4194303, 4194304, 8388607, 8388608, UINT64_C(0xfffffffffffff)};
  unsigned long id = 0;
  for (unsigned long i = 0; i < 6; i++)
    for (unsigned long s = 0; s < 2; s++)
      double_conversion(small[i] | ((uint64_t)s << 63), id++);
  for (unsigned long i = 0; i < 18; i++)
    for (unsigned long j = 0; j < 7; j++)
      for (unsigned long s = 0; s < 2; s++)
        double_conversion(((uint64_t)(es[i] + 1023) << 52) | fractions[j] |
                              ((uint64_t)s << 63),
                          id++);
  const unsigned long mans[] = {
      536870912UL, 536870913UL, 536870914UL,  536870915UL,  671088639UL,
      671088640UL, 671088641UL, 1073741821UL, 1073741822UL, 1073741823UL};
  const long exps[] = {-1075, -1001, -1000, -999, -31,  -30, -29,
                       -1,    0,     1,     29,   30,   31,  63,
                       64,    65,    999,   1000, 1001, 1024};
  mag_t x, y, original;
  mag_init(x);
  mag_init(y);
  mag_init(original);
  fmpq_t q;
  fmpq_init(q);
  fmpz_t z;
  fmpz_init(z);
  mpz_t got, want;
  mpz_inits(got, want, (mpz_ptr)0);
  for (unsigned long i = 0; i < 201; i++) {
    mag_zero(x);
    if (i) {
      MAG_MAN(x) = mans[(i - 1) % 10];
      fmpz_set_si(MAG_EXPREF(x), exps[(i - 1) / 10]);
    }
    mag_set(original, x);
    mag_q(target, x);
    mag_get_fmpq(q, x);
    fmpq_get_mpq(scratch, q);
    exact_checks++;
    check(mpq_equal(target, scratch), "get-fmpq", i);
    mag_get_fmpz(z, x);
    fmpz_get_mpz(got, z);
    mpz_cdiv_q(want, mpq_numref(target), mpq_denref(target));
    exact_checks++;
    check(mpz_cmp(got, want) == 0, "get-ceil", i);
    mag_get_fmpz_lower(z, x);
    fmpz_get_mpz(got, z);
    mpz_fdiv_q(want, mpq_numref(target), mpq_denref(target));
    exact_checks++;
    check(mpz_cmp(got, want) == 0, "get-floor", i);
    double d = mag_get_d(x);
    uint64_t bits;
    memcpy(&bits, &d, sizeof bits);
    exact_checks++;
    if (isinf(d))
      check(i && MAG_EXP(x) > 1000, "get-double-infinity", i);
    else {
      mpq_set_d(scratch, d);
      check(mpq_cmp(scratch, target) >= 0, "get-double-upper", i);
      if (!i || (MAG_EXP(x) >= -1000 && MAG_EXP(x) <= 1000))
        check(mpq_equal(scratch, target), "get-double-exact", i);
    }
    char *dump = mag_dump_str(x);
    assert(dump);
    mag_set_ui(y, 7);
    int err = mag_load_str(y, dump);
    roundtrips++;
    check(err == 0 && mag_equal(y, x), "string-roundtrip", i);
    err = mag_load_str(y, dump);
    roundtrips++;
    check(err == 0 && mag_equal(y, x), "string-reload", i);
    flint_free(dump);
    preservation++;
    check(mag_equal(x, original), "conversion-input", i);
    printf("{\"kind\":\"magnitude-conversion\",\"id\":%lu,\"input\":", i);
    emit(x);
    printf(",\"double\":\"%016llx\",\"roundtrip\":", (unsigned long long)bits);
    emit(y);
    printf("}\n");
  }
  mpz_clears(got, want, (mpz_ptr)0);
  fmpz_clear(z);
  fmpq_clear(q);
  mag_clear(x);
  mag_clear(y);
  mag_clear(original);
}

int main(void) {
  assert(FLINT_BITS == 64 && MAG_BITS == 30 && sizeof(double) == 8 &&
         fegetround() == FE_TONEAREST);
  mpq_inits(target, decoded, scratch, (mpq_ptr)0);
  combinatorial();
  geometric();
  conversions();
  for (long prec = 512; prec <= 768; prec += 256) {
    mpfr_inits2(prec, low, high, val, xref, (mpfr_ptr)0);
    analytic_tails(prec);
    mpfr_clears(low, high, val, xref, (mpfr_ptr)0);
  }
  mpq_clears(target, decoded, scratch, (mpq_ptr)0);
  mpfr_free_cache();
  flint_cleanup_master();
  assert(fegetround() == FE_TONEAREST);
  printf("{\"kind\":\"summary\",\"exactChecks\":%lu,\"directedChecks\":%lu,"
         "\"inputChecks\":%lu,\"aliasChecks\":%lu,\"roundtrips\":%lu,"
         "\"infinitePolylogOutputs\":%lu,\"failures\":%lu}\n",
         exact_checks, directed_checks, preservation, aliases, roundtrips,
         infinite_polylog, failures);
  return failures ? 1 : 0;
}
