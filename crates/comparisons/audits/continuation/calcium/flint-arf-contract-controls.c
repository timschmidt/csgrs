/* Checkpoint 44: bounded finite scalar contracts, exact GMP power oracle.
 * No MPFR oracle, raw-size/overlap probes, invalid precisions, special values,
 * huge exponents, parser failures or previous crash reproduction. */
#include "arf.h"
#include <assert.h>
#include <fenv.h>
#include <gmp.h>
#include <stdio.h>

static unsigned long checks, aliases, preserved, failures, ties, exacts;
static const slong precisions[] = {1,  2,  3,  7,   31,  32,  33,
                                   63, 64, 65, 127, 128, 129, 257};
static const arf_rnd_t modes[] = {ARF_RND_DOWN, ARF_RND_UP, ARF_RND_FLOOR,
                                  ARF_RND_CEIL, ARF_RND_NEAR};
static mpz_t a, b, t, floor_q, g, power, midpoint, expected, actual;

static void check(int ok, const char *what) {
  if (!ok) {
    failures++;
    fprintf(stderr, "FAIL %s at check %lu\n", what, checks);
  }
}
static void canonical(mpz_t x, slong *e) {
  if (!mpz_sgn(x)) {
    *e = 0;
    return;
  }
  mp_bitcnt_t v = mpz_scan1(x, 0);
  mpz_tdiv_q_2exp(x, x, v);
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
  nn_srcptr limbs;
  slong n;
  ARF_GET_MPN_READONLY(limbs, n, x);
  mpz_import(m, (size_t)n, -1, sizeof(ulong), 0, 0, limbs);
  if (ARF_SGNBIT(x))
    mpz_neg(m, m);
  *e = ARF_EXP(x) - n * FLINT_BITS;
  assert(*e > -10000 && *e < 10000);
  canonical(m, e);
}
static void set_q(mpq_t q, const mpz_t m, slong e) {
  mpq_set_z(q, m);
  if (e >= 0)
    mpq_mul_2exp(q, q, (mp_bitcnt_t)e);
  else
    mpq_div_2exp(q, q, (mp_bitcnt_t)-e);
}
static void input_check(const arf_t x, const mpq_t q) {
  slong e;
  mpq_t decoded;
  mpq_init(decoded);
  decode(actual, &e, x);
  set_q(decoded, actual, e);
  preserved++;
  check(mpq_equal(decoded, q), "input preservation");
  mpq_clear(decoded);
}
/* Round sign(q)*abs(q)^(1/k), using integer quotient/root and exact powers.
 * All production root inputs here are nonnegative; k=1 covers signed
 * arithmetic. */
static int oracle(const mpq_t q, ulong k, slong p, arf_rnd_t mode,
                  slong *out_e) {
  const int negative = mpq_sgn(q) < 0;
  if (!mpq_sgn(q)) {
    mpz_set_ui(expected, 0);
    *out_e = 0;
    return 0;
  }
  mpz_abs(a, mpq_numref(q));
  mpz_set(b, mpq_denref(q));
  slong e = (slong)mpz_sizeinbase(a, 2) - (slong)mpz_sizeinbase(b, 2);
  int cmp;
  if (e >= 0) {
    mpz_mul_2exp(t, b, (mp_bitcnt_t)e);
    cmp = mpz_cmp(a, t);
  } else {
    mpz_mul_2exp(t, a, (mp_bitcnt_t)-e);
    cmp = mpz_cmp(t, b);
  }
  if (cmp < 0)
    e--;
  const slong root_e =
      e >= 0 ? e / (slong)k : -((-e + (slong)k - 1) / (slong)k);
  *out_e = root_e + 1 - p;
  const slong scale = -*out_e * (slong)k;
  if (scale >= 0)
    mpz_mul_2exp(a, a, (mp_bitcnt_t)scale);
  else
    mpz_mul_2exp(b, b, (mp_bitcnt_t)-scale);
  mpz_fdiv_q(floor_q, a, b);
  mpz_root(g, floor_q, k);
  mpz_pow_ui(power, g, k);
  mpz_mul(power, power, b);
  const int inexact = mpz_cmp(power, a) != 0;
  int up = mode == ARF_RND_UP || (mode == ARF_RND_FLOOR && negative) ||
           (mode == ARF_RND_CEIL && !negative);
  if (mode == ARF_RND_NEAR) {
    mpz_mul_2exp(midpoint, g, 1);
    mpz_add_ui(midpoint, midpoint, 1);
    mpz_pow_ui(midpoint, midpoint, k);
    mpz_mul(midpoint, midpoint, b);
    mpz_mul_2exp(t, a, k);
    cmp = mpz_cmp(t, midpoint);
    ties += cmp == 0;
    up = cmp > 0 || (cmp == 0 && mpz_odd_p(g));
  }
  if (inexact && up)
    mpz_add_ui(g, g, 1);
  mpz_set(expected, g);
  if (negative)
    mpz_neg(expected, expected);
  canonical(expected, out_e);
  return inexact;
}
static void emit_result(const arf_t result, int ret, const mpq_t q, ulong k,
                        slong p, arf_rnd_t mode) {
  slong expected_e, actual_e;
  const int inexact = oracle(q, k, p, mode, &expected_e);
  decode(actual, &actual_e, result);
  checks++;
  exacts += !inexact;
  check(ret == inexact && actual_e == expected_e &&
            mpz_cmp(actual, expected) == 0,
        "rounded value/exactness");
  gmp_printf("[\"%Zx\",%ld,%d]", actual, (long)actual_e, ret);
}
static void pattern(mpz_t z, unsigned bits, unsigned variant) {
  mpz_set_ui(z, 0);
  mpz_setbit(z, bits - 1);
  if (variant == 1) {
    mpz_mul_2exp(z, z, 1);
    mpz_sub_ui(z, z, 1);
  }
  if (variant == 2) {
    mpz_setbit(z, bits / 2);
    mpz_setbit(z, 0);
  }
}
static void scalar_case(unsigned id, const mpz_t xm, slong xe, const mpz_t ym,
                        slong ye) {
  arf_t x, y, z, alias;
  arf_init(x);
  arf_init(y);
  arf_init(z);
  arf_init(alias);
  fmpz_t import;
  fmpz_init(import);
  fmpz_set_mpz(import, xm);
  arf_set_fmpz(x, import);
  arf_mul_2exp_si(x, x, xe);
  fmpz_set_mpz(import, ym);
  arf_set_fmpz(y, import);
  arf_mul_2exp_si(y, y, ye);
  mpq_t xq, yq, target;
  mpq_inits(xq, yq, target, NULL);
  set_q(xq, xm, xe);
  set_q(yq, ym, ye);
  assert(mpq_sgn(yq) != 0);
  input_check(x, xq);
  input_check(y, yq);
  for (unsigned op = 0; op < 3; op++) {
    if (op == 0)
      mpq_add(target, xq, yq);
    else if (op == 1)
      mpq_sub(target, xq, yq);
    else
      mpq_div(target, xq, yq);
    int (*fn)(arf_ptr, arf_srcptr, arf_srcptr, slong, arf_rnd_t) =
        op == 0   ? arf_add
        : op == 1 ? arf_sub
                  : arf_div;
    for (unsigned pi = 0; pi < sizeof(precisions) / sizeof(*precisions); pi++) {
      const slong p = precisions[pi];
      gmp_printf("{\"kind\":\"arithmetic\",\"id\":%u,\"op\":%u,\"q\":[\"%Zx\","
                 "\"%Zx\"],\"k\":1,\"p\":%ld,\"results\":[",
                 id, op, mpq_numref(target), mpq_denref(target), (long)p);
      for (unsigned mi = 0; mi < 5; mi++) {
        if (mi)
          printf(",");
        printf("[");
        arf_set_si(z, -7);
        int ret = fn(z, x, y, p, modes[mi]);
        emit_result(z, ret, target, 1, p, modes[mi]);
        printf(",");
        arf_set(alias, x);
        int ar = fn(alias, alias, y, p, modes[mi]);
        aliases++;
        check(ret == ar && arf_equal(alias, z), "left alias");
        emit_result(alias, ar, target, 1, p, modes[mi]);
        printf(",");
        arf_set(alias, y);
        ar = fn(alias, x, alias, p, modes[mi]);
        aliases++;
        check(ret == ar && arf_equal(alias, z), "right alias");
        emit_result(alias, ar, target, 1, p, modes[mi]);
        printf("]");
      }
      printf("]}\n");
      input_check(x, xq);
      input_check(y, yq);
    }
  }
  arf_clear(x);
  arf_clear(y);
  arf_clear(z);
  arf_clear(alias);
  fmpz_clear(import);
  mpq_clears(xq, yq, target, NULL);
}
static void root_case(unsigned id, const mpz_t xm, slong xe) {
  arf_t x, z, alias;
  arf_init(x);
  arf_init(z);
  arf_init(alias);
  fmpz_t import;
  fmpz_init(import);
  fmpz_set_mpz(import, xm);
  arf_set_fmpz(x, import);
  arf_mul_2exp_si(x, x, xe);
  mpq_t xq, target;
  mpq_inits(xq, target, NULL);
  set_q(xq, xm, xe);
  assert(mpq_sgn(xq) > 0);
  input_check(x, xq);
  const ulong degrees[] = {1, 2, 3, 5, 7, 17, 2};
  for (unsigned op = 0; op < 7; op++) {
    const ulong k = degrees[op];
    if (op == 6)
      mpq_inv(target, xq);
    else
      mpq_set(target, xq);
    for (unsigned pi = 0; pi < sizeof(precisions) / sizeof(*precisions); pi++) {
      const slong p = precisions[pi];
      gmp_printf("{\"kind\":\"root\",\"id\":%u,\"op\":%u,\"q\":[\"%Zx\",\"%"
                 "Zx\"],\"k\":%lu,\"p\":%ld,\"results\":[",
                 id, op, mpq_numref(target), mpq_denref(target), k, (long)p);
      for (unsigned mi = 0; mi < 5; mi++) {
        if (mi)
          printf(",");
        printf("[");
        arf_set_ui(z, 9);
        int ret = op == 6 ? arf_rsqrt(z, x, p, modes[mi])
                          : arf_root(z, x, k, p, modes[mi]);
        emit_result(z, ret, target, k, p, modes[mi]);
        printf(",");
        arf_set(alias, x);
        int ar = op == 6 ? arf_rsqrt(alias, alias, p, modes[mi])
                         : arf_root(alias, alias, k, p, modes[mi]);
        aliases++;
        check(ret == ar && arf_equal(alias, z), "root alias");
        emit_result(alias, ar, target, k, p, modes[mi]);
        printf("]");
      }
      printf("]}\n");
      input_check(x, xq);
    }
  }
  arf_clear(x);
  arf_clear(z);
  arf_clear(alias);
  fmpz_clear(import);
  mpq_clears(xq, target, NULL);
}
int main(void) {
  assert(FLINT_BITS == 64 && GMP_NUMB_BITS == 64 &&
         fegetround() == FE_TONEAREST);
  mpz_inits(a, b, t, floor_q, g, power, midpoint, expected, actual, NULL);
  mpz_t xm, ym, sx, sy;
  mpz_inits(xm, ym, sx, sy, NULL);
  const unsigned widths[] = {1,  2,   3,   31,  32,  33,  63, 64,
                             65, 127, 128, 129, 255, 256, 257};
  const slong exponents[] = {-257, -1, 65};
  unsigned root_id = 0, arithmetic_id = 0;
  for (unsigned wi = 0; wi < 15; wi++)
    for (unsigned variant = 0; variant < 3; variant++)
      for (unsigned ei = 0; ei < 3; ei++) {
        pattern(xm, widths[wi], variant);
        pattern(ym, widths[(wi + 7) % 15], (variant + 1) % 3);
        root_case(root_id++, xm, exponents[ei]);
        for (unsigned sign = 0; sign < 4; sign++) {
          mpz_set(sx, xm);
          mpz_set(sy, ym);
          if (sign & 1)
            mpz_neg(sx, sx);
          if (sign & 2)
            mpz_neg(sy, sy);
          scalar_case(arithmetic_id++, sx, exponents[ei], sy,
                      exponents[(ei + 1) % 3]);
        }
      }
  /* Exact nearest-even root midpoints and one input quantum on either side.
   * Values are (odd^k + delta) / 2^k, not floating approximations. */
  const ulong tie_degrees[] = {2, 3, 5, 7, 17};
  for (unsigned ki = 0; ki < 5; ki++)
    for (ulong odd = 3; odd <= 7; odd += 2)
      for (int delta = -1; delta <= 1; delta++) {
        mpz_ui_pow_ui(xm, odd, tie_degrees[ki]);
        if (delta < 0)
          mpz_sub_ui(xm, xm, 1);
        else if (delta > 0)
          mpz_add_ui(xm, xm, 1);
        root_case(root_id++, xm, -(slong)tie_degrees[ki]);
      }
  mpz_set_ui(xm, 0);
  mpz_set_ui(ym, 3);
  scalar_case(arithmetic_id++, xm, 0, ym, -1);
  mpz_set_ui(xm, 3);
  scalar_case(arithmetic_id++, xm, -1, ym, -1);
  mpz_neg(ym, ym);
  scalar_case(arithmetic_id++, xm, -1, ym, -1);
  printf("{\"summary\":true,\"rootFixtures\":%u,\"arithmeticFixtures\":%u,"
         "\"checks\":%lu,\"aliases\":%lu,\"inputChecks\":%lu,\"exact\":%lu,"
         "\"ties\":%lu,\"failures\":%lu}\n",
         root_id, arithmetic_id, checks, aliases, preserved, exacts, ties,
         failures);
  mpz_clears(xm, ym, sx, sy, a, b, t, floor_q, g, power, midpoint, expected,
             actual, NULL);
  flint_cleanup_master();
  return failures ? 1 : 0;
}
