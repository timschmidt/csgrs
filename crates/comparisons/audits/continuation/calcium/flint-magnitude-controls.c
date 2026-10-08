/* Audit-only finite magnitude direction and ball error-inflation controls.
   Reuse the frozen exact rational decoder/rectangle oracle, not its old main.
   All fast inputs and outputs have bounded inline exponents. No raw invalid
   shapes, huge exponents, overlapping storage or safety-failure reproduction.
 */
#define main arb_dot_checkpoint38_main
#include "flint-arb-dot-controls.c"
#undef main

static const char *mag_names[] = {"add",
                                  "add-lower",
                                  "sub",
                                  "sub-lower",
                                  "mul",
                                  "mul-lower",
                                  "div",
                                  "div-lower",
                                  "fast-mul",
                                  "sqrt",
                                  "sqrt-lower",
                                  "rsqrt",
                                  "rsqrt-lower",
                                  "inv",
                                  "inv-lower",
                                  "pow-ui",
                                  "pow-ui-lower",
                                  "pow-fmpz",
                                  "pow-fmpz-lower",
                                  "addmul",
                                  "fast-addmul",
                                  "scale-si",
                                  "scale-fmpz",
                                  "fast-scale",
                                  "add-pow2",
                                  "fast-add-pow2",
                                  "min",
                                  "max"};
static const ulong mans[] = {0,          536870912,  536870913, 536870915,
                             570425343,  570425344,  805306368, 1073741821,
                             1073741822, 1073741823, 626349397, 894784853};
static const slong gaps[] = {-1025, -129, -65, -31, -30, -29, -1,  0,   1,
                             29,    30,   31,  63,  64,  65,  129, 1025};
static mpq_t mxq, myq, mcq, target, quality;
static uint64_t bound_calls, bound_failures, quality_failures,
    comparison_checks, comparison_failures;
static uint64_t mag_inputs, mag_input_failures, ball_results, ball_failures;

static int valid_mag(const mag_t x) {
  return !COEFF_IS_MPZ(MAG_EXP(x)) && mag_is_finite(x) &&
         (MAG_MAN(x) == 0 ? MAG_EXP(x) == 0
                          : MAG_MAN(x) >= 536870912 && MAG_MAN(x) < 1073741824);
}

static void mag_check(stats *s, const mag_t x, const mpq_t exact, int lower,
                      int square) {
  int ok = valid_mag(x) && decode_radius(qm, x);
  if (square)
    mpq_mul(qm, qm, qm);
  int c = mpq_cmp(qm, exact);
  ok = ok && (lower ? c <= 0 : c >= 0);
  /* Supplemental non-vacuity/quality control, not a best-rounding promise. */
  mpq_set_ui(quality, lower ? 1023 : 1025, 1024);
  mpq_mul(quality, quality, exact);
  int tight = lower ? mpq_cmp(qm, quality) >= 0 : mpq_cmp(qm, quality) <= 0;
  s->calls++;
  s->failed += !ok;
  s->exact += c == 0;
  bound_calls++;
  bound_failures += !ok;
  quality_failures += !tight;
  s->trace = mix(s->trace, MAG_MAN(x));
  s->trace = mix(s->trace, (uint64_t)MAG_EXP(x));
  if (!ok || !tight)
    fprintf(stderr, "magnitude %s mismatch at group call %" PRIu64 "\n",
            !ok ? "direction" : "quality", s->calls);
}

static void mag_input(const mag_t x, const mpq_t exact) {
  mag_inputs++;
  mag_input_failures +=
      !valid_mag(x) || !decode_radius(qm, x) || mpq_cmp(qm, exact) != 0;
}

static void qpow(mpq_t out, const mpq_t in, slong k) {
  mpz_pow_ui(mpq_numref(out), mpq_numref(in), (ulong)(k < 0 ? -k : k));
  mpz_pow_ui(mpq_denref(out), mpq_denref(in), (ulong)(k < 0 ? -k : k));
  if (k < 0)
    mpq_inv(out, out);
}

static void mag_emit(const char *kind, int a, int b, const char *op,
                     const stats *s) {
  printf("{\"kind\":\"%s\",\"a\":%d,\"b\":%d,\"op\":\"%s\",\"calls\":%" PRIu64
         ",\"failed\":%" PRIu64 ",\"exact\":%" PRIu64 ",\"trace\":\"%016" PRIx64
         "\"}\n",
         kind, a, b, op, s->calls, s->failed, s->exact, s->trace);
}

static void arithmetic_controls(void) {
  mag_t x, y, z;
  mag_init(x);
  mag_init(y);
  mag_init(z);
  fmpz_t e;
  fmpz_init(e);
  mpq_set_ui(mcq, 3, 8);
  for (int base = 0; base < 3; base++)
    for (int gi = 0; gi < 17; gi++) {
      stats s[28] = {{0}};
      for (int op = 0; op < 28; op++)
        s[op].trace = UINT64_C(14695981039346656037);
      slong xe = (base - 1) * 127, ye = xe + gaps[gi],
            shift = gaps[(gi + 5) % 17];
      const ulong powers[] = {0, 1, 2, 3, 7};
      const slong signed_powers[] = {-7, -1, 0, 1, 2, 3, 8};
      ulong power = powers[(gi + base) % 5];
      slong signed_power = signed_powers[(gi + base) % 7];
      for (int ai = 0; ai < 12; ai++)
        for (int bi = 0; bi < 12; bi++) {
          mag_set_ui_2exp_si(x, mans[ai], xe - 30);
          mag_set_ui_2exp_si(y, mans[bi], ye - 30);
          mpq_set_ui(mxq, mans[ai], 1);
          scale(mxq, xe - 30);
          mpq_set_ui(myq, mans[bi], 1);
          scale(myq, ye - 30);
          mag_input(x, mxq);
          mag_input(y, myq);
          comparison_checks++;
          int c = mag_cmp(x, y), qc = mpq_cmp(mxq, myq);
          comparison_failures += (c > 0) - (c < 0) != (qc > 0) - (qc < 0);
          mpq_set_ui(target, 1, 1);
          scale(target, shift);
          c = mag_cmp_2exp_si(x, shift);
          qc = mpq_cmp(mxq, target);
          comparison_checks++;
          comparison_failures += (c > 0) - (c < 0) != (qc > 0) - (qc < 0);
          for (int op = 0; op < 28; op++) {
            if ((op == 6 || op == 7) && !mans[bi])
              continue;
            if (((op >= 11 && op <= 14) ||
                 ((op == 17 || op == 18) && signed_power < 0)) &&
                !mans[ai])
              continue;
            int unary = (op >= 9 && op <= 18) || (op >= 21 && op <= 25);
            int lower = op == 1 || op == 3 || op == 5 || op == 7 || op == 10 ||
                        op == 12 || op == 14 || op == 16 || op == 18;
            for (int alias = 0; alias < (unary ? 2 : 3); alias++) {
              mag_set_ui_2exp_si(z, 3, -3);
              if (alias == 1)
                mag_set(z, x);
              if (alias == 2)
                mag_set(z, y);
              mag_srcptr xx = alias == 1 ? z : x, yy = alias == 2 ? z : y;
              if (op <= 1) {
                mpq_add(target, mxq, myq);
                if (op == 0)
                  mag_add(z, xx, yy);
                else
                  mag_add_lower(z, xx, yy);
              }
              if (op == 2 || op == 3) {
                mpq_sub(target, mxq, myq);
                if (mpq_sgn(target) < 0)
                  mpq_set_ui(target, 0, 1);
                if (op == 2)
                  mag_sub(z, xx, yy);
                else
                  mag_sub_lower(z, xx, yy);
              }
              if (op == 4 || op == 5 || op == 8) {
                mpq_mul(target, mxq, myq);
                if (op == 4)
                  mag_mul(z, xx, yy);
                else if (op == 5)
                  mag_mul_lower(z, xx, yy);
                else
                  mag_fast_mul(z, xx, yy);
              }
              if (op == 6 || op == 7) {
                mpq_div(target, mxq, myq);
                if (op == 6)
                  mag_div(z, xx, yy);
                else
                  mag_div_lower(z, xx, yy);
              }
              if (op == 9 || op == 10) {
                mpq_set(target, mxq);
                if (op == 9)
                  mag_sqrt(z, xx);
                else
                  mag_sqrt_lower(z, xx);
              }
              if (op >= 11 && op <= 14) {
                mpq_inv(target, mxq);
                if (op == 11)
                  mag_rsqrt(z, xx);
                if (op == 12)
                  mag_rsqrt_lower(z, xx);
                if (op == 13)
                  mag_inv(z, xx);
                if (op == 14)
                  mag_inv_lower(z, xx);
              }
              if (op == 15 || op == 16) {
                qpow(target, mxq, (slong)power);
                if (op == 15)
                  mag_pow_ui(z, xx, power);
                else
                  mag_pow_ui_lower(z, xx, power);
              }
              if (op == 17 || op == 18) {
                qpow(target, mxq, signed_power);
                fmpz_set_si(e, signed_power);
                if (op == 17)
                  mag_pow_fmpz(z, xx, e);
                else
                  mag_pow_fmpz_lower(z, xx, e);
              }
              if (op == 19 || op == 20) {
                mpq_mul(target, mxq, myq);
                mpq_add(target, target,
                        alias == 0   ? mcq
                        : alias == 1 ? mxq
                                     : myq);
                if (op == 19)
                  mag_addmul(z, xx, yy);
                else
                  mag_fast_addmul(z, xx, yy);
              }
              if (op >= 21 && op <= 23) {
                mpq_set(target, mxq);
                scale(target, shift);
                fmpz_set_si(e, shift);
                if (op == 21)
                  mag_mul_2exp_si(z, xx, shift);
                if (op == 22)
                  mag_mul_2exp_fmpz(z, xx, e);
                if (op == 23)
                  mag_fast_mul_2exp_si(z, xx, shift);
              }
              if (op == 24 || op == 25) {
                mpq_set_ui(target, 1, 1);
                scale(target, shift);
                mpq_add(target, target, mxq);
                fmpz_set_si(e, shift);
                if (op == 24)
                  mag_add_2exp_fmpz(z, xx, e);
                else
                  mag_fast_add_2exp_si(z, xx, shift);
              }
              if (op == 26 || op == 27) {
                int cmp = mpq_cmp(mxq, myq);
                mpq_set(target, (op == 26 ? cmp <= 0 : cmp >= 0) ? mxq : myq);
                if (op == 26)
                  mag_min(z, xx, yy);
                else
                  mag_max(z, xx, yy);
              }
              mag_check(s + op, z, target, lower, op >= 9 && op <= 12);
            }
          }
          mag_input(x, mxq);
          mag_input(y, myq);
        }
      for (int op = 0; op < 28; op++)
        mag_emit("arithmetic", base, gi, mag_names[op], s + op);
    }
  mag_clear(x);
  mag_clear(y);
  mag_clear(z);
  fmpz_clear(e);
}

static void conversion_controls(void) {
  const int bits[] = {0,   1,   29,  30,  31,  32,  63,  64, 65,
                      127, 128, 129, 191, 192, 193, 257, 513};
  const char *cn[] = {
      "fmpz",     "fmpz-lower",   "fmpz-scaled", "fmpz-scaled-lower",
      "arf",      "arf-lower",    "fast-arf",    "ui",
      "ui-lower", "ui-scaled",    "add-ui",      "add-ui-lower",
      "mul-ui",   "mul-ui-lower", "div-ui",      "add-ui-scaled"};
  mag_t z, x;
  mag_init(z);
  mag_init(x);
  mag_set_ui_2exp_si(x, 3, -3);
  arf_t a;
  arf_init(a);
  fmpz_t f, e;
  fmpz_init(f);
  fmpz_init(e);
  stats s[16] = {{0}};
  for (int op = 0; op < 16; op++)
    s[op].trace = UINT64_C(14695981039346656037);
  for (int j = 0; j < 17; j++)
    for (int pat = 0; pat < 4; pat++)
      for (int sign = 0; sign < 2; sign++)
        for (int sc = 0; sc < 5; sc++) {
          mpz_set_ui(ztmp, 0);
          if (bits[j]) {
            mpz_setbit(ztmp, (mp_bitcnt_t)bits[j] - 1);
            if (pat == 1)
              mpz_add_ui(ztmp, ztmp, 1);
            if (pat == 2) {
              mpz_mul_2exp(ztmp, ztmp, 1);
              mpz_sub_ui(ztmp, ztmp, 1);
            }
            if (pat == 3) {
              mpz_setbit(ztmp, (mp_bitcnt_t)bits[j] / 2);
              mpz_setbit(ztmp, 0);
            }
          }
          if (sign)
            mpz_neg(ztmp, ztmp);
          fmpz_set_mpz(f, ztmp);
          slong ex = (sc - 2) * 129;
          fmpz_set_si(e, ex);
          arf_set_mpz(a, ztmp);
          arf_mul_2exp_si(a, a, ex);
          mpq_set_z(mxq, ztmp);
          scale(mxq, ex);
          mag_inputs++;
          mag_input_failures += !decode_mid(qm, a) || mpq_cmp(qm, mxq) != 0;
          mpz_abs(ztmp, ztmp);
          mpq_set_z(myq, ztmp);
          mpq_abs(mxq, mxq);
          int word = mpz_fits_ulong_p(ztmp);
          ulong u = word ? mpz_get_ui(ztmp) : 0;
          for (int op = 0; op < 16; op++) {
            if (op >= 7 && !word)
              continue;
            if (op == 14 && u == 0)
              continue;
            mpq_set(target, (op == 2 || op == 3 || op == 4 || op == 5 ||
                             op == 6 || op == 9)
                                ? mxq
                                : myq);
            if (op == 0)
              mag_set_fmpz(z, f);
            if (op == 1)
              mag_set_fmpz_lower(z, f);
            if (op == 2)
              mag_set_fmpz_2exp_fmpz(z, f, e);
            if (op == 3)
              mag_set_fmpz_2exp_fmpz_lower(z, f, e);
            if (op == 4)
              arf_get_mag(z, a);
            if (op == 5)
              arf_get_mag_lower(z, a);
            if (op == 6)
              mag_fast_init_set_arf(z, a);
            if (op == 7)
              mag_set_ui(z, u);
            if (op == 8)
              mag_set_ui_lower(z, u);
            if (op == 9)
              mag_set_ui_2exp_si(z, u, ex);
            if (op == 10 || op == 11) {
              mpq_add(target, myq, mcq);
              if (op == 10)
                mag_add_ui(z, x, u);
              else
                mag_add_ui_lower(z, x, u);
            }
            if (op == 12 || op == 13) {
              mpq_mul(target, myq, mcq);
              if (op == 12)
                mag_mul_ui(z, x, u);
              else
                mag_mul_ui_lower(z, x, u);
            }
            if (op == 14) {
              mpq_div(target, mcq, myq);
              mag_div_ui(z, x, u);
            }
            if (op == 15) {
              mpq_add(target, mxq, mcq);
              mag_add_ui_2exp_si(z, x, u, ex);
            }
            mag_check(s + op, z, target,
                      op == 1 || op == 3 || op == 5 || op == 8 || op == 11 ||
                          op == 13,
                      0);
          }
          fmpz_get_mpz(ztmp, f);
          mpq_set_z(target, ztmp);
          mpq_abs(target, target);
          mag_inputs++;
          mag_input_failures += mpq_cmp(target, myq) != 0;
          mag_input(x, mcq);
        }
  for (int op = 0; op < 16; op++)
    mag_emit("conversion", 0, 0, cn[op], s + op);
  mag_clear(x);
  mag_clear(z);
  arf_clear(a);
  fmpz_clear(f);
  fmpz_clear(e);
}

static void ball_controls(void) {
  const int ws[] = {1, 2, 3, 25, 26, 331};
  const char *bn[] = {
      "fma",           "fma-x",          "fma-y",      "fma-initial",
      "addmul",        "error-arf",      "error-ball", "error-mag",
      "error-pow2-si", "error-pow2-fmpz"};
  arb_t x, y, c, z, err;
  arb_init(x);
  arb_init(y);
  arb_init(c);
  arb_init(z);
  arb_init(err);
  ball a, b, initial;
  mpq_inits(a.m, a.r, b.m, b.r, initial.m, initial.r, NULL);
  mpq_t lower, upper;
  mpq_inits(lower, upper, NULL);
  fmpz_t e;
  fmpz_init(e);
  for (int wi = 0; wi < 6; wi++)
    for (int family = 0; family < 6; family++) {
      stats s[10] = {{0}};
      for (int op = 0; op < 10; op++)
        s[op].trace = UINT64_C(14695981039346656037);
      for (int signs = 0; signs < 4; signs++) {
        make_ball(x, &a, ws[wi], family, 0, 0);
        make_ball(y, &b, ws[wi], family, 1, 1);
        make_ball(c, &initial, ws[wi], family, 2, 0);
        if (signs & 1) {
          arb_neg(x, x);
          mpq_neg(a.m, a.m);
        }
        if (signs & 2) {
          arb_neg(y, y);
          mpq_neg(b.m, b.m);
        }
        oracle(lower, upper, &a, 0, 1, &b, 0, 1, 1, &initial, 0);
        const slong ps[] = {2, 30, 31, 63, 64, 65, 129, ws[wi] * 128 + 1};
        for (int p = 0; p < 8; p++)
          for (int op = 0; op < 5; op++) {
            arb_set(z, op == 1 ? x : op == 2 ? y : c);
            if (op == 0)
              arb_fma(z, x, y, c, ps[p]);
            if (op == 1)
              arb_fma(z, z, y, c, ps[p]);
            if (op == 2)
              arb_fma(z, x, z, c, ps[p]);
            if (op == 3)
              arb_fma(z, x, y, z, ps[p]);
            if (op == 4)
              arb_addmul(z, x, y, ps[p]);
            check(s + op, z, lower, upper);
          }
        arb_set(err, y);
        if (signs == 0)
          arb_zero(err);
        for (int op = 5; op < 10; op++) {
          arb_set(z, x);
          mpq_set(target, b.m);
          mpq_abs(target, target);
          if (op == 6)
            mpq_add(target, target, b.r);
          if (op == 7)
            mpq_set(target, b.r);
          if (signs == 0)
            mpq_set_ui(target, 0, 1);
          slong exponent = (family - 3) * 65;
          fmpz_set_si(e, exponent);
          if (op >= 8) {
            mpq_set_ui(target, 1, 1);
            scale(target, exponent);
          }
          endpoints(lower, upper, &a);
          mpq_sub(lower, lower, target);
          mpq_add(upper, upper, target);
          if (op == 5)
            arb_add_error_arf(z, arb_midref(err));
          if (op == 6)
            arb_add_error(z, err);
          if (op == 7)
            arb_add_error_mag(z, arb_radref(err));
          if (op == 8)
            arb_add_error_2exp_si(z, exponent);
          if (op == 9)
            arb_add_error_2exp_fmpz(z, e);
          check(s + op, z, lower, upper);
          /* Inflation changes radius only. */
          inputs++;
          input_failures +=
              !decode_mid(qm, arb_midref(z)) || mpq_cmp(qm, a.m) != 0;
        }
        input_check(x, &a);
        input_check(y, &b);
        input_check(c, &initial);
      }
      for (int op = 0; op < 10; op++) {
        mag_emit("ball", wi, family, bn[op], s + op);
        ball_results += s[op].calls;
        ball_failures += s[op].failed;
      }
    }
  arb_clear(x);
  arb_clear(y);
  arb_clear(c);
  arb_clear(z);
  arb_clear(err);
  fmpz_clear(e);
  mpq_clears(a.m, a.r, b.m, b.r, initial.m, initial.r, lower, upper, NULL);
}

int main(void) {
  if (FLINT_BITS != 64 || sizeof(ulong) != 8 || GMP_NUMB_BITS != 64)
    return 2;
  mpz_init(ztmp);
  mpq_inits(qm, qr, lo, hi, tmp, xl, xh, yl, yh, mxq, myq, mcq, target, quality,
            NULL);
  for (int i = 0; i < 4; i++)
    mpq_init(prod[i]);
  arithmetic_controls();
  conversion_controls();
  ball_controls();
  mpz_clear(ztmp);
  mpq_clears(qm, qr, lo, hi, tmp, xl, xh, yl, yh, mxq, myq, mcq, target,
             quality, NULL);
  for (int i = 0; i < 4; i++)
    mpq_clear(prod[i]);
  flint_cleanup_master();
  printf("{\"kind\":\"summary\",\"boundChecks\":%" PRIu64
         ",\"boundFailures\":%" PRIu64 ",\"qualityFailures\":%" PRIu64
         ",\"comparisonChecks\":%" PRIu64 ",\"comparisonFailures\":%" PRIu64
         ",\"magnitudeInputChecks\":%" PRIu64
         ",\"magnitudeInputFailures\":%" PRIu64 ",\"ballResults\":%" PRIu64
         ",\"ballFailures\":%" PRIu64 ",\"ballInputChecks\":%" PRIu64
         ",\"ballInputFailures\":%" PRIu64 "}\n",
         bound_calls, bound_failures, quality_failures, comparison_checks,
         comparison_failures, mag_inputs, mag_input_failures, ball_results,
         ball_failures, inputs, input_failures);
  return bound_failures || quality_failures || comparison_failures ||
                 mag_input_failures || ball_failures || input_failures
             ? 1
             : 0;
}
