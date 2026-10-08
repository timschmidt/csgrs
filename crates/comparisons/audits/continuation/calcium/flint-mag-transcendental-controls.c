/* Checkpoint 40: bounded, valid public numerical inputs only.
 * MPFR receives the exact 30-bit dyadic directly, never through ARF or a
 * potentially enormous integer expansion. The infinite exp-tail reference
 * includes an independently bounded remainder, not just a finite lower sum.
 * This is an audit harness, not a donor implementation or performance driver. */
#include <assert.h>
#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <mpfr.h>
#include "mag.h"

typedef void (*mag_fn)(mag_t, const mag_t);
typedef struct {
    const char *name;
    mag_fn fn;
    int target, lower, exp_limited;
} operation;
enum { EXP, EXPM1, EXPINV, SINH, COSH, ATAN, LOG, NEGLOG, LOG1P };
static const operation ops[] = {
    {"exp", mag_exp, EXP, 0, 1},
    {"exp_lower", mag_exp_lower, EXP, 1, 1},
    {"expm1", mag_expm1, EXPM1, 0, 1},
    {"expinv", mag_expinv, EXPINV, 0, 1},
    {"expinv_lower", mag_expinv_lower, EXPINV, 1, 1},
    {"sinh", mag_sinh, SINH, 0, 1},
    {"sinh_lower", mag_sinh_lower, SINH, 1, 1},
    {"cosh", mag_cosh, COSH, 0, 1},
    {"cosh_lower", mag_cosh_lower, COSH, 1, 1},
    {"atan", mag_atan, ATAN, 0, 0},
    {"atan_lower", mag_atan_lower, ATAN, 1, 0},
    {"log", mag_log, LOG, 0, 0},
    {"log_lower", mag_log_lower, LOG, 1, 0},
    {"neg_log", mag_neg_log, NEGLOG, 0, 0},
    {"neg_log_lower", mag_neg_log_lower, NEGLOG, 1, 0},
    {"log1p", mag_log1p, LOG1P, 0, 0}
};
static const long exponents[] = {
    -1075,-1001,-1000,-971,-970,-969,-61,-60,-31,-30,-29,
    -17,-16,-15,-14,-11,-10,-9,-1,0,1,3,4,5,6,10,
    23,24,25,26,29,30,31,999,1000,1001,1024
};
static const unsigned long mantissas[] = {
    536870912UL,536870913UL,536870914UL,536870915UL,
    671088639UL,671088640UL,671088641UL,
    1073741821UL,1073741822UL,1073741823UL
};
static const unsigned long roots[] = {1,2,3,4,5,7,16,31};
static const unsigned long tails[] = {0,1,2,3,8,31,64,128,255,256,257};
static unsigned long checks, quality_checks, input_checks, alias_checks, failures;
static unsigned long finite_results, infinite_results;
static mpfr_t lo, hi, val, quality, xx;

static void require(int ok, const char *what, unsigned long id, const char *op) {
    if (!ok) {
        failures++;
        fprintf(stderr, "FAIL %s fixture=%lu operation=%s\n", what, id, op);
    }
}

static void exact_import(mpfr_t y, const mag_t x) {
    if (mag_is_zero(x)) {
        mpfr_set_zero(y, 1);
    } else if (mag_is_inf(x)) {
        mpfr_set_inf(y, 1);
    } else {
        assert(!COEFF_IS_MPZ(MAG_EXP(x)));
        assert(MAG_MAN(x) >= MAG_ONE_HALF && MAG_MAN(x) < (1UL << MAG_BITS));
        assert(MAG_EXP(x) >= -200000000L && MAG_EXP(x) <= 200000000L);
        assert(mpfr_set_ui(y, MAG_MAN(x), MPFR_RNDN) == 0);
        assert(mpfr_mul_2si(y, y, MAG_EXP(x) - MAG_BITS, MPFR_RNDN) == 0);
    }
}

static void reference(mpfr_t y, const mpfr_t x, int target, mpfr_rnd_t rnd) {
    switch (target) {
    case EXP: mpfr_exp(y, x, rnd); break;
    case EXPM1: mpfr_expm1(y, x, rnd); break;
    case EXPINV: mpfr_neg(y, x, MPFR_RNDN); mpfr_exp(y, y, rnd); break;
    case SINH: mpfr_sinh(y, x, rnd); break;
    case COSH: mpfr_cosh(y, x, rnd); break;
    case ATAN: mpfr_atan(y, x, rnd); break;
    case LOG:
        if (mpfr_cmp_ui(x, 1) <= 0) mpfr_set_zero(y, 1);
        else mpfr_log(y, x, rnd);
        break;
    case NEGLOG:
        if (mpfr_cmp_ui(x, 1) >= 0) mpfr_set_zero(y, 1);
        else {
            mpfr_log(y, x, rnd == MPFR_RNDD ? MPFR_RNDU : MPFR_RNDD);
            mpfr_neg(y, y, MPFR_RNDN);
        }
        break;
    case LOG1P: mpfr_log1p(y, x, rnd); break;
    default: assert(0);
    }
    assert(!mpfr_nan_p(y));
}

/* Write every returned bound, including the supported in-place route. */
static void check_result(const mag_t y, int lower, int quality_enabled,
                         unsigned long id, const char *name) {
    exact_import(val, y);
    checks++;
    require(lower ? mpfr_cmp(val, lo) <= 0 : mpfr_cmp(val, hi) >= 0,
            "bound", id, name);
    if (quality_enabled) {
        quality_checks++;
        /* Strong comparisons: the opposite endpoint bounds the tolerance. */
        mpfr_mul_ui(quality, lower ? hi : lo, lower ? 1023 : 1025,
                    lower ? MPFR_RNDU : MPFR_RNDD);
        mpfr_div_2ui(quality, quality, 10, lower ? MPFR_RNDU : MPFR_RNDD);
        require(lower ? mpfr_cmp(val, quality) >= 0 : mpfr_cmp(val, quality) <= 0,
                "relative-quality", id, name);
    }
    if (mag_is_inf(y)) infinite_results++;
    else finite_results++;
    printf("[%lu,%ld,%d]", MAG_MAN(y), (long)MAG_EXP(y), mag_is_inf(y));
}

static void unary(unsigned long id, unsigned long man, long exp, long prec) {
    mag_t x, y, z, original;
    mag_init(x); mag_init(y); mag_init(z); mag_init(original);
    if (man) { MAG_MAN(x) = man; fmpz_set_si(MAG_EXPREF(x), exp); }
    mag_set(original, x);
    exact_import(xx, x);
    printf("{\"kind\":\"unary\",\"precision\":%ld,\"id\":%lu,\"man\":%lu,\"exp\":%ld,\"results\":[",
           prec, id, man, exp);
    int first = 1;
    for (unsigned long j = 0; j < sizeof(ops)/sizeof(*ops); j++) {
        const operation *op = ops+j;
        if (op->exp_limited && exp > 26) continue;
        if (op->target == NEGLOG && !man) continue;
        reference(lo, xx, op->target, MPFR_RNDD);
        reference(hi, xx, op->target, MPFR_RNDU);
        assert(mpfr_cmp(lo, hi) <= 0 && mpfr_number_p(lo) && mpfr_number_p(hi));
        mag_set_ui(y, 7); /* a distinct, finite initial destination */
        op->fn(y, x);
        input_checks++;
        require(mag_equal(x, original), "input-preservation", id, op->name);
        mag_set(z, x);
        op->fn(z, z);
        alias_checks++;
        require(mag_equal(y, z), "whole-alias", id, op->name);
        if (!first) printf(",");
        first = 0;
        printf("[%lu,", j);
        int q = !op->exp_limited || exp <= 10;
        check_result(y, op->lower, q, id, op->name);
        printf(",");
        check_result(z, op->lower, q, id, op->name);
        printf("]");
    }
    for (unsigned long j = 0; j < sizeof(roots)/sizeof(*roots); j++) {
        unsigned long n = roots[j];
        mpfr_rootn_ui(lo, xx, n, MPFR_RNDD);
        mpfr_rootn_ui(hi, xx, n, MPFR_RNDU);
        mag_set_ui(y, 7);
        mag_root(y, x, n);
        input_checks++;
        require(mag_equal(x, original), "input-preservation", id, "root");
        mag_set(z, x); mag_root(z, z, n);
        alias_checks++;
        require(mag_equal(y, z), "whole-alias", id, "root");
        if (!first) printf(",");
        first = 0;
        printf("[%lu,", 16+j);
        check_result(y, 0, 1, id, "root");
        printf(",");
        check_result(z, 0, 1, id, "root");
        printf("]");
    }
    printf("]}\n");
    mag_clear(x); mag_clear(y); mag_clear(z); mag_clear(original);
}

/* For t_k=x^k/k!, sum t_N..t_K using directed arithmetic. All remaining
 * ratios are <= q=x/(K+2), so remainder <= t_(K+1)/(1-q).
 * x in [0,4], K=N+256: positive denominator, no cancellation of exp prefixes. */
static void tail_reference(const mpfr_t x, unsigned long n, long prec) {
    mpfr_t tl, th, q, den;
    mpfr_inits2(prec, tl, th, q, den, (mpfr_ptr)0);
    mpfr_set_ui(tl, 1, MPFR_RNDN); mpfr_set_ui(th, 1, MPFR_RNDN);
    for (unsigned long k = 1; k <= n; k++) {
        mpfr_mul(tl, tl, x, MPFR_RNDD); mpfr_div_ui(tl, tl, k, MPFR_RNDD);
        mpfr_mul(th, th, x, MPFR_RNDU); mpfr_div_ui(th, th, k, MPFR_RNDU);
    }
    mpfr_set(lo, tl, MPFR_RNDN); mpfr_set(hi, th, MPFR_RNDN);
    const unsigned long end = n+256;
    for (unsigned long k = n+1; k <= end; k++) {
        mpfr_mul(tl, tl, x, MPFR_RNDD); mpfr_div_ui(tl, tl, k, MPFR_RNDD);
        mpfr_mul(th, th, x, MPFR_RNDU); mpfr_div_ui(th, th, k, MPFR_RNDU);
        mpfr_add(lo, lo, tl, MPFR_RNDD); mpfr_add(hi, hi, th, MPFR_RNDU);
    }
    mpfr_mul(th, th, x, MPFR_RNDU);
    mpfr_div_ui(th, th, end+1, MPFR_RNDU);
    mpfr_div_ui(q, x, end+2, MPFR_RNDU);
    assert(mpfr_cmp_ui(q, 1) < 0);
    mpfr_ui_sub(den, 1, q, MPFR_RNDD);
    mpfr_div(th, th, den, MPFR_RNDU);
    mpfr_add(hi, hi, th, MPFR_RNDU);
    assert(mpfr_cmp(lo, hi) <= 0);
    mpfr_clears(tl, th, q, den, (mpfr_ptr)0);
}

static void tail_tests(long prec) {
    const unsigned long mans[] = {0,536870912UL,536870912UL,536870912UL,
        1073741823UL,536870912UL,536870913UL,536870912UL,536870912UL};
    const long exps[] = {0,-59,-3,0,0,1,1,2,3};
    mag_t x, y, z, original;
    mag_init(x); mag_init(y); mag_init(z); mag_init(original);
    for (unsigned long i = 0; i < sizeof(mans)/sizeof(*mans); i++) {
        mag_zero(x);
        if (mans[i]) { MAG_MAN(x) = mans[i]; fmpz_set_si(MAG_EXPREF(x), exps[i]); }
        mag_set(original, x); exact_import(xx, x);
        for (unsigned long j = 0; j < sizeof(tails)/sizeof(*tails); j++) {
            unsigned long n = tails[j], id = i*11+j;
            tail_reference(xx, n, prec);
            mag_set_ui(y, 7); mag_exp_tail(y, x, n);
            input_checks++;
            require(mag_equal(x, original), "input-preservation", id, "exp_tail");
            mag_set(z, x); mag_exp_tail(z, z, n);
            alias_checks++;
            require(mag_equal(y, z), "whole-alias", id, "exp_tail");
            printf("{\"kind\":\"tail\",\"precision\":%ld,\"id\":%lu,\"man\":%lu,\"exp\":%ld,\"n\":%lu,\"results\":[",
                prec,id,mans[i],exps[i],n);
            check_result(y, 0, 0, id, "exp_tail"); printf(",");
            check_result(z, 0, 0, id, "exp_tail"); printf("]}\n");
        }
    }
    mag_clear(x); mag_clear(y); mag_clear(z); mag_clear(original);
}

static void double_log(unsigned long id, double x, long prec) {
    assert(isfinite(x) && x > 0);
    assert(mpfr_set_d(xx, x, MPFR_RNDN) == 0);
    mpfr_log(lo, xx, MPFR_RNDD); mpfr_log(hi, xx, MPFR_RNDU);
    double a = mag_d_log_lower_bound(x), b = mag_d_log_upper_bound(x);
    mpfr_set_d(val, a, MPFR_RNDN); checks++;
    require(mpfr_cmp(val, lo) <= 0, "double-lower", id, "d_log");
    mpfr_set_d(val, b, MPFR_RNDN); checks++;
    require(mpfr_cmp(val, hi) >= 0, "double-upper", id, "d_log");
    uint64_t ux, ua, ub;
    memcpy(&ux, &x, sizeof ux); memcpy(&ua, &a, sizeof ua); memcpy(&ub, &b, sizeof ub);
    printf("{\"kind\":\"dlog\",\"precision\":%ld,\"id\":%lu,\"x\":\"%016llx\",\"lower\":\"%016llx\",\"upper\":\"%016llx\"}\n",
        prec,id,(unsigned long long)ux,(unsigned long long)ua,(unsigned long long)ub);
}

int main(void) {
    assert(FLINT_BITS == 64 && MAG_BITS == 30 && sizeof(double) == 8);
    assert(fegetround() == FE_TONEAREST);
    assert(mpfr_get_emin() < -200000001L && mpfr_get_emax() > 200000001L);
    const long precisions[] = {512,768};
    for (unsigned long p = 0; p < 2; p++) {
        long prec = precisions[p];
        mpfr_inits2(prec, lo, hi, val, quality, xx, (mpfr_ptr)0);
        unary(0,0,0,prec);
        unsigned long id = 1;
        for (unsigned long e = 0; e < sizeof(exponents)/sizeof(*exponents); e++)
            for (unsigned long m = 0; m < sizeof(mantissas)/sizeof(*mantissas); m++)
                unary(id++,mantissas[m],exponents[e],prec);
        tail_tests(prec);
        id = 0;
        const int scales[] = {-1000,-31,-1,0,1,31,1000};
        for (unsigned long s = 0; s < sizeof(scales)/sizeof(*scales); s++)
            for (int k = 16; k <= 47; k++) {
                double x = ldexp(k/32.0,scales[s]);
                double_log(id++,nextafter(x,0),prec);
                double_log(id++,x,prec);
                double_log(id++,nextafter(x,INFINITY),prec);
            }
        const double more[] = {DBL_TRUE_MIN,nextafter(DBL_MIN,0),DBL_MIN,
            nextafter(DBL_MIN,INFINITY),DBL_MAX,nextafter(1.0,0),1.0,nextafter(1.0,INFINITY)};
        for (unsigned long j=0; j<sizeof(more)/sizeof(*more); j++) double_log(id++,more[j],prec);
        mag_t pi;
        mag_init(pi);
        mpfr_const_pi(lo,MPFR_RNDD); mpfr_const_pi(hi,MPFR_RNDU);
        printf("{\"kind\":\"pi\",\"precision\":%ld,\"results\":[",prec);
        mag_const_pi_lower(pi); check_result(pi,1,1,0,"pi_lower"); printf(",");
        mag_const_pi(pi); check_result(pi,0,1,0,"pi"); printf("]}\n");
        mag_clear(pi);
        mpfr_clears(lo,hi,val,quality,xx,(mpfr_ptr)0);
    }
    assert(fegetround() == FE_TONEAREST);
    mpfr_free_cache(); flint_cleanup_master();
    printf("{\"kind\":\"summary\",\"checks\":%lu,\"qualityChecks\":%lu,\"inputChecks\":%lu,\"aliasChecks\":%lu,\"finiteMagResults\":%lu,\"infiniteMagResults\":%lu,\"failures\":%lu}\n",
        checks,quality_checks,input_checks,alias_checks,finite_results,infinite_results,failures);
    return failures ? 1 : 0;
}
