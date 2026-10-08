/* Independently specified phase, rounding and binary64 boundary controls. */
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ca.h"

static void pi_fraction(ca_t out, int n, unsigned d, ca_ctx_t ctx)
{
    ca_pi(out, ctx);
    ca_mul_si(out, out, n, ctx);
    ca_div_ui(out, out, d, ctx);
}

static int phase(ca_ctx_t ctx)
{
    const char *names[] = {"minus-one", "minus-two", "minus-third", "minus-sqrt-two",
        "zero", "one", "i", "minus-i", "one-plus-i", "one-minus-i", "negative-infinity",
        "positive-infinity", "unknown", "undefined", "unsigned-infinity"};
    const int numerator[] = {1, 1, 1, 1, 0, 0, 1, -1, 1, -1, 1, 0};
    const unsigned denominator[] = {1, 1, 1, 1, 1, 1, 2, 2, 4, 4, 1, 1};
    ca_t x, y, expected;
    ca_init(x, ctx); ca_init(y, ctx); ca_init(expected, ctx);
    int failed = 0;
    puts("kind,input,state,outcome,passed");
    for (int input = 0; input < 15; input++) for (int alias = 0; alias < 2; alias++)
    {
        switch (input)
        {
            case 0: ca_set_si(x, -1, ctx); break;
            case 1: ca_set_si(x, -2, ctx); break;
            case 2: ca_set_si(x, -1, ctx); ca_div_ui(x, x, 3, ctx); break;
            case 3: ca_set_ui(x, 2, ctx); ca_sqrt(x, x, ctx); ca_neg(x, x, ctx); break;
            case 4: ca_zero(x, ctx); break;
            case 5: ca_one(x, ctx); break;
            case 6: ca_i(x, ctx); break;
            case 7: ca_neg_i(x, ctx); break;
            case 8: ca_set_d_d(x, 1.0, 1.0, ctx); break;
            case 9: ca_set_d_d(x, 1.0, -1.0, ctx); break;
            case 10: ca_neg_inf(x, ctx); break;
            case 11: ca_pos_inf(x, ctx); break;
            case 12: ca_unknown(x, ctx); break;
            case 13: ca_undefined(x, ctx); break;
            default: ca_uinf(x, ctx); break;
        }
        ca_unknown(y, ctx);
        if (alias) { ca_set(y, x, ctx); ca_arg(y, y, ctx); }
        else ca_arg(y, x, ctx);
        truth_t equal;
        if (input < 12)
        {
            pi_fraction(expected, numerator[input], denominator[input], ctx);
            equal = ca_check_equal(y, expected, ctx);
        }
        else if (input == 12) equal = ca_is_unknown(y, ctx) ? T_TRUE : T_FALSE;
        else equal = ca_check_is_undefined(y, ctx);
        int pass = equal == T_TRUE;
        printf("phase,%s,%s,%s,%d\n", names[input], alias ? "in-place" : "separate",
            equal == T_TRUE ? "Equal" : equal == T_FALSE ? "NotEqual" : "Unknown", pass);
        failed += !pass;
    }
    ca_clear(x, ctx); ca_clear(y, ctx); ca_clear(expected, ctx);
    return failed;
}

static int binary64(ca_ctx_t ctx)
{
    const double edges[] = {0.0, -0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN,
        DBL_MAX, -DBL_MAX, 0.1, -0.1, INFINITY, -INFINITY, NAN};
    uint64_t seed = UINT64_C(918173);
    ca_t x;
    arf_t a;
    fmpq_t q;
    mpq_t expected, actual;
    ca_init(x, ctx); arf_init(a); fmpq_init(q); mpq_init(expected); mpq_init(actual);
    int failed = 0;
    for (int i = 0; i < 10013; i++)
    {
        double d;
        if (i < 13) d = edges[i];
        else
        {
            seed = seed * UINT64_C(6364136223846793005) + UINT64_C(1);
            memcpy(&d, &seed, sizeof d);
        }
        ca_set_d(x, d, ctx);
        arf_set_d(a, d);
        int pass = arf_allocated_bytes(a) == 0;
        if (isfinite(d))
        {
            mpq_set_d(expected, d);
            int converted = ca_get_fmpq(q, x, ctx);
            if (converted) fmpq_get_mpq(actual, q);
            pass &= converted && mpq_equal(expected, actual);
        }
        else if (isnan(d)) pass &= ca_is_unknown(x, ctx);
        else pass &= (signbit(d) ? ca_check_is_neg_inf(x, ctx) : ca_check_is_pos_inf(x, ctx)) == T_TRUE;
        if (!pass) printf("binary64-failure,%d\n", i);
        failed += !pass;
    }
    printf("{\"suite\":\"binary64\",\"cases\":10013,\"failed\":%d}\n", failed);
    ca_clear(x, ctx); arf_clear(a); fmpq_clear(q); mpq_clear(expected); mpq_clear(actual);
    return failed;
}

static int rounding(ca_ctx_t ctx)
{
    const double real[] = {-2.0, -1.5, -0.5, 0.0, 0.5, 1.5, 2.0};
    const slong floors[] = {-2, -2, -1, 0, 0, 1, 2};
    const slong ceilings[] = {-2, -1, 0, 0, 1, 2, 2};
    ca_t x, result, expected;
    ca_init(x, ctx); ca_init(result, ctx); ca_init(expected, ctx);
    int failed = 0;
    for (int i = 0; i < 7; i++) for (int imaginary = -1; imaginary <= 1; imaginary++)
        for (int ceiling = 0; ceiling < 2; ceiling++) for (int alias = 0; alias < 2; alias++)
    {
        ca_set_d_d(x, real[i], imaginary, ctx);
        ca_set_si(expected, ceiling ? ceilings[i] : floors[i], ctx);
        if (alias) ca_set(result, x, ctx);
        if (ceiling) ca_ceil(result, alias ? result : x, ctx);
        else ca_floor(result, alias ? result : x, ctx);
        failed += ca_check_equal(result, expected, ctx) != T_TRUE;
    }
    printf("{\"suite\":\"rounding\",\"cases\":84,\"failed\":%d}\n", failed);
    ca_clear(x, ctx); ca_clear(result, ctx); ca_clear(expected, ctx);
    return failed;
}

int main(void)
{
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ctx->options[CA_OPT_PREC_LIMIT] = 256;
    int phase_failures = phase(ctx);
    printf("{\"suite\":\"phase\",\"cases\":30,\"failed\":%d}\n", phase_failures);
    int failed = phase_failures + binary64(ctx) + rounding(ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    return failed ? 1 : 0;
}
