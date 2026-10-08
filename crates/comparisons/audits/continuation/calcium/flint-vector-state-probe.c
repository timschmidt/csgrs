#include <stdio.h>
#include "ca.h"
#include "ca_vec.h"

static void fill(ca_vec_t v, int kind, ca_ctx_t ctx)
{
    for (slong i = 0; i < ca_vec_length(v, ctx); i++)
    {
        ca_ptr x = ca_vec_entry(v, i);
        switch (kind)
        {
            case 0: ca_zero(x, ctx); break;
            case 1: ca_one(x, ctx); break;
            case 2: ca_neg_one(x, ctx); break;
            case 3: ca_set_si(x, 3 + i, ctx); ca_div_ui(x, x, 5 + 2*i, ctx); break;
            case 4: ca_sqrt_ui(x, 2, ctx); break;
            case 5: ca_pi(x, ctx); break;
            case 6:
                if (i % 3 == 0) ca_pi(x, ctx);
                else if (i % 3 == 1) ca_zero(x, ctx);
                else { ca_set_si(x, -3, ctx); ca_div_ui(x, x, 7, ctx); }
                break;
        }
    }
}

static int expected_negation(const ca_vec_t actual, const ca_vec_t original, ca_ctx_t ctx)
{
    ca_t expected;
    ca_init(expected, ctx);
    int correct = ca_vec_length(actual, ctx) == ca_vec_length(original, ctx);
    for (slong i = 0; correct && i < ca_vec_length(original, ctx); i++)
    {
        ca_neg(expected, ca_vec_entry(original, i), ctx);
        correct = ca_check_equal(expected, ca_vec_entry(actual, i), ctx) == T_TRUE;
    }
    ca_clear(expected, ctx);
    return correct;
}

int main(void)
{
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    const slong lengths[] = {0, 1, 2, 5, 16};
    const char * modes[] = {"separate", "public-in-place", "raw-in-place"};
    int cases = 0, failed = 0, zero_checks = 0, zero_failed = 0;
    puts("kind,length,mode,correct");
    for (int kind = 0; kind < 7; kind++) for (int l = 0; l < 5; l++)
    {
        ca_vec_t original;
        ca_vec_init(original, lengths[l], ctx); fill(original, kind, ctx);
        for (int mode = 0; mode < 3; mode++)
        {
            ca_vec_t actual;
            ca_vec_init(actual, 0, ctx);
            if (mode == 0) ca_vec_neg(actual, original, ctx);
            else
            {
                ca_vec_set(actual, original, ctx);
                if (mode == 1) ca_vec_neg(actual, actual, ctx);
                else _ca_vec_neg(ca_vec_entry(actual, 0), ca_vec_entry(actual, 0), lengths[l], ctx);
            }
            int correct = expected_negation(actual, original, ctx);
            printf("%d,%ld,%s,%d\n", kind, (long) lengths[l], modes[mode], correct);
            cases++; failed += !correct;
            ca_vec_clear(actual, ctx);
        }
        ca_vec_clear(original, ctx);
    }
    // A known nonzero coordinate dominates unknown coordinates in any position.
    for (int code = 0; code < 27; code++)
    {
        ca_vec_t v;
        ca_vec_init(v, 3, ctx);
        int digits = code, nonzero = 0, unknown = 0;
        for (slong i = 0; i < 3; i++)
        {
            int digit = digits % 3; digits /= 3;
            if (digit == 1) { ca_one(ca_vec_entry(v, i), ctx); nonzero = 1; }
            if (digit == 2) { ca_unknown(ca_vec_entry(v, i), ctx); unknown = 1; }
        }
        truth_t expected = nonzero ? T_FALSE : unknown ? T_UNKNOWN : T_TRUE;
        zero_failed += _ca_vec_check_is_zero(ca_vec_entry(v, 0), 3, ctx) != expected;
        zero_checks++; ca_vec_clear(v, ctx);
    }
    printf("{\"suite\":\"vector-state\",\"negation_cases\":%d,\"negation_failed\":%d,\"zero_cases\":%d,\"zero_failed\":%d}\n",
        cases, failed, zero_checks, zero_failed);
    ca_ctx_clear(ctx);
    flint_cleanup();
    return failed || zero_failed;
}
