/* Independent special-value and closed-form controls, no donor code copied. */
#include <stdio.h>
#include <string.h>
#include "ca.h"

static const char *classify(const ca_t x, ca_ctx_t ctx)
{
    if (ca_is_unknown(x, ctx)) return "unknown";
    if (ca_check_is_undefined(x, ctx) == T_TRUE) return "undefined";
    if (ca_check_is_pos_inf(x, ctx) == T_TRUE) return "positive_infinity";
    if (ca_check_is_zero(x, ctx) == T_TRUE) return "zero";
    return "other";
}

int main(void)
{
    typedef void (*constructor)(ca_t, ca_ctx_t);
    const constructor inputs[] = {ca_pos_inf, ca_neg_inf, ca_unknown, ca_undefined,
                                  ca_uinf, ca_pos_i_inf, ca_neg_i_inf};
    const char *names[] = {"positive_infinity", "negative_infinity", "unknown", "undefined",
                           "unsigned_infinity", "positive_imaginary_infinity", "negative_imaginary_infinity"};
    const char *expected[] = {"positive_infinity", "undefined", "unknown", "undefined", "undefined", "zero", "zero"};
    int failures = 0;
    ca_ctx_t ctx;
    ca_t x, y, correct;
    ca_ctx_init(ctx);
    ca_init(x, ctx); ca_init(y, ctx); ca_init(correct, ctx);
    printf("truth_encoding,true=%d,false=%d,unknown=%d\n", T_TRUE, T_FALSE, T_UNKNOWN);
    puts("case,expected,actual,passed");
    for (int i = 0; i < 7; i++)
    {
        inputs[i](x, ctx);
        ca_gamma(y, x, ctx);
        const char *actual = classify(y, ctx);
        int passed = strcmp(actual, expected[i]) == 0;
        printf("%s,%s,%s,%d\n", names[i], expected[i], actual, passed);
        failures += !passed;
    }
    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
        {
            ca_set_ui(x, 3, ctx); ca_set_ui(correct, 2, ctx);
        }
        else
        {
            ca_set_si(x, i == 1 ? 1 : -1, ctx); ca_div_ui(x, x, 2, ctx);
            ca_pi(correct, ctx); ca_sqrt(correct, correct, ctx);
            if (i == 2) ca_mul_si(correct, correct, -2, ctx);
        }
        ca_gamma(y, x, ctx);
        int passed = ca_check_equal(y, correct, ctx) == T_TRUE;
        printf("finite_control_%d,Equal,%s,%d\n", i, passed ? "Equal" : "UnknownOrUnequal", passed);
        failures += !passed;
    }
    ca_clear(x, ctx); ca_clear(y, ctx); ca_clear(correct, ctx); ca_ctx_clear(ctx);
    flint_cleanup();
    return failures ? 1 : 0;
}
