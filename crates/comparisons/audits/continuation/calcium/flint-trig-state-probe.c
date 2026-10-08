/* Independent destination-state, alias and known complex-branch controls. */
#include <stdio.h>
#include <string.h>
#include "ca.h"

typedef void (*unary)(ca_t, const ca_t, ca_ctx_t);
typedef void (*constructor)(ca_t, ca_ctx_t);

static const char *classify(const ca_t x, ca_ctx_t ctx)
{
    if (ca_is_unknown(x, ctx)) return "unknown";
    if (ca_check_is_undefined(x, ctx) == T_TRUE) return "undefined";
    if (ca_check_is_i(x, ctx) == T_TRUE) return "i";
    if (ca_check_is_neg_i(x, ctx) == T_TRUE) return "minus_i";
    return "other";
}

int main(void)
{
    const unary funcs[] = {ca_tan, ca_tan_direct, ca_tan_exponential, ca_tan_sine_cosine, ca_cot};
    const char *fnames[] = {"tan", "tan_direct", "tan_exponential", "tan_sine_cosine", "cot"};
    const constructor inputs[] = {ca_unknown, ca_undefined, ca_uinf, ca_pos_inf, ca_neg_inf, ca_pos_i_inf, ca_neg_i_inf};
    const char *inames[] = {"unknown", "undefined", "uinf", "posinf", "neginf", "posiinf", "negiinf"};
    const constructor states[] = {ca_zero, ca_unknown, ca_undefined, ca_i};
    const char *snames[] = {"zero", "unknown", "undefined", "i"};
    ca_ctx_t ctx;
    ca_t x, y, direct, separate, aliased, expected, pi, log3;
    ca_ctx_init(ctx);
    ca_init(x, ctx); ca_init(y, ctx); ca_init(direct, ctx); ca_init(separate, ctx);
    ca_init(aliased, ctx); ca_init(expected, ctx); ca_init(pi, ctx); ca_init(log3, ctx);
    int failures = 0;
    puts("kind,function,input,state,expected,actual,passed");
    for (int f = 0; f < 5; f++) for (int i = 0; i < 7; i++) for (int s = 0; s < 5; s++)
    {
        inputs[i](x, ctx);
        const char *want = i == 0 ? "unknown" : i < 5 ? "undefined" :
            (i == 5) == (f != 4) ? "i" : "minus_i";
        const char *state;
        if (s == 4)
        {
            ca_set(y, x, ctx); funcs[f](y, y, ctx); state = "in-place";
        }
        else
        {
            states[s](y, ctx); funcs[f](y, x, ctx); state = snames[s];
        }
        const char *actual = classify(y, ctx);
        int pass = !strcmp(actual, want);
        printf("special,%s,%s,%s,%s,%s,%d\n", fnames[f], inames[i], state, want, actual, pass);
        failures += !pass;
    }

    // Principal atan(2i) = pi/2 + i*log(3)/2, atan(-2i) is its negative.
    // The direct constructor and distinct-output logarithmic path are extra
    // comparators, not the source of the expected formula.
    ca_pi(pi, ctx); ca_div_ui(pi, pi, 2, ctx);
    ca_set_ui(log3, 3, ctx); ca_log(log3, log3, ctx); ca_div_ui(log3, log3, 2, ctx);
    ca_i(y, ctx); ca_mul(log3, log3, y, ctx); ca_add(expected, pi, log3, ctx);
    for (int sign = 1; sign >= -1; sign -= 2)
    {
        ca_i(x, ctx); ca_mul_si(x, x, 2 * sign, ctx);
        if (sign < 0) ca_neg(expected, expected, ctx);
        ca_atan_direct(direct, x, ctx);
        ca_atan_logarithm(separate, x, ctx);
        ca_set(aliased, x, ctx); ca_atan_logarithm(aliased, aliased, ctx);
        const char *states2[] = {"direct", "separate", "in-place"};
        ca_srcptr values[] = {direct, separate, aliased};
        for (int j = 0; j < 3; j++)
        {
            truth_t eq = ca_check_equal(values[j], expected, ctx);
            int pass = eq == T_TRUE;
            printf("branch,atan_logarithm,%di,%s,Equal,%s,%d\n", 2 * sign, states2[j],
                eq == T_TRUE ? "Equal" : eq == T_FALSE ? "NotEqual" : "Unknown", pass);
            failures += !pass;
        }
    }
    ca_clear(x, ctx); ca_clear(y, ctx); ca_clear(direct, ctx); ca_clear(separate, ctx);
    ca_clear(aliased, ctx); ca_clear(expected, ctx); ca_clear(pi, ctx); ca_clear(log3, ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    return failures ? 1 : 0;
}
