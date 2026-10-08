/* Mathematical controls for principal roots of positive real exponentials.
   Independent probe; no donor implementation copied. */
#include <stdio.h>
#include "ca.h"

static void argument(ca_t x, int which, ca_ctx_t ctx)
{
    ca_t t;
    ca_init(t, ctx);
    switch (which)
    {
        case 0: ca_set_ui(x, 1, ctx); ca_div_ui(x, x, 3, ctx); break;
        case 1: ca_sqrt_ui(x, 2, ctx); break;
        case 2: ca_one(x, ctx); ca_sin(x, x, ctx); break;
        case 3: ca_pi(x, ctx); ca_sub_ui(x, x, 3, ctx); break;
        case 4:
            ca_set_ui(x, 2, ctx); ca_log(x, x, ctx);
            ca_set_ui(t, 3, ctx); ca_log(t, t, ctx);
            ca_add(x, x, t, ctx); break;
        case 5:
            ca_sqrt_ui(x, 2, ctx); ca_sqrt_ui(t, 3, ctx);
            ca_add(x, x, t, ctx); break;
        case 6: ca_sqrt_ui(x, 2, ctx); ca_mul_ui(x, x, 32, ctx); break;
        case 7: ca_sqrt_ui(x, 2, ctx); ca_div_ui(x, x, 1024, ctx); break;
    }
    ca_clear(t, ctx);
}

int main(void)
{
    const slong budgets[] = {64, 256, 512};
    int false_results = 0;
    puts("case,negative,precision,control,outcome");
    for (int which = 0; which < 8; which++)
    for (int negative = 0; negative < 2; negative++)
    for (int p = 0; p < 3; p++)
    for (int perturbed = 0; perturbed < 2; perturbed++)
    {
        ca_ctx_t ctx;
        ca_t x, y, left, right, epsilon;
        ca_ctx_init(ctx);
        ca_ctx_set_option(ctx, CA_OPT_PREC_LIMIT, budgets[p]);
        ca_init(x, ctx); ca_init(y, ctx); ca_init(left, ctx);
        ca_init(right, ctx); ca_init(epsilon, ctx);
        argument(x, which, ctx); argument(y, which, ctx);
        if (negative) { ca_neg(x, x, ctx); ca_neg(y, y, ctx); }
        ca_exp(left, x, ctx); ca_sqrt(left, left, ctx);
        ca_div_ui(y, y, 2, ctx); ca_exp(right, y, ctx);
        if (perturbed)
        {
            ca_one(epsilon, ctx); ca_div_ui(epsilon, epsilon, 1024, ctx);
            ca_add(right, right, epsilon, ctx);
        }
        truth_t result = ca_check_equal(left, right, ctx);
        false_results += (perturbed && result == T_TRUE) || (!perturbed && result == T_FALSE);
        const char *outcome = result == T_TRUE ? "Equal" : result == T_FALSE ? "NotEqual" : "Unknown";
        printf("%d,%s,%ld,%s,%s\n", which, negative ? "true" : "false", -budgets[p],
            perturbed ? "perturbed" : "identity", outcome);
        ca_clear(x, ctx); ca_clear(y, ctx); ca_clear(left, ctx);
        ca_clear(right, ctx); ca_clear(epsilon, ctx); ca_ctx_clear(ctx);
    }
    flint_cleanup();
    return false_results ? 1 : 0;
}
