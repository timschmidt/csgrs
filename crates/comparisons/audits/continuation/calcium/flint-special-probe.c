/* Independent total-real-function identities; no donor source is copied. */
#include <stdio.h>
#include <stdlib.h>
#include "ca.h"

static void argument(ca_t x, int which, ca_ctx_t ctx)
{
    switch (which)
    {
        case 0: ca_set_si(x, -2, ctx); break;
        case 1: ca_set_si(x, -1, ctx); ca_div_ui(x, x, 3, ctx); break;
        case 2: ca_zero(x, ctx); break;
        case 3: ca_one(x, ctx); ca_div_ui(x, x, 3, ctx); break;
        case 4: ca_set_ui(x, 2, ctx); break;
        case 5: ca_sqrt_ui(x, 2, ctx); break;
        case 6: ca_pi(x, ctx); break;
        default: abort();
    }
}

int main(void)
{
    const slong precisions[] = {64, 256, 512};
    const char *names[] = {"complement", "erf_odd", "erfc_reflection"};
    puts("kind,case,precision,control,outcome");
    for (int which = 0; which < 7; which++)
    for (int p = 0; p < 3; p++)
    for (int kind = 0; kind < 3; kind++)
    for (int perturb = 0; perturb < 2; perturb++)
    {
        ca_ctx_t ctx;
        ca_t x, left, right, t;
        ca_ctx_init(ctx);
        ca_ctx_set_option(ctx, CA_OPT_PREC_LIMIT, precisions[p]);
        ca_init(x, ctx); ca_init(left, ctx); ca_init(right, ctx); ca_init(t, ctx);
        argument(x, which, ctx);
        if (kind == 0)
        {
            ca_erf(left, x, ctx); ca_erfc(t, x, ctx);
            ca_one(right, ctx);
        }
        else if (kind == 1)
        {
            ca_erf(left, x, ctx); ca_neg(x, x, ctx); ca_erf(t, x, ctx);
            ca_zero(right, ctx);
        }
        else
        {
            ca_erfc(left, x, ctx); ca_neg(x, x, ctx); ca_erfc(t, x, ctx);
            ca_set_ui(right, 2, ctx);
        }
        ca_add(left, left, t, ctx);
        if (perturb)
        {
            ca_one(t, ctx); ca_div_ui(t, t, 1024, ctx);
            ca_add(right, right, t, ctx);
        }
        truth_t answer = ca_check_equal(left, right, ctx);
        if ((perturb && answer == T_TRUE) || (!perturb && answer == T_FALSE))
            abort();
        const char *result = answer == T_TRUE ? "Equal" : answer == T_FALSE ? "NotEqual" : "Unknown";
        printf("%s,%d,%ld,%s,%s\n", names[kind], which, (long) precisions[p],
               perturb ? "perturbed" : "identity", result);
        ca_clear(x, ctx); ca_clear(left, ctx); ca_clear(right, ctx); ca_clear(t, ctx);
        ca_ctx_clear(ctx);
    }
    flint_cleanup();
    return 0;
}
