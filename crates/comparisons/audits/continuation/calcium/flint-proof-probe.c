/* Independent public-API oracle; no Calcium implementation code is copied. */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ca.h"

static void report(const char *kind, long a, long d, long prec,
                   const ca_t left, const ca_t right, int expected,
                   ca_ctx_t ctx)
{
    clock_t start = clock();
    truth_t answer = ca_check_equal(left, right, ctx);
    const char *name = answer == T_TRUE ? "Equal" :
                       answer == T_FALSE ? "NotEqual" : "Unknown";
    printf("%s,%ld,%ld,%ld,%s,%.0f\n", kind, a, d, prec, name,
           1e9 * (double) (clock() - start) / CLOCKS_PER_SEC);
    if ((expected && answer == T_FALSE) || (!expected && answer == T_TRUE))
    {
        fprintf(stderr, "wrong proof result: %s a=%ld d=%ld\n", kind, a, d);
        abort();
    }
}

int main(void)
{
    const ulong ds[] = {2, 3, 5, 6, 7, 10, 11, 13, 17, 19};
    const slong precisions[] = {64, 256, 512};
    puts("kind,a,d,precision,outcome,elapsed_ns");
    for (ulong a = 1; a <= 3; a++)
    for (size_t k = 0; k < sizeof(ds) / sizeof(ds[0]); k++)
    {
        ulong d = ds[k];
        ca_ctx_t ctx;
        ca_t root, x, expanded, twice_log, log_square, perturbed, square, t;
        ca_ctx_init(ctx);
        ca_init(root, ctx); ca_init(x, ctx); ca_init(expanded, ctx);
        ca_init(twice_log, ctx); ca_init(log_square, ctx);
        ca_init(perturbed, ctx); ca_init(square, ctx); ca_init(t, ctx);

        ca_sqrt_ui(root, d, ctx);
        ca_add_ui(x, root, a, ctx);
        ca_mul_ui(expanded, root, 2 * a, ctx);
        ca_add_ui(expanded, expanded, a * a + d, ctx);
        ca_log(twice_log, x, ctx);
        ca_mul_ui(twice_log, twice_log, 2, ctx);
        ca_log(log_square, expanded, ctx);
        ca_one(t, ctx);
        ca_div_ui(t, t, 1024, ctx);
        ca_add(perturbed, expanded, t, ctx);
        ca_log(perturbed, perturbed, ctx);
        ca_sqr(square, x, ctx);
        for (size_t p = 0; p < sizeof(precisions) / sizeof(precisions[0]); p++)
        {
            // Both APIs' budgets are recorded, not asserted to have identical
            // scheduling semantics. Timings are diagnostics, not an A/B bench.
            slong prec = precisions[p];
            ca_ctx_set_option(ctx, CA_OPT_PREC_LIMIT, prec);
            report("algebraic_control", a, d, prec, square, expanded, 1, ctx);
            report("log_identity", a, d, prec, twice_log, log_square, 1, ctx);
            report("perturbed_control", a, d, prec, twice_log, perturbed, 0, ctx);
        }

        // Principal complex logarithms: log(-x)+log(-x) differs from
        // log(x*x) by 2*pi*i. Exact product equality alone is insufficient.
        ca_neg(t, x, ctx);
        ca_log(t, t, ctx);
        ca_mul_ui(t, t, 2, ctx);
        report("complex_branch_control", a, d, 512, t, log_square, 0, ctx);

        ca_clear(root, ctx); ca_clear(x, ctx); ca_clear(expanded, ctx);
        ca_clear(twice_log, ctx); ca_clear(log_square, ctx);
        ca_clear(perturbed, ctx); ca_clear(square, ctx); ca_clear(t, ctx);
        ca_ctx_clear(ctx);
    }
    flint_cleanup();
    return 0;
}
