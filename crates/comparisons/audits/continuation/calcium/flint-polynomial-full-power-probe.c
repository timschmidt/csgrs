#define main calcium_prior_series_main
#include "flint-polynomial-series-probe.c"
#undef main

int main(void)
{
    const slong lengths[] = {0, 1, 2, 4, 8};
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ca_t quadratic, quartic, other;
    ca_init(quadratic, ctx); ca_init(quartic, ctx); ca_init(other, ctx);
    ca_sqrt_ui(quadratic, 2, ctx); ca_sqrt_ui(other, 3, ctx);
    qqbar_t a2, a3;
    qqbar_init(a2); qqbar_init(a3);
    qqbar_sqrt_ui(a2, 2); qqbar_sqrt_ui(a3, 3); qqbar_add(a2, a2, a3);
    ca_set_qqbar(quartic, a2, ctx); qqbar_clear(a2); qqbar_clear(a3);
    int cases = 0;
    puts("mode,length,constant,exponent,checks,failures");
    for (int mode = 0; mode < 4; mode++) for (int li = 0; li < 5; li++)
    for (int constant = -1; constant <= 1; constant++)
    {
        ca_poly_t p, expected, next, actual, alias;
        ca_poly_init(p, ctx); ca_poly_init(expected, ctx); ca_poly_init(next, ctx);
        ca_poly_init(actual, ctx); ca_poly_init(alias, ctx);
        input(p, mode, lengths[li], constant, quadratic, quartic, other, ctx);
        if (lengths[li] == 0) ca_poly_zero(p, ctx);
        ca_poly_one(expected, ctx);
        for (ulong exponent = 0; exponent <= 8; exponent++)
        {
            int before_checks = checks, before_failures = failures;
            ca_poly_pow_ui(actual, p, exponent, ctx); equal(actual, expected, ctx);
            ca_poly_set(alias, p, ctx); ca_poly_pow_ui(alias, alias, exponent, ctx);
            equal(alias, expected, ctx);
            printf("%d,%ld,%d,%lu,%d,%d\n", mode, lengths[li], constant,
                exponent, checks - before_checks, failures - before_failures);
            cases++;
            if (exponent < 8)
            {
                // The full expected power uses scalar multiplication/addition
                // only. The length bound is deliberately larger than needed
                // for zero polynomials, so no coefficient is truncated.
                product(next, expected, p, expected->length + p->length, ctx);
                ca_poly_swap(next, expected, ctx);
            }
        }
        ca_poly_clear(p, ctx); ca_poly_clear(expected, ctx); ca_poly_clear(next, ctx);
        ca_poly_clear(actual, ctx); ca_poly_clear(alias, ctx);
    }
    ca_clear(quadratic, ctx); ca_clear(quartic, ctx); ca_clear(other, ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"full-polynomial-powers\",\"cases\":%d,\"checks\":%d,\"failures\":%d}\n", cases, checks, failures);
    return failures != 0;
}
