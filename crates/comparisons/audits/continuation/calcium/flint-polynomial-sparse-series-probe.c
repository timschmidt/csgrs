#define main dense_series_control_main
#include "flint-polynomial-series-probe.c"
#undef main

int main(void)
{
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ca_t c, b;
    ca_init(c, ctx); ca_init(b, ctx);
    int cases = 0;
    puts("kind,degree,constant,order,checks,failures");
    for (int kind = 0; kind < 2; kind++) for (slong degree = 1; degree <= 9; degree++)
    for (int constant = 1; constant <= 2; constant++) for (slong n = 0; n <= 20; n++)
    {
        ca_poly_t p, ref, actual, alias;
        ca_poly_init(p, ctx); ca_poly_init(ref, ctx); ca_poly_init(actual, ctx); ca_poly_init(alias, ctx);
        ca_set_si(c, constant, ctx); ca_poly_set_coeff_ca(p, 0, c, ctx);
        if (kind) ca_sqrt_ui(b, 2, ctx); else ca_set_si(b, -2, ctx);
        ca_poly_set_coeff_ca(p, degree, b, ctx);
        int before_checks = checks, before_failures = failures;
        logarithm(ref, p, n, ctx);
        ca_poly_log_series(actual, p, n, ctx); equal(actual, ref, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_log_series(alias, alias, n, ctx); equal(alias, ref, ctx);
        ca_set_si(c, constant - 1, ctx); ca_poly_set_coeff_ca(p, 0, c, ctx);
        exponential(ref, p, n, ctx);
        ca_poly_exp_series(actual, p, n, ctx); equal(actual, ref, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_exp_series(alias, alias, n, ctx); equal(alias, ref, ctx);
        printf("%d,%ld,%d,%ld,%d,%d\n", kind, (long)degree, constant, (long)n, checks - before_checks, failures - before_failures);
        cases++;
        ca_poly_clear(p, ctx); ca_poly_clear(ref, ctx); ca_poly_clear(actual, ctx); ca_poly_clear(alias, ctx);
    }
    ca_clear(c, ctx); ca_clear(b, ctx); ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"polynomial-sparse-series\",\"cases\":%d,\"checks\":%d,\"failures\":%d}\n", cases, checks, failures);
    return failures != 0;
}
