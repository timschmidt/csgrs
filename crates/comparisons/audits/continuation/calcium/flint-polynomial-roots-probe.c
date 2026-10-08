#define main series_control_main
#include "flint-polynomial-series-probe.c"
#undef main

int main(void)
{
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ca_t c, value;
    ca_init(c, ctx); ca_init(value, ctx);
    int cases = 0, factor_failures = 0, root_failures = 0, root_unmatched = 0;
    puts("kind,code,set_equal,factor_success,factor_equal,squarefree_success,squarefree_equal,roots_success,root_count,matches");
    for (int kind = 0; kind < 3; kind++) for (int code = 0; code < 27; code++)
    {
        ca_vec_t roots, recovered;
        ca_vec_init(roots, 3, ctx); ca_vec_init(recovered, 0, ctx);
        ca_zero(roots->entries, ctx);
        if (kind == 0) { ca_one(roots->entries + 1, ctx); ca_set_si(roots->entries + 2, 2, ctx); }
        else if (kind == 1) {
            ca_sqrt_ui(roots->entries + 1, 2, ctx); ca_neg(roots->entries + 2, roots->entries + 1, ctx);
        } else {
            ca_set_si(value, 2, ctx); ca_log(roots->entries + 1, value, ctx);
            ca_add_ui(roots->entries + 2, roots->entries + 1, 1, ctx);
        }
        ulong exponents[3], recovered_exponents[7] = {0}, factor_exponents[7] = {0};
        int digits = code, expected_count = 0;
        for (int i = 0; i < 3; i++) { exponents[i] = (ulong)(digits % 3); digits /= 3; expected_count += exponents[i] != 0; }
        ca_poly_t p, reference, factor, next, squarefree, expected_squarefree, rebuilt;
        ca_poly_init(p, ctx); ca_poly_init(reference, ctx); ca_poly_init(factor, ctx); ca_poly_init(next, ctx);
        ca_poly_init(squarefree, ctx); ca_poly_init(expected_squarefree, ctx); ca_poly_init(rebuilt, ctx);
        ca_poly_one(reference, ctx); ca_poly_one(expected_squarefree, ctx);
        for (int i = 0; i < 3; i++)
        {
            ca_neg(value, roots->entries + i, ctx); ca_poly_set_coeff_ca(factor, 0, value, ctx);
            ca_one(value, ctx); ca_poly_set_coeff_ca(factor, 1, value, ctx);
            for (ulong e = 0; e < exponents[i]; e++) {
                product(next, reference, factor, 7, ctx); ca_poly_swap(next, reference, ctx);
            }
            if (exponents[i]) {
                product(next, expected_squarefree, factor, 7, ctx); ca_poly_swap(next, expected_squarefree, ctx);
            }
        }
        ca_poly_set_roots(p, roots, exponents, ctx);
        int set_equal = ca_poly_check_equal(p, reference, ctx) == T_TRUE;
        ca_set_si(value, code % 2 ? -2 : 2, ctx); ca_poly_mul_ca(p, p, value, ctx);
        ca_poly_vec_t factors;
        ca_poly_vec_init(factors, 0, ctx);
        int factor_success = ca_poly_factor_squarefree(c, factors, factor_exponents, p, ctx), factor_equal = 0;
        if (factor_success) {
            ca_poly_set_ca(rebuilt, c, ctx);
            for (slong i = 0; i < factors->length; i++) for (ulong e = 0; e < factor_exponents[i]; e++) {
                product(next, rebuilt, factors->entries + i, 7, ctx); ca_poly_swap(next, rebuilt, ctx);
            }
            factor_equal = ca_poly_check_equal(p, rebuilt, ctx) == T_TRUE;
        }
        int squarefree_success = ca_poly_squarefree_part(squarefree, p, ctx);
        int squarefree_equal = squarefree_success && ca_poly_check_equal(squarefree, expected_squarefree, ctx) == T_TRUE;
        int roots_success = ca_poly_roots(recovered, recovered_exponents, p, ctx), matches = 0;
        if (roots_success && recovered->length == expected_count) {
            int used[3] = {0};
            for (slong i = 0; i < recovered->length; i++) for (int j = 0; j < 3; j++) {
                if (!used[j] && exponents[j] && recovered_exponents[i] == exponents[j]
                    && ca_check_equal(recovered->entries + i, roots->entries + j, ctx) == T_TRUE) {
                    used[j] = 1; matches++; break;
                }
            }
        }
        factor_failures += !factor_success; root_failures += !roots_success;
        root_unmatched += roots_success && (recovered->length != expected_count || matches != expected_count);
        failures += !set_equal || !factor_success || !factor_equal || !squarefree_success || !squarefree_equal
            || !roots_success || recovered->length != expected_count || matches != expected_count;
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d\n", kind, code, set_equal, factor_success, factor_equal,
            squarefree_success, squarefree_equal, roots_success, (long)recovered->length, matches);
        cases++;
        ca_poly_vec_clear(factors, ctx);
        ca_poly_clear(p, ctx); ca_poly_clear(reference, ctx); ca_poly_clear(factor, ctx); ca_poly_clear(next, ctx);
        ca_poly_clear(squarefree, ctx); ca_poly_clear(expected_squarefree, ctx); ca_poly_clear(rebuilt, ctx);
        ca_vec_clear(roots, ctx); ca_vec_clear(recovered, ctx);
    }
    ca_clear(c, ctx); ca_clear(value, ctx); ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"polynomial-roots\",\"cases\":%d,\"failed_cases\":%d,\"factor_failures\":%d,\"root_failures\":%d,\"root_unmatched\":%d}\n",
        cases, failures, factor_failures, root_failures, root_unmatched);
    return failures != 0;
}
