#include <stdio.h>
#include "ca.h"
#include "ca_poly.h"
#include "qqbar.h"

static int checks = 0, failures = 0;
static void equal_poly(const ca_poly_t a, const ca_poly_t b, ca_ctx_t ctx)
{
    checks++;
    failures += ca_poly_check_equal(a, b, ctx) != T_TRUE;
}
static void equal_scalar(const ca_t a, const ca_t b, ca_ctx_t ctx)
{
    checks++;
    failures += ca_check_equal(a, b, ctx) != T_TRUE;
}
static void input(ca_poly_t p, int mode, slong length, int side,
    ca_srcptr quadratic, ca_srcptr quartic, ca_srcptr other, ca_ctx_t ctx)
{
    ca_t c;
    ca_init(c, ctx);
    for (slong i = 0; i < length; i++)
    {
        ca_set_si(c, (i + side) % 2 ? -i - 2 : i + 1, ctx);
        if (mode != 0) ca_div_ui(c, c, (ulong)(i % 4 + 2), ctx);
        if (mode >= 2 && i % 3 != 0)
            ca_add(c, c, mode == 3 ? quartic : mode == 4 && i % 2 ? other : quadratic, ctx);
        ca_poly_set_coeff_ca(p, i, c, ctx);
    }
    ca_clear(c, ctx);
}
// Independent scalar convolution: no polynomial multiplication or packed field
// operation is used in the expected coefficient construction.
static void product(ca_poly_t out, const ca_poly_t a, const ca_poly_t b, slong n, ca_ctx_t ctx)
{
    ca_t c;
    ca_init(c, ctx);
    ca_poly_zero(out, ctx);
    for (slong i = 0; i < a->length; i++) for (slong j = 0; j < b->length && i + j < n; j++)
    {
        ca_mul(c, a->coeffs + i, b->coeffs + j, ctx);
        if (i + j < out->length) ca_add(c, c, out->coeffs + i + j, ctx);
        ca_poly_set_coeff_ca(out, i + j, c, ctx);
    }
    ca_clear(c, ctx);
}
static void evaluation(ca_t out, const ca_poly_t p, const ca_t x, ca_ctx_t ctx)
{
    ca_t power, term;
    ca_init(power, ctx); ca_init(term, ctx);
    ca_one(power, ctx); ca_zero(out, ctx);
    for (slong i = 0; i < p->length; i++)
    {
        ca_mul(term, p->coeffs + i, power, ctx);
        ca_add(out, out, term, ctx);
        ca_mul(power, power, x, ctx);
    }
    ca_clear(power, ctx); ca_clear(term, ctx);
}
int main(void)
{
    const slong lengths[] = {0, 1, 3, 4, 10, 12}, truncations[] = {0, 1, 3, 8, 23};
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ca_t quadratic, quartic, other, x, actual, expected;
    ca_init(quadratic, ctx); ca_init(quartic, ctx); ca_init(other, ctx);
    ca_init(x, ctx); ca_init(actual, ctx); ca_init(expected, ctx);
    ca_sqrt_ui(quadratic, 2, ctx); ca_sqrt_ui(other, 3, ctx);
    qqbar_t a2, a3;
    qqbar_init(a2); qqbar_init(a3);
    qqbar_sqrt_ui(a2, 2); qqbar_sqrt_ui(a3, 3); qqbar_add(a2, a2, a3);
    ca_set_qqbar(quartic, a2, ctx);
    qqbar_clear(a2); qqbar_clear(a3);
    int cases = 0;
    puts("mode,left_length,right_length,checks,failures");
    for (int mode = 0; mode < 5; mode++) for (int li = 0; li < 6; li++) for (int ri = 0; ri < 6; ri++)
    {
        ca_poly_t a, b, result, reference, alias;
        ca_poly_init(a, ctx); ca_poly_init(b, ctx); ca_poly_init(result, ctx);
        ca_poly_init(reference, ctx); ca_poly_init(alias, ctx);
        input(a, mode, lengths[li], 0, quadratic, quartic, other, ctx);
        input(b, mode, lengths[ri], 1, quadratic, quartic, other, ctx);
        int before_checks = checks, before_failures = failures;
        product(reference, a, b, 23, ctx);
        ca_poly_mul(result, a, b, ctx); equal_poly(result, reference, ctx);
        ca_poly_set(alias, a, ctx); ca_poly_mul(alias, alias, b, ctx); equal_poly(alias, reference, ctx);
        ca_poly_set(alias, b, ctx); ca_poly_mul(alias, a, alias, ctx); equal_poly(alias, reference, ctx);
        product(reference, a, a, 23, ctx);
        ca_poly_mul(result, a, a, ctx); equal_poly(result, reference, ctx);
        for (int t = 0; t < 5; t++)
        {
            product(reference, a, b, truncations[t], ctx);
            ca_poly_mullow(result, a, b, truncations[t], ctx); equal_poly(result, reference, ctx);
            ca_poly_set(alias, a, ctx); ca_poly_mullow(alias, alias, b, truncations[t], ctx); equal_poly(alias, reference, ctx);
            ca_poly_set(alias, b, ctx); ca_poly_mullow(alias, a, alias, truncations[t], ctx); equal_poly(alias, reference, ctx);
        }
        ca_poly_integral(result, a, ctx); ca_poly_derivative(result, result, ctx); equal_poly(result, a, ctx);
        ca_poly_set(alias, a, ctx); ca_poly_integral(alias, alias, ctx);
        ca_poly_derivative(result, alias, ctx); equal_poly(result, a, ctx);
        ca_poly_neg(result, a, ctx); ca_poly_neg(result, result, ctx); equal_poly(result, a, ctx);
        ca_poly_set(alias, a, ctx); ca_poly_neg(alias, alias, ctx); ca_poly_neg(result, alias, ctx); equal_poly(result, a, ctx);
        for (int point = 0; point < 2; point++)
        {
            ca_set_si(x, point ? 3 : 0, ctx); ca_div_ui(x, x, 2, ctx);
            evaluation(expected, a, x, ctx);
            ca_poly_evaluate(actual, a, x, ctx); equal_scalar(actual, expected, ctx);
            ca_set(actual, x, ctx); ca_poly_evaluate_horner(actual, a, actual, ctx); equal_scalar(actual, expected, ctx);
        }
        printf("%d,%ld,%ld,%d,%d\n", mode, (long)lengths[li], (long)lengths[ri], checks - before_checks, failures - before_failures);
        cases++;
        ca_poly_clear(a, ctx); ca_poly_clear(b, ctx); ca_poly_clear(result, ctx);
        ca_poly_clear(reference, ctx); ca_poly_clear(alias, ctx);
    }
    ca_clear(quadratic, ctx); ca_clear(quartic, ctx); ca_clear(other, ctx);
    ca_clear(x, ctx); ca_clear(actual, ctx); ca_clear(expected, ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"polynomial-arithmetic\",\"cases\":%d,\"checks\":%d,\"failures\":%d}\n", cases, checks, failures);
    return failures != 0;
}
