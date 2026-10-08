#include <stdio.h>
#include "ca.h"
#include "ca_poly.h"
#include "qqbar.h"

static int checks = 0, failures = 0;
static ca_srcptr at(const ca_poly_t p, slong i, const ca_t zero)
{
    return i < p->length ? p->coeffs + i : zero;
}
static void equal(const ca_poly_t actual, const ca_poly_t expected, ca_ctx_t ctx)
{
    checks++;
    failures += ca_poly_check_equal(actual, expected, ctx) != T_TRUE;
}
static void input(ca_poly_t p, int mode, slong len, int constant,
    ca_srcptr quadratic, ca_srcptr quartic, ca_srcptr other, ca_ctx_t ctx)
{
    ca_t c;
    ca_init(c, ctx); ca_poly_zero(p, ctx);
    ca_set_si(c, constant, ctx); ca_poly_set_coeff_ca(p, 0, c, ctx);
    for (slong i = 1; i < len; i++)
    {
        ca_set_si(c, i % 2 ? -i : i + 1, ctx);
        ca_div_ui(c, c, (ulong)(i % 3 + 2), ctx);
        if (mode != 0)
            ca_add(c, c, mode == 2 ? quartic : mode == 3 && i % 2 ? other : quadratic, ctx);
        ca_poly_set_coeff_ca(p, i, c, ctx);
    }
    ca_clear(c, ctx);
}
// Deliberately scalar recurrences: expected values do not call the donor's
// polynomial series, dot product or polynomial convolution kernels.
static void product(ca_poly_t out, const ca_poly_t a, const ca_poly_t b, slong n, ca_ctx_t ctx)
{
    ca_t c;
    ca_init(c, ctx); ca_poly_zero(out, ctx);
    for (slong i = 0; i < a->length; i++) for (slong j = 0; j < b->length && i + j < n; j++)
    {
        ca_mul(c, a->coeffs + i, b->coeffs + j, ctx);
        if (i + j < out->length) ca_add(c, c, out->coeffs + i + j, ctx);
        ca_poly_set_coeff_ca(out, i + j, c, ctx);
    }
    ca_clear(c, ctx);
}
static void inverse(ca_poly_t out, const ca_poly_t p, slong n, ca_ctx_t ctx)
{
    ca_t zero, sum, term, inv;
    ca_init(zero, ctx); ca_init(sum, ctx); ca_init(term, ctx); ca_init(inv, ctx);
    ca_poly_zero(out, ctx);
    if (n > 0)
    {
        ca_inv(inv, p->coeffs, ctx); ca_poly_set_coeff_ca(out, 0, inv, ctx);
        for (slong k = 1; k < n; k++)
        {
            ca_zero(sum, ctx);
            for (slong i = 1; i < p->length && i <= k; i++)
            {
                ca_mul(term, p->coeffs + i, at(out, k - i, zero), ctx);
                ca_add(sum, sum, term, ctx);
            }
            ca_mul(sum, sum, inv, ctx); ca_neg(sum, sum, ctx);
            ca_poly_set_coeff_ca(out, k, sum, ctx);
        }
    }
    ca_clear(zero, ctx); ca_clear(sum, ctx); ca_clear(term, ctx); ca_clear(inv, ctx);
}
static void exponential(ca_poly_t out, const ca_poly_t p, slong n, ca_ctx_t ctx)
{
    ca_t zero, sum, term;
    ca_init(zero, ctx); ca_init(sum, ctx); ca_init(term, ctx);
    ca_poly_zero(out, ctx);
    if (n > 0)
    {
        ca_exp(sum, at(p, 0, zero), ctx); ca_poly_set_coeff_ca(out, 0, sum, ctx);
        for (slong k = 1; k < n; k++)
        {
            ca_zero(sum, ctx);
            for (slong i = 1; i < p->length && i <= k; i++)
            {
                ca_mul(term, p->coeffs + i, at(out, k - i, zero), ctx);
                ca_mul_ui(term, term, (ulong)i, ctx); ca_add(sum, sum, term, ctx);
            }
            ca_div_ui(sum, sum, (ulong)k, ctx); ca_poly_set_coeff_ca(out, k, sum, ctx);
        }
    }
    ca_clear(zero, ctx); ca_clear(sum, ctx); ca_clear(term, ctx);
}
static void logarithm(ca_poly_t out, const ca_poly_t p, slong n, ca_ctx_t ctx)
{
    ca_t zero, sum, term;
    ca_poly_t inv;
    ca_init(zero, ctx); ca_init(sum, ctx); ca_init(term, ctx); ca_poly_init(inv, ctx);
    ca_poly_zero(out, ctx); inverse(inv, p, n, ctx);
    if (n > 0)
    {
        ca_log(sum, p->coeffs, ctx); ca_poly_set_coeff_ca(out, 0, sum, ctx);
        for (slong k = 1; k < n; k++)
        {
            ca_zero(sum, ctx);
            for (slong i = 1; i < p->length && i <= k; i++)
            {
                ca_mul(term, p->coeffs + i, at(inv, k - i, zero), ctx);
                ca_mul_ui(term, term, (ulong)i, ctx); ca_add(sum, sum, term, ctx);
            }
            ca_div_ui(sum, sum, (ulong)k, ctx); ca_poly_set_coeff_ca(out, k, sum, ctx);
        }
    }
    ca_clear(zero, ctx); ca_clear(sum, ctx); ca_clear(term, ctx); ca_poly_clear(inv, ctx);
}
int main(void)
{
    const slong lengths[] = {1, 2, 4, 8, 9}, orders[] = {0, 1, 2, 3, 8, 15, 17};
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    ca_t quadratic, quartic, other, c;
    ca_init(quadratic, ctx); ca_init(quartic, ctx); ca_init(other, ctx); ca_init(c, ctx);
    ca_sqrt_ui(quadratic, 2, ctx); ca_sqrt_ui(other, 3, ctx);
    qqbar_t a2, a3;
    qqbar_init(a2); qqbar_init(a3);
    qqbar_sqrt_ui(a2, 2); qqbar_sqrt_ui(a3, 3); qqbar_add(a2, a2, a3);
    ca_set_qqbar(quartic, a2, ctx); qqbar_clear(a2); qqbar_clear(a3);
    int cases = 0;
    puts("mode,length,constant,order,checks,failures");
    for (int mode = 0; mode < 4; mode++) for (int li = 0; li < 5; li++)
    for (int constant = 1; constant <= 2; constant++) for (int ni = 0; ni < 7; ni++)
    {
        const slong n = orders[ni];
        ca_poly_t p, a, inv, ref, actual, alias, next;
        ca_poly_init(p, ctx); ca_poly_init(a, ctx); ca_poly_init(inv, ctx); ca_poly_init(ref, ctx);
        ca_poly_init(actual, ctx); ca_poly_init(alias, ctx); ca_poly_init(next, ctx);
        input(p, mode, lengths[li], constant, quadratic, quartic, other, ctx);
        ca_poly_set(a, p, ctx); ca_set_si(c, 3, ctx); ca_poly_set_coeff_ca(a, 0, c, ctx);
        int before_checks = checks, before_failures = failures;
        inverse(inv, p, n, ctx);
        ca_poly_inv_series(actual, p, n, ctx); equal(actual, inv, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_inv_series(alias, alias, n, ctx); equal(alias, inv, ctx);
        product(ref, a, inv, n, ctx);
        ca_poly_div_series(actual, a, p, n, ctx); equal(actual, ref, ctx);
        ca_poly_set(alias, a, ctx); ca_poly_div_series(alias, alias, p, n, ctx); equal(alias, ref, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_div_series(alias, a, alias, n, ctx); equal(alias, ref, ctx);
        logarithm(ref, p, n, ctx);
        ca_poly_log_series(actual, p, n, ctx); equal(actual, ref, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_log_series(alias, alias, n, ctx); equal(alias, ref, ctx);
        // Zero and nonzero constants qualify both exp(h-h0) normalization paths.
        ca_set_si(c, constant - 1, ctx); ca_poly_set_coeff_ca(p, 0, c, ctx);
        exponential(ref, p, n, ctx);
        ca_poly_exp_series(actual, p, n, ctx); equal(actual, ref, ctx);
        ca_poly_set(alias, p, ctx); ca_poly_exp_series(alias, alias, n, ctx); equal(alias, ref, ctx);
        for (ulong exponent = 0; exponent <= 5; exponent++)
        {
            if (n > 0) ca_poly_one(ref, ctx); else ca_poly_zero(ref, ctx);
            for (ulong e = 0; e < exponent; e++)
            {
                product(next, ref, p, n, ctx); ca_poly_swap(ref, next, ctx);
            }
            ca_poly_pow_ui_trunc(actual, p, exponent, n, ctx); equal(actual, ref, ctx);
            ca_poly_set(alias, p, ctx); ca_poly_pow_ui_trunc(alias, alias, exponent, n, ctx); equal(alias, ref, ctx);
        }
        printf("%d,%ld,%d,%ld,%d,%d\n", mode, (long)lengths[li], constant, (long)n, checks - before_checks, failures - before_failures);
        cases++;
        ca_poly_clear(p, ctx); ca_poly_clear(a, ctx); ca_poly_clear(inv, ctx); ca_poly_clear(ref, ctx);
        ca_poly_clear(actual, ctx); ca_poly_clear(alias, ctx); ca_poly_clear(next, ctx);
    }
    ca_clear(quadratic, ctx); ca_clear(quartic, ctx); ca_clear(other, ctx); ca_clear(c, ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"polynomial-series\",\"cases\":%d,\"checks\":%d,\"failures\":%d}\n", cases, checks, failures);
    return failures != 0;
}
