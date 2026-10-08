/* Check independently specified small polynomial values, including stale output. */
#include <stdio.h>
#include "ca.h"
#include "fmpz_poly.h"
#include "fmpq_poly.h"

static const char *outcome(truth_t t)
{
    return t == T_TRUE ? "Equal" : t == T_FALSE ? "NotEqual" : "Unknown";
}

int main(void)
{
    const char *inputs[] = {"two-thirds", "sqrt-two", "pi", "one-plus-i"};
    const char *states[] = {"zero", "seven", "unknown", "in-place"};
    ca_ctx_t ctx;
    ca_t x, expected, square, actual, rational;
    fmpz_poly_t polynomial;
    fmpq_poly_t qpolynomial;
    ca_ctx_init(ctx);
    ctx->options[CA_OPT_PREC_LIMIT] = 256;
    ca_init(x, ctx); ca_init(expected, ctx); ca_init(square, ctx);
    ca_init(actual, ctx); ca_init(rational, ctx);
    fmpz_poly_init(polynomial); fmpq_poly_init(qpolynomial);
    int integer_failed = 0, rational_failed = 0;
    puts("polynomial,input,state,integer_result,rational_result");
    for (int p = 0; p < 4; p++) for (int input = 0; input < 4; input++) for (int state = 0; state < 4; state++)
    {
        switch (input)
        {
            case 0: ca_set_ui(x, 2, ctx); ca_div_ui(x, x, 3, ctx); break;
            case 1: ca_set_ui(x, 2, ctx); ca_sqrt(x, x, ctx); break;
            case 2: ca_pi(x, ctx); break;
            default: ca_set_d_d(x, 1.0, 1.0, ctx); break;
        }
        fmpz_poly_zero(polynomial);
        if (p == 0) ca_zero(expected, ctx);
        if (p == 1)
        {
            fmpz_poly_set_coeff_si(polynomial, 0, 5);
            ca_set_ui(expected, 5, ctx);
        }
        if (p == 2)
        {
            // 1 + 2*x, using ordinary scalar operations, not polynomial evaluation.
            fmpz_poly_set_coeff_si(polynomial, 0, 1);
            fmpz_poly_set_coeff_si(polynomial, 1, 2);
            ca_mul_ui(expected, x, 2, ctx); ca_add_ui(expected, expected, 1, ctx);
        }
        if (p == 3)
        {
            // 5 - 2*x + 3*x*x, independent expanded expression.
            fmpz_poly_set_coeff_si(polynomial, 0, 5);
            fmpz_poly_set_coeff_si(polynomial, 1, -2);
            fmpz_poly_set_coeff_si(polynomial, 2, 3);
            ca_mul(square, x, x, ctx); ca_mul_ui(square, square, 3, ctx);
            ca_mul_ui(expected, x, 2, ctx); ca_sub(expected, square, expected, ctx);
            ca_add_ui(expected, expected, 5, ctx);
        }
        fmpq_poly_set_fmpz_poly(qpolynomial, polynomial);
        if (state == 0) ca_zero(actual, ctx);
        else if (state == 1) ca_set_ui(actual, 7, ctx);
        else if (state == 2) ca_unknown(actual, ctx);
        else ca_set(actual, x, ctx);
        ca_set(rational, actual, ctx);
        ca_fmpz_poly_evaluate(actual, polynomial, state == 3 ? actual : x, ctx);
        ca_fmpq_poly_evaluate(rational, qpolynomial, state == 3 ? rational : x, ctx);
        truth_t integer_equal = ca_check_equal(actual, expected, ctx);
        truth_t rational_equal = ca_check_equal(rational, expected, ctx);
        integer_failed += integer_equal != T_TRUE;
        rational_failed += rational_equal != T_TRUE;
        printf("%d,%s,%s,%s,%s\n", p, inputs[input], states[state], outcome(integer_equal), outcome(rational_equal));
    }
    printf("{\"suite\":\"polynomial-output\",\"cases\":64,\"integer_failed\":%d,\"rational_failed\":%d}\n", integer_failed, rational_failed);
    ca_clear(x, ctx); ca_clear(expected, ctx); ca_clear(square, ctx);
    ca_clear(actual, ctx); ca_clear(rational, ctx);
    fmpz_poly_clear(polynomial); fmpq_poly_clear(qpolynomial);
    ca_ctx_clear(ctx); flint_cleanup();
    return integer_failed || rational_failed ? 1 : 0;
}
