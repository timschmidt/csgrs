/* Small, valid real algebraic values in Q(sqrt(2), sqrt(3)).
   Collect full polynomials and dyadic root enclosures for an independent
   BigInt field oracle. No malformed objects, extreme sizes or crash probes. */
#include <stdio.h>
#include <stdlib.h>
#include <gmp.h>
#include "fmpz.h"
#include "fmpz_poly.h"
#include "fmpq.h"
#include "fmpq_poly.h"
#include "qqbar.h"

enum { COUNT = 16, POLYS = 5 };
static const long recipes[COUNT][5] = {
    {0,0,0,0,1}, {1,0,0,0,1}, {-1,0,0,0,1}, {1,0,0,0,2},
    {0,1,0,0,1}, {0,-1,0,0,1}, {0,0,1,0,1}, {0,0,0,1,1},
    {1,1,0,0,1}, {1,-1,0,0,1}, {0,1,1,0,1}, {0,1,-1,0,1},
    {1,1,1,1,1}, {-1,2,-1,1,3}, {2,-1,1,-2,5}, {0,0,-1,0,1}
};
static const long affine[3][3] = {{0,2,3}, {-2,1,3}, {3,-2,-5}};
static const long poly_coeffs[POLYS][6] = {
    {0,0,0,0,0,0}, {1,0,0,0,0,0}, {-4,3,0,0,0,0},
    {1,-2,0,1,0,0}, {1,-2,3,0,-1,2}
};
static const long poly_den[POLYS] = {1,3,2,1,3};
static const int poly_len[POLYS] = {0,1,2,4,6};
static unsigned long rows = 0;

static void polynomial(const fmpz_poly_t p)
{
    slong k;
    putchar('[');
    for (k = 0; k < p->length; k++)
    {
        char *s = fmpz_get_str(NULL, 10, p->coeffs + k);
        printf("%s\"%s\"", k ? "," : "", s);
        flint_free(s);
    }
    putchar(']');
}

static void endpoints(const arb_t x)
{
    fmpz_t lo, hi, exp;
    char *l, *h, *e;
    fmpz_init(lo); fmpz_init(hi); fmpz_init(exp);
    arb_get_interval_fmpz_2exp(lo, hi, exp, x);
    l = fmpz_get_str(NULL, 10, lo);
    h = fmpz_get_str(NULL, 10, hi);
    e = fmpz_get_str(NULL, 10, exp);
    printf("[\"%s\",\"%s\",\"%s\"]", l, h, e);
    flint_free(l); flint_free(h); flint_free(e);
    fmpz_clear(lo); fmpz_clear(hi); fmpz_clear(exp);
}

static void value(const qqbar_t x, int phase, int i, int j,
                  const char *op, int alias, int parameter)
{
    acb_t z;
    acb_init(z);
    qqbar_enclosure_raw(z, x, 128);
    printf("{\"type\":\"value\",\"phase\":%d,\"i\":%d,\"j\":%d,"
           "\"op\":\"%s\",\"alias\":%d,\"parameter\":%d,\"poly\":",
           phase, i, j, op, alias, parameter);
    polynomial(QQBAR_POLY(x));
    printf(",\"real\":"); endpoints(acb_realref(z));
    printf(",\"imagZero\":%d}\n", arb_is_zero(acb_imagref(z)));
    acb_clear(z);
    rows++;
}

static void binary(qqbar_t out, const qqbar_t x, const qqbar_t y, int op)
{
    switch (op)
    {
        case 0: qqbar_add(out, x, y); break;
        case 1: qqbar_sub(out, x, y); break;
        case 2: qqbar_mul(out, x, y); break;
        case 3: qqbar_div(out, x, y); break;
        default: abort();
    }
}

int main(void)
{
    qqbar_struct values[COUNT], saved[COUNT], basis[4];
    qqbar_t term, out, x, y;
    fmpz_poly_t composed;
    fmpq_poly_t polys[POLYS];
    fmpq_t coefficient;
    fmpz_t a, b, c;
    int i, j, k, op, alias, phase, parameter;
    const char *names[] = {"add", "sub", "mul", "div"};
    const unsigned long squares[] = {1,2,3,6};
    qqbar_init(term); qqbar_init(out); qqbar_init(x); qqbar_init(y);
    fmpz_poly_init(composed); fmpq_init(coefficient);
    fmpz_init(a); fmpz_init(b); fmpz_init(c);
    for (k = 0; k < 4; k++)
    {
        qqbar_init(basis + k);
        qqbar_sqrt_ui(basis + k, squares[k]);
    }
    for (i = 0; i < COUNT; i++)
    {
        qqbar_init(values + i); qqbar_init(saved + i);
        for (k = 0; k < 4; k++)
        {
            qqbar_mul_si(term, basis + k, recipes[i][k]);
            qqbar_add(values + i, values + i, term);
        }
        qqbar_div_si(values + i, values + i, recipes[i][4]);
    }
    for (k = 0; k < POLYS; k++)
    {
        fmpq_poly_init(polys[k]);
        for (j = 0; j < poly_len[k]; j++)
        {
            fmpq_set_si(coefficient, poly_coeffs[k][j], poly_den[k]);
            fmpq_poly_set_coeff_fmpq(polys[k], j, coefficient);
        }
    }
    for (phase = 0; phase < 2; phase++)
    {
        if (phase)
            for (i = 0; i < COUNT; i++) qqbar_cache_enclosure(values + i, 256);
        for (i = 0; i < COUNT; i++) qqbar_set(saved + i, values + i);
        for (i = 0; i < COUNT; i++)
        {
            value(values + i, phase, i, -1, "input", 0, 0);
            for (j = 0; j < COUNT; j++)
                for (op = 0; op < 4; op++)
                {
                    /* The recipe oracle, not a numerical zero test, excludes 0. */
                    if (op == 3 && j == 0) continue;
                    for (alias = 0; alias < 3; alias++)
                    {
                        qqbar_set(x, values + i); qqbar_set(y, values + j);
                        if (alias == 0) binary(out, x, y, op);
                        else if (alias == 1) { binary(x, x, y, op); qqbar_set(out, x); }
                        else { binary(y, x, y, op); qqbar_set(out, y); }
                        value(out, phase, i, j, names[op], alias, 0);
                    }
                    if (!phase)
                    {
                        qqbar_fmpz_poly_composed_op(composed, QQBAR_POLY(values + i),
                                                   QQBAR_POLY(values + j), op);
                        printf("{\"type\":\"composed\",\"i\":%d,\"j\":%d,\"op\":\"%s\",\"poly\":",
                               i, j, names[op]);
                        polynomial(composed); puts("}"); rows++;
                    }
                }
            for (alias = 0; alias < 2; alias++)
            {
                qqbar_ptr dest = alias ? x : out;
                if (i != 0)
                {
                    qqbar_set(x, values + i); qqbar_inv(dest, x);
                    value(dest, phase, i, -1, "inv", alias, 0);
                }
                for (parameter = -3; parameter <= 4; parameter++)
                {
                    if (i == 0 && parameter < 0) continue;
                    qqbar_set(x, values + i); qqbar_pow_si(dest, x, parameter);
                    value(dest, phase, i, -1, "pow", alias, parameter);
                }
                for (parameter = -3; parameter <= 3; parameter += 3)
                {
                    qqbar_set(x, values + i); qqbar_mul_2exp_si(dest, x, parameter);
                    value(dest, phase, i, -1, "scale", alias, parameter);
                }
                for (parameter = 0; parameter < 3; parameter++)
                {
                    fmpz_set_si(a, affine[parameter][0]);
                    fmpz_set_si(b, affine[parameter][1]);
                    fmpz_set_si(c, affine[parameter][2]);
                    qqbar_set(x, values + i); qqbar_scalar_op(dest, x, a, b, c);
                    value(dest, phase, i, -1, "affine", alias, parameter);
                }
                for (parameter = 0; parameter < POLYS; parameter++)
                {
                    qqbar_set(x, values + i);
                    qqbar_evaluate_fmpq_poly(dest, polys[parameter], x);
                    value(dest, phase, i, -1, "evaluate", alias, parameter);
                }
            }
            for (parameter = 0; parameter < POLYS; parameter++)
                for (j = 0; j < COUNT; j++)
                {
                    printf("{\"type\":\"relation\",\"phase\":%d,\"i\":%d,\"j\":%d,\"parameter\":%d,\"equal\":%d}\n",
                           phase, i, j, parameter,
                           qqbar_equal_fmpq_poly_val(values + j, polys[parameter], values + i));
                    rows++;
                }
        }
        for (i = 0; i < COUNT; i++)
        {
            printf("{\"type\":\"preserved\",\"phase\":%d,\"i\":%d,\"polynomial\":%d,\"enclosure\":%d}\n",
                   phase, i, fmpz_poly_equal(QQBAR_POLY(values + i), QQBAR_POLY(saved + i)),
                   acb_equal(QQBAR_ENCLOSURE(values + i), QQBAR_ENCLOSURE(saved + i)));
            rows++;
        }
    }
    for (i = 0; i < COUNT; i++) { qqbar_clear(values + i); qqbar_clear(saved + i); }
    for (k = 0; k < 4; k++) qqbar_clear(basis + k);
    for (k = 0; k < POLYS; k++) fmpq_poly_clear(polys[k]);
    qqbar_clear(term); qqbar_clear(out); qqbar_clear(x); qqbar_clear(y);
    fmpz_poly_clear(composed); fmpq_clear(coefficient);
    fmpz_clear(a); fmpz_clear(b); fmpz_clear(c);
    flint_cleanup();
    printf("{\"type\":\"terminal\",\"rowsBeforeTerminal\":%lu,\"values\":16,\"phases\":2}\n", rows);
    return 0;
}
