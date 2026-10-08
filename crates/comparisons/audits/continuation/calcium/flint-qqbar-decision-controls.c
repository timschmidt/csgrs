/* Bounded, valid algebraic values only. The independent oracle is JavaScript
   BigInt arithmetic, not another qqbar operation. No raw enclosure mutation,
   arbitrary serialized input, resource-boundary or failure-path probes. */
#include <stdio.h>
#include <stdlib.h>
#include <gmp.h>
#include "fmpz.h"
#include "fmpz_poly.h"
#include "qqbar.h"

enum { SIDE = 7, COUNT = SIDE * SIDE };

static void coordinate(qqbar_t out, int k)
{
    qqbar_sqrt_ui(out, (ulong) abs(k));
    if (k < 0)
        qqbar_neg(out, out);
}

static void endpoints(const char *name, const arb_t x)
{
    fmpz_t lo, hi, exp;
    char *l, *h, *e;
    fmpz_init(lo);
    fmpz_init(hi);
    fmpz_init(exp);
    arb_get_interval_fmpz_2exp(lo, hi, exp, x);
    l = fmpz_get_str(NULL, 10, lo);
    h = fmpz_get_str(NULL, 10, hi);
    e = fmpz_get_str(NULL, 10, exp);
    printf(",\"%s\":[\"%s\",\"%s\",\"%s\"]", name, l, h, e);
    flint_free(l);
    flint_free(h);
    flint_free(e);
    fmpz_clear(lo);
    fmpz_clear(hi);
    fmpz_clear(exp);
}

int main(void)
{
    qqbar_struct values[COUNT], saved[COUNT];
    qqbar_t re, im, copy;
    acb_t z;
    const slong precisions[] = {32, 128, 512};
    int i, j, phase, p;
    qqbar_init(re);
    qqbar_init(im);
    qqbar_init(copy);
    acb_init(z);
    for (i = 0; i < COUNT; i++)
    {
        qqbar_init(values + i);
        qqbar_init(saved + i);
        coordinate(re, i / SIDE - 3);
        coordinate(im, i % SIDE - 3);
        qqbar_set_re_im(values + i, re, im);
    }
    for (phase = 0; phase < 2; phase++)
    {
        if (phase)
            for (i = 0; i < COUNT; i++)
                qqbar_cache_enclosure(values + i, 256);
        for (i = 0; i < COUNT; i++)
            qqbar_set(saved + i, values + i);
        for (i = 0; i < COUNT; i++)
        {
            qqbar_set(copy, values + i);
            printf("{\"type\":\"unary\",\"phase\":%d,\"i\":%d,"
                   "\"re\":%d,\"im\":%d,\"csgn\":%d,\"copyEqual\":%d,"
                   "\"copyRootOrder\":%d,\"copyHashEqual\":%d}\n",
                   phase, i, qqbar_sgn_re(values + i), qqbar_sgn_im(values + i),
                   qqbar_csgn(values + i), qqbar_equal(values + i, copy),
                   qqbar_cmp_root_order(values + i, copy),
                   qqbar_hash(values + i) == qqbar_hash(copy));
            for (p = 0; p < 3; p++)
            {
                qqbar_get_acb(z, values + i, precisions[p]);
                printf("{\"type\":\"enclosure\",\"phase\":%d,\"i\":%d,"
                       "\"prec\":%ld,\"exactRe\":%d,\"exactIm\":%d,"
                       "\"accRe\":%ld,\"accIm\":%ld",
                       phase, i, (long) precisions[p], arb_is_exact(acb_realref(z)),
                       arb_is_exact(acb_imagref(z)),
                       (long) arb_rel_accuracy_bits(acb_realref(z)),
                       (long) arb_rel_accuracy_bits(acb_imagref(z)));
                endpoints("real", acb_realref(z));
                endpoints("imag", acb_imagref(z));
                puts("}");
            }
            for (j = 0; j < COUNT; j++)
            {
                printf("{\"type\":\"pair\",\"phase\":%d,\"i\":%d,\"j\":%d,"
                       "\"equal\":%d,\"re\":%d,\"im\":%d,\"absRe\":%d,"
                       "\"absIm\":%d,\"abs\":%d,\"rootOrder\":%d}\n",
                       phase, i, j, qqbar_equal(values + i, values + j),
                       qqbar_cmp_re(values + i, values + j),
                       qqbar_cmp_im(values + i, values + j),
                       qqbar_cmpabs_re(values + i, values + j),
                       qqbar_cmpabs_im(values + i, values + j),
                       qqbar_cmpabs(values + i, values + j),
                       qqbar_cmp_root_order(values + i, values + j));
            }
        }
        for (i = 0; i < COUNT; i++)
            printf("{\"type\":\"preserved\",\"phase\":%d,\"i\":%d,"
                   "\"polynomial\":%d,\"enclosure\":%d}\n", phase, i,
                   fmpz_poly_equal(QQBAR_POLY(values + i), QQBAR_POLY(saved + i)),
                   acb_equal(QQBAR_ENCLOSURE(values + i), QQBAR_ENCLOSURE(saved + i)));
    }
    for (i = 0; i < COUNT; i++)
    {
        qqbar_clear(values + i);
        qqbar_clear(saved + i);
    }
    qqbar_clear(re);
    qqbar_clear(im);
    qqbar_clear(copy);
    acb_clear(z);
    flint_cleanup();
    puts("{\"type\":\"terminal\",\"values\":49,\"phases\":2,\"pairs\":4802,\"enclosures\":294}");
    return 0;
}
