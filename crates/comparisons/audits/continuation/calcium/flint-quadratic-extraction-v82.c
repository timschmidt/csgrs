/* Public degree-1/2 extraction; distinct output integers, no invalid aliases. */
#include <stdio.h>
#include <stdlib.h>
#include "qqbar.h"

static void integer(const fmpz_t n)
{
    char *s = fmpz_get_str(NULL, 10, n);
    printf("\"%s\"", s);
    flint_free(s);
}
static void polynomial(const fmpz_poly_t p)
{
    slong i;
    putchar('[');
    for (i = 0; i < p->length; i++)
    {
        if (i) putchar(',');
        integer(p->coeffs + i);
    }
    putchar(']');
}
static void endpoints(const arb_t x)
{
    fmpz_t lo, hi, e;
    fmpz_init(lo); fmpz_init(hi); fmpz_init(e);
    arb_get_interval_fmpz_2exp(lo, hi, e, x);
    putchar('['); integer(lo); putchar(','); integer(hi); putchar(','); integer(e); putchar(']');
    fmpz_clear(lo); fmpz_clear(hi); fmpz_clear(e);
}
static void components(qqbar_t x, const char *prefix)
{
    acb_t z;
    acb_init(z);
    qqbar_enclosure_raw(z, x, 2048);
    printf(",\"%sReal\":", prefix); endpoints(acb_realref(z));
    printf(",\"%sImag\":", prefix); endpoints(acb_imagref(z));
    acb_clear(z);
}
int main(int argc, char **argv)
{
    FILE *input;
    unsigned long id, count = 0;
    int mode, phase, fields;
    char astr[4096], bstr[4096], cstr[4096], qstr[4096];
    qqbar_t original, x, y;
    fmpz_t a, b, c, q;
    acb_t z;
    if (argc != 2 || !(input = fopen(argv[1], "r"))) return 2;
    qqbar_init(original); qqbar_init(x); qqbar_init(y);
    fmpz_init(a); fmpz_init(b); fmpz_init(c); fmpz_init(q); acb_init(z);
    while ((fields = fscanf(input, "%lu %4095s %4095s %4095s %4095s", &id, astr, bstr, cstr, qstr)) == 5)
    {
        if (fmpz_set_str(a, astr, 10) || fmpz_set_str(b, bstr, 10) ||
            fmpz_set_str(c, cstr, 10) || fmpz_set_str(q, qstr, 10) || fmpz_sgn(q) <= 0) abort();
        qqbar_set_fmpz(original, c); qqbar_sqrt(original, original);
        qqbar_mul_fmpz(original, original, b); qqbar_add_fmpz(original, original, a); qqbar_div_fmpz(original, original, q);
        for (mode = 0; mode < 3; mode++)
        {
            qqbar_set(x, original);
            for (phase = 0; phase < 2; phase++)
            {
                if (phase) qqbar_cache_enclosure(x, 1536);
                qqbar_get_quadratic(a, b, c, q, x, mode);
                printf("{\"id\":%lu,\"mode\":%d,\"phase\":%d,\"a\":", id, mode, phase);
                integer(a); printf(",\"b\":"); integer(b); printf(",\"c\":"); integer(c); printf(",\"q\":"); integer(q);
                printf(",\"poly\":"); polynomial(QQBAR_POLY(x));
                qqbar_enclosure_raw(z, x, 2048);
                printf(",\"real\":"); endpoints(acb_realref(z)); printf(",\"imag\":"); endpoints(acb_imagref(z));
                qqbar_set_fmpz(y, c); qqbar_sqrt(y, y); qqbar_mul_fmpz(y, y, b);
                qqbar_add_fmpz(y, y, a); qqbar_div_fmpz(y, y, q);
                printf(",\"reconstructedPoly\":"); polynomial(QQBAR_POLY(y));
                components(y, "reconstructed"); puts("}"); count++;
            }
        }
    }
    if (fields != EOF || ferror(input) || count != 4650) abort();
    fclose(input);
    qqbar_clear(original); qqbar_clear(x); qqbar_clear(y);
    fmpz_clear(a); fmpz_clear(b); fmpz_clear(c); fmpz_clear(q); acb_clear(z);
    flint_cleanup();
    printf("{\"terminal\":true,\"rows\":%lu}\n", count);
    return 0;
}
