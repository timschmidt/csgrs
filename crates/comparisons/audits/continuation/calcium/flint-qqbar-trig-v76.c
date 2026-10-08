/* In-contract small rational angles; independent oracle is check-qqbar-trig-v76.mjs. */
#include <stdio.h>
#include <stdlib.h>
#include "fmpz.h"
#include "fmpz_poly.h"
#include "qqbar.h"

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
    l = fmpz_get_str(NULL, 10, lo); h = fmpz_get_str(NULL, 10, hi);
    e = fmpz_get_str(NULL, 10, exp);
    printf("[\"%s\",\"%s\",\"%s\"]", l, h, e);
    flint_free(l); flint_free(h); flint_free(e);
    fmpz_clear(lo); fmpz_clear(hi); fmpz_clear(exp);
}

static int construct(qqbar_t out, int op, slong p, ulong q)
{
    switch (op)
    {
        case 0: qqbar_root_of_unity(out,p,q); return 1;
        case 1: qqbar_exp_pi_i(out,p,q); return 1;
        case 2: qqbar_sin_pi(out,p,q); return 1;
        case 3: qqbar_cos_pi(out,p,q); return 1;
        case 4: return qqbar_tan_pi(out,p,q);
        case 5: return qqbar_cot_pi(out,p,q);
        case 6: return qqbar_sec_pi(out,p,q);
        case 7: return qqbar_csc_pi(out,p,q);
        default: abort();
    }
}

int main(void)
{
    const ulong denominators[] = {1,2,3,4,6,12};
    const char *names[] = {"root","exp","sin","cos","tan","cot","sec","csc"};
    qqbar_t out;
    acb_t z;
    size_t d;
    slong p;
    int scale, op, phase, ok, control;
    qqbar_init(out); acb_init(z);
    for (d = 0; d < sizeof(denominators)/sizeof(denominators[0]); d++)
    {
        ulong q = denominators[d];
        for (p = -2*(slong)q; p <= 2*(slong)q; p++)
            for (scale = 1; scale <= 3; scale += 2)
                for (op = 0; op < 8; op++)
                {
                    qqbar_set_si(out,17);
                    ok = construct(out,op,p*scale,q*(ulong)scale);
                    for (phase = 0; phase < 2; phase++)
                    {
                        printf("{\"type\":\"angle\",\"q\":%lu,\"p\":%ld,\"scale\":%d,\"op\":\"%s\",\"phase\":%d,\"ok\":%d",
                            (unsigned long)q,(long)p,scale,names[op],phase,ok);
                        if (ok)
                        {
                            slong rp = -99;
                            ulong rq = 99;
                            int recognized;
                            if (phase) qqbar_cache_enclosure(out,256);
                            qqbar_enclosure_raw(z,out,128);
                            printf(",\"poly\":"); polynomial(QQBAR_POLY(out));
                            printf(",\"real\":"); endpoints(acb_realref(z));
                            printf(",\"imag\":"); endpoints(acb_imagref(z));
                            if (op < 2)
                            {
                                recognized = qqbar_is_root_of_unity(&rp,&rq,out);
                                printf(",\"recognized\":%d,\"nullRecognized\":%d,\"rp\":%ld,\"rq\":%lu",
                                    recognized,qqbar_is_root_of_unity(NULL,NULL,out),(long)rp,(unsigned long)rq);
                            }
                        }
                        puts("}"); rows++;
                    }
                }
    }
    for (control = 0; control < 5; control++)
    {
        if (control == 0) qqbar_zero(out);
        if (control == 1) qqbar_set_si(out,2);
        if (control == 2) qqbar_set_si(out,-2);
        if (control == 3) { qqbar_one(out); qqbar_mul_2exp_si(out,out,-1); }
        if (control == 4) qqbar_sqrt_ui(out,2);
        for (phase = 0; phase < 2; phase++)
        {
            slong rp = -99;
            ulong rq = 99;
            if (phase) qqbar_cache_enclosure(out,256);
            printf("{\"type\":\"nonroot\",\"control\":%d,\"phase\":%d,\"recognized\":%d,\"nullRecognized\":%d}\n",
                control,phase,qqbar_is_root_of_unity(&rp,&rq,out),qqbar_is_root_of_unity(NULL,NULL,out));
            rows++;
        }
    }
    printf("{\"type\":\"terminal\",\"rowsBeforeTerminal\":%lu}\n",rows);
    qqbar_clear(out); acb_clear(z); flint_cleanup();
    return 0;
}
