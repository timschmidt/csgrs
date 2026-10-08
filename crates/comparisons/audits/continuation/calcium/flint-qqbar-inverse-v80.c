/* Bounded rational-angle recognition audit; no failed-output inspection. */
#include <stdio.h>
#include <stdlib.h>
#include "fmpz.h"
#include "fmpz_poly.h"
#include "ulong_extras.h"
#include "qqbar.h"
#include "qqbar/impl.h"

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
static int forward(qqbar_t x, int op, slong p, ulong q)
{
    switch (op)
    {
        case 0: qqbar_sin_pi(x,p,q); return 1;
        case 1: qqbar_cos_pi(x,p,q); return 1;
        case 2: return qqbar_tan_pi(x,p,q);
        case 3: return qqbar_cot_pi(x,p,q);
        case 4: qqbar_exp_pi_i(x,p,q); return 1;
        default: abort();
    }
}
static int inverse(slong *p, ulong *q, const qqbar_t x, int op)
{
    switch (op)
    {
        case 0: return qqbar_asin_pi(p,q,x);
        case 1: return qqbar_acos_pi(p,q,x);
        case 2: return qqbar_atan_pi(p,q,x);
        case 3: return qqbar_acot_pi(p,q,x);
        case 4: return qqbar_log_pi_i(p,q,x);
        default: abort();
    }
}
static void proposal(const qqbar_t x, int op)
{
    qqbar_t t;
    arb_t z, pi;
    slong p;
    ulong q, g;
    int overlap;
    qqbar_init(t); arb_init(z); arb_init(pi);
    if (op == 3) qqbar_inv(t,x); else qqbar_set(t,x);
    qqbar_get_arb(z,t,64);
    if (op < 2) arb_asin(z,z,64); else arb_atan(z,z,64);
    arb_const_pi(pi,64); arb_div(z,z,pi,64);
    /* Only the known finite, interior near-match cases enter this helper. */
    if (!arb_is_finite(z)) abort();
    best_rational_fast(&p,&q,arf_get_d(arb_midref(z),ARF_RND_NEAR),1000000);
    arb_mul_ui(z,z,q,64); overlap = arb_contains_si(z,p);
    if (op == 1) { p = (slong)q - 2*p; q *= 2; }
    g = n_gcd(FLINT_ABS(p),q); p /= (slong)g; q /= g;
    printf(",\"proposal\":{\"p\":%ld,\"q\":%lu,\"overlap\":%d}",(long)p,(unsigned long)q,overlap);
    qqbar_clear(t); arb_clear(z); arb_clear(pi);
}
static void emit(unsigned long id, qqbar_t x, int op, int exists, int near)
{
    int phase;
    acb_t z;
    acb_init(z);
    for (phase = 0; phase < 2; phase++)
    {
        printf("{\"id\":%lu,\"phase\":%d,\"exists\":%d",id,phase,exists);
        if (exists)
        {
            slong p;
            ulong q;
            int ok;
            if (phase) qqbar_cache_enclosure(x,256);
            qqbar_enclosure_raw(z,x,128);
            printf(",\"poly\":"); polynomial(QQBAR_POLY(x));
            printf(",\"real\":"); endpoints(acb_realref(z));
            printf(",\"imag\":"); endpoints(acb_imagref(z));
            ok = inverse(&p,&q,x,op);
            printf(",\"recognized\":%d",ok);
            if (ok) printf(",\"p\":%ld,\"q\":%lu",(long)p,(unsigned long)q);
            if (near) proposal(x,op);
        }
        puts("}"); rows++;
    }
    acb_clear(z);
}
int main(void)
{
    const slong shifts[] = {-3,0,5};
    qqbar_t x, t;
    unsigned long id = 0;
    int op, k, which, sign, scale;
    size_t shift;
    qqbar_init(x); qqbar_init(t);
    for (op = 0; op < 5; op++)
    {
        ulong q = (op == 2 || op == 3) ? 24 : 12;
        for (k = 0; k < 2*(int)q; k++)
            for (shift = 0; shift < 3; shift++)
                for (scale = 1; scale <= 3; scale += 2)
                {
                    int exists = forward(x,op,(k+2*(slong)q*shifts[shift])*scale,q*(ulong)scale);
                    emit(id++,x,op,exists,0);
                }
    }
    for (which = 0; which < 4; which++)
        for (op = 0; op < 2; op++)
        {
            qqbar_sqrt_ui(x,5); qqbar_add_si(x,x,which % 2 ? 1 : -1);
            qqbar_mul_2exp_si(x,x,-2); if (which >= 2) qqbar_neg(x,x);
            emit(id++,x,op,1,0);
        }
    for (which = 0; which < 6; which++)
        for (op = 0; op < 2; op++)
        {
            qqbar_cos_pi(x,2*(which % 3+1),7); if (which >= 3) qqbar_neg(x,x);
            emit(id++,x,op,1,0);
        }
    for (op = 0; op < 4; op++)
        for (sign = -1; sign <= 1; sign += 2)
        {
            if (!forward(x,op,1,op < 2 ? 12 : 24)) abort();
            qqbar_set_si(t,sign); qqbar_mul_2exp_si(t,t,-100); qqbar_add(x,x,t);
            emit(id++,x,op,1,1);
        }
    for (which = 0; which < 9; which++)
        for (op = 0; op < 5; op++)
        {
            switch (which)
            {
                case 0: qqbar_zero(x); break;
                case 1: qqbar_one(x); break;
                case 2: qqbar_set_si(x,-1); break;
                case 3: qqbar_set_si(x,2); break;
                case 4: qqbar_set_si(x,-2); break;
                case 5: qqbar_one(x); qqbar_mul_2exp_si(x,x,-1); break;
                case 6: qqbar_set_si(x,3); qqbar_inv(x,x); break;
                case 7: qqbar_i(x); break;
                case 8: qqbar_sqrt_ui(x,2); qqbar_mul_2exp_si(x,x,-2); break;
                default: abort();
            }
            emit(id++,x,op,1,0);
        }
    if (id != 1081 || rows != 2162) abort();
    printf("{\"terminal\":true,\"cases\":%lu,\"rows\":%lu}\n",id,rows);
    qqbar_clear(x); qqbar_clear(t); flint_cleanup(); return 0;
}
