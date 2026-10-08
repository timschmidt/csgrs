/* Public expression-reader diagnostics. Never inspect a failed output value. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fmpq.h"
#include "fexpr.h"
#include "fexpr_builtin.h"
#include "qqbar.h"

static void integer(const fmpz_t n)
{
    char *s = fmpz_get_str(NULL, 10, n);
    printf("\"%s\"", s); flint_free(s);
}
static void polynomial(const fmpz_poly_t p)
{
    slong i; putchar('[');
    for (i = 0; i < p->length; i++) { if (i) putchar(','); integer(p->coeffs + i); }
    putchar(']');
}
static void endpoints(const arb_t v)
{
    fmpz_t lo, hi, e; fmpz_init(lo); fmpz_init(hi); fmpz_init(e);
    arb_get_interval_fmpz_2exp(lo, hi, e, v);
    putchar('['); integer(lo); putchar(','); integer(hi); putchar(','); integer(e); putchar(']');
    fmpz_clear(lo); fmpz_clear(hi); fmpz_clear(e);
}
static void value(qqbar_t x)
{
    acb_t z; acb_init(z); qqbar_enclosure_raw(z, x, 256);
    printf(",\"poly\":"); polynomial(QQBAR_POLY(x));
    printf(",\"real\":"); endpoints(acb_realref(z)); printf(",\"imag\":"); endpoints(acb_imagref(z)); acb_clear(z);
}
static void angle_expr(fexpr_t out, int op, const fmpz_t p, ulong den, int has_pi)
{
    const slong funcs[] = {FEXPR_Sin,FEXPR_Cos,FEXPR_Tan,FEXPR_Cot,FEXPR_Sec,FEXPR_Csc,FEXPR_Exp};
    fexpr_t a,b,h,t; fexpr_init(a); fexpr_init(b); fexpr_init(h); fexpr_init(t);
    fexpr_set_fmpz(a,p); fexpr_set_ui(b,den); fexpr_div(t,a,b);
    if (has_pi) { fexpr_set_symbol_builtin(b,FEXPR_Pi); fexpr_mul(a,t,b); fexpr_swap(t,a); }
    if (op==6) { fexpr_set_symbol_builtin(b,FEXPR_NumberI); fexpr_mul(a,t,b); fexpr_swap(t,a); }
    fexpr_set_symbol_builtin(h,funcs[op]); fexpr_call1(out,h,t);
    fexpr_clear(a); fexpr_clear(b); fexpr_clear(h); fexpr_clear(t);
}
static void angles(void)
{
    const ulong exponents[]={0,30,60,61,62,63,80,256};
    const slong initial[]={7,-11};
    int op,j,sign,si,has_pi,kind,ok; unsigned long rows=0;
    fmpz_t p; fexpr_t expr; qqbar_t x; fmpz_init(p); fexpr_init(expr); qqbar_init(x);
    for (op=0;op<7;op++) for (j=0;j<8;j++) for(sign=-1;sign<=1;sign+=2) for(si=0;si<2;si++)
    {
        ulong den=(op==3||op==5)?2:1;
        fmpz_one(p); fmpz_mul_2exp(p,p,exponents[j]); if(den==2) fmpz_add_ui(p,p,1); if(sign<0) fmpz_neg(p,p);
        angle_expr(expr,op,p,den,1); qqbar_set_si(x,initial[si]); ok=qqbar_set_fexpr(x,expr);
        printf("{\"family\":\"large-angle\",\"op\":%d,\"exponent\":%lu,\"sign\":%d,\"initial\":%ld,\"den\":%lu,\"ok\":%d",op,exponents[j],sign,(long)initial[si],den,ok);
        if(ok) value(x);
        puts("}"); rows++;
    }
    for(op=0;op<7;op++)for(kind=0;kind<3;kind++)for(sign=-1;sign<=1;sign+=2)for(has_pi=0;has_pi<2;has_pi++)
    {
        ulong den=kind==2?6:1; fmpz_set_si(p,kind==0?0:sign); angle_expr(expr,op,p,den,has_pi);
        qqbar_set_si(x,7);ok=qqbar_set_fexpr(x,expr);
        printf("{\"family\":\"pi-factor\",\"op\":%d,\"kind\":%d,\"sign\":%d,\"hasPi\":%d,\"ok\":%d",op,kind,sign,has_pi,ok);
        if(ok) value(x);
        puts("}");rows++;
    }
    qqbar_clear(x);fexpr_clear(expr);fmpz_clear(p);flint_cleanup();
    if(rows!=308) abort();
    printf("{\"terminal\":true,\"rows\":%lu}\n",rows);
}
static void powers(int which, int count)
{
    int j,ok; fexpr_t base,exponent,expr,tmp,den; fmpz_t n; qqbar_t x;
    fexpr_init(base);fexpr_init(exponent);fexpr_init(expr);fexpr_init(tmp);fexpr_init(den);fmpz_init(n);qqbar_init(x);
    fexpr_set_ui(base,1);fmpz_one(n);fmpz_mul_2exp(n,n,80);
    if(which==0) fexpr_set_ui(exponent,2);
    else if(which==1) fexpr_set_fmpz(exponent,n);
    else {fexpr_set_ui(tmp,1);fexpr_set_fmpz(den,n);fexpr_div(exponent,tmp,den);}
    fexpr_pow(expr,base,exponent);
    for(j=0;j<count;j++){
        qqbar_set_si(x,7);ok=qqbar_set_fexpr(x,expr);
        printf("{\"family\":\"power\",\"which\":%d,\"iteration\":%d,\"ok\":%d",which,j,ok);
        if(ok) value(x);
        puts("}");
    }
    qqbar_clear(x);fexpr_clear(base);fexpr_clear(exponent);fexpr_clear(expr);fexpr_clear(tmp);fexpr_clear(den);fmpz_clear(n);flint_cleanup();
    printf("{\"terminal\":true,\"rows\":%d}\n",count);
}
int main(int argc,char **argv)
{
    if(argc==2&&!strcmp(argv[1],"angles")){angles();return 0;}
    if(argc==4&&!strcmp(argv[1],"powers")){
        int which=atoi(argv[2]),count=atoi(argv[3]);if(which<0||which>2||count<1||count>256)return 2;
        powers(which,count);return 0;
    }
    return 2;
}
