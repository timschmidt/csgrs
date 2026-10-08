/* Independent finite complex controls. Approximate residuals, not enclosures.
   No invalid overlap, nonfinite input, exponent-limit or assertion testing. */
#include <fenv.h>
#include <stdio.h>
#include <stdlib.h>
#include "gr.h"
#include "arf.h"
#include "nfloat.h"
#include "fmpq.h"

static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
static unsigned long rows, failures, comparisons, aliases, skips;
static void require(int ok) { if (!ok) { fputs("control setup/domain failure\n",stderr); exit(2); } }

static void decode(mpq_t q, nfloat_srcptr x, gr_ctx_t ctx)
{
    if (NFLOAT_IS_ZERO(x)) { mpq_set_ui(q,0,1); return; }
    require(!NFLOAT_IS_SPECIAL(x) && NFLOAT_EXP(x)>-20000 && NFLOAT_EXP(x)<20000);
    require(NFLOAT_SGNBIT(x)<=1 && LIMB_MSB_IS_SET(NFLOAT_D(x)[NFLOAT_CTX_NLIMBS(ctx)-1]));
    mpz_import(mpq_numref(q),NFLOAT_CTX_NLIMBS(ctx),-1,sizeof(ulong),0,0,NFLOAT_D(x));
    mpz_set_ui(mpq_denref(q),1);
    slong shift=NFLOAT_EXP(x)-NFLOAT_CTX_PREC(ctx);
    if (shift>=0) mpz_mul_2exp(mpq_numref(q),mpq_numref(q),shift);
    else mpz_mul_2exp(mpq_denref(q),mpq_denref(q),-shift);
    if (NFLOAT_SGNBIT(x)) mpz_neg(mpq_numref(q),mpq_numref(q));
    mpq_canonicalize(q);
}

static void encode(nfloat_ptr x, const mpq_t q, gr_ctx_t ctx)
{
    fmpq_t f; fmpq_init(f); fmpq_set_mpq(f,q);
    require(nfloat_set_fmpq(x,f,ctx)==GR_SUCCESS); fmpq_clear(f);
    mpq_t r; mpq_init(r); decode(r,x,ctx); require(mpq_equal(r,q)); mpq_clear(r);
}

static void scale(mpq_t q, slong shift)
{
    if (shift>=0) mpq_mul_2exp(q,q,shift); else mpq_div_2exp(q,q,-shift);
}

static void inputs(mpq_t *q, slong bits, int pattern)
{
    for (int i=0;i<4;i++) {
        mpz_set_ui(mpq_numref(q[i]),1); mpz_mul_2exp(mpq_numref(q[i]),mpq_numref(q[i]),bits);
        mpz_sub_ui(mpq_numref(q[i]),mpq_numref(q[i]),2*i+1);
        mpz_set_ui(mpq_denref(q[i]),1); mpz_mul_2exp(mpq_denref(q[i]),mpq_denref(q[i]),bits);
        if ((pattern>>(i%3))&1) mpz_neg(mpq_numref(q[i]),mpq_numref(q[i]));
        mpq_canonicalize(q[i]);
    }
    switch (pattern) {
        case 4: mpq_set_ui(q[0],0,1); mpq_set_ui(q[1],0,1); break;
        case 5: mpq_set_ui(q[2],0,1); mpq_set_ui(q[3],0,1); break;
        case 6: mpq_set_ui(q[1],0,1); break;
        case 7: mpq_set_ui(q[0],0,1); break;
        case 8: mpq_set_ui(q[3],0,1); break;
        case 9: mpq_set_ui(q[2],0,1); break;
        case 10: mpq_set_ui(q[1],0,1); mpq_set_ui(q[3],0,1); break;
        case 11: mpq_set_ui(q[0],0,1); mpq_set_ui(q[2],0,1); break;
        case 12: mpq_set_ui(q[1],0,1); mpq_set_ui(q[2],0,1); break;
        case 13: mpq_set_ui(q[0],0,1); mpq_set_ui(q[3],0,1); break;
        case 14: mpq_set(q[2],q[0]); mpq_set(q[3],q[1]); break;
        case 15: mpq_set(q[2],q[0]); mpq_neg(q[3],q[1]); break;
        case 16: mpq_set(q[1],q[0]); mpq_set(q[3],q[2]); break;
        case 17: mpq_neg(q[1],q[0]); mpq_set(q[3],q[2]); break;
        case 18: scale(q[1],-70); break;
        case 19: scale(q[0],40); scale(q[3],-80); break;
        case 20: mpq_set(q[2],q[0]); break;
        case 21: mpq_neg(q[2],q[0]); mpq_neg(q[3],q[1]); break;
        case 22: mpq_set_ui(q[0],3,1); mpq_set_ui(q[1],4,1); mpq_set_ui(q[2],5,1); mpq_set_ui(q[3],0,1); break;
        case 23: mpq_set_ui(q[0],0,1); mpq_set_ui(q[1],0,1); mpq_set_ui(q[2],0,1); mpq_set_ui(q[3],0,1); break;
        case 24: case 25: case 26: case 27: scale(q[0],20); scale(q[1],-60); break;
        case 28: case 29: case 30: case 31: scale(q[2],20); scale(q[3],-60); break;
    }
}

static void product(mpq_t *r, const mpq_t *x, const mpq_t *y)
{
    mpq_t t; mpq_init(t);
    mpq_mul(r[0],x[0],y[0]); mpq_mul(t,x[1],y[1]); mpq_sub(r[0],r[0],t);
    mpq_mul(r[1],x[0],y[1]); mpq_mul(t,x[1],y[0]); mpq_add(r[1],r[1],t);
    mpq_clear(t);
}

static void norm(mpq_t r, const mpq_t *x)
{
    mpq_t t; mpq_init(t); mpq_mul(r,x[0],x[0]); mpq_mul(t,x[1],x[1]); mpq_add(r,r,t); mpq_clear(t);
}

/* Corpus tolerance 64*2^-p*max(1,|expected.re|,|expected.im|).
   Root operations compare exact rational squared residuals, not rounded roots.
   This deliberately does not assert a general componentwise ULP guarantee. */
static int close_pair(const mpq_t *got, const mpq_t *expected, slong bits, int exact)
{
    mpq_t bound,t; mpq_inits(bound,t,NULL); mpq_set_ui(bound,1,1);
    for (int i=0;i<2;i++) { mpq_abs(t,expected[i]); if (mpq_cmp(t,bound)>0) mpq_set(bound,t); }
    scale(bound,6-bits); int ok=1;
    for (int i=0;i<2;i++) {
        mpq_sub(t,got[i],expected[i]); mpq_abs(t,t); comparisons++;
        ok &= exact ? mpq_sgn(t)==0 : mpq_cmp(t,bound)<=0;
    }
    mpq_clears(bound,t,NULL); return ok;
}

static void arithmetic(gr_ctx_t ctx, int pattern)
{
    mpq_t q[4],expected[2],got[2],residual[2],temp[2],baseline[2],denom;
    for (int i=0;i<4;i++) mpq_init(q[i]);
    for (int i=0;i<2;i++) mpq_inits(expected[i],got[i],residual[i],temp[i],baseline[i],NULL);
    mpq_init(denom); inputs(q,NFLOAT_CTX_PREC(ctx),pattern);
    for (int op=0;op<14;op++) {
        int xzero=!mpq_sgn(q[0])&&!mpq_sgn(q[1]), yzero=!mpq_sgn(q[2])&&!mpq_sgn(q[3]);
        if (((op==4||op==12)&&xzero)||(op==5&&yzero)) { skips+=3; continue; }
        for (int i=0;i<2;i++) mpq_set_ui(expected[i],0,1);
        switch (op) {
            case 0: for(int i=0;i<2;i++) mpq_add(expected[i],q[i],q[i+2]); break;
            case 1: for(int i=0;i<2;i++) mpq_sub(expected[i],q[i],q[i+2]); break;
            case 2: product(expected,q,q+2); break;
            case 3: case 13: product(expected,q,q); break;
            case 4: norm(denom,q); mpq_div(expected[0],q[0],denom); mpq_div(expected[1],q[1],denom); mpq_neg(expected[1],expected[1]); break;
            case 5: norm(denom,q+2); mpq_set(temp[0],q[2]); mpq_neg(temp[1],q[3]); product(expected,q,temp);
                    for(int i=0;i<2;i++) mpq_div(expected[i],expected[i],denom); break;
            case 6: mpq_set(expected[0],q[0]); mpq_neg(expected[1],q[1]); break;
            case 7: for(int i=0;i<2;i++) mpq_neg(expected[i],q[i]); break;
            case 8: mpq_set(expected[0],q[0]); break;
            case 9: mpq_set(expected[0],q[1]); break;
            case 10: norm(expected[0],q); break;
            case 11: for(int i=0;i<2;i++) mpq_set(expected[i],q[i]); break;
            case 12: mpq_set_ui(expected[0],1,1); break;
        }
        for(int alias=0;alias<3;alias++) {
            ulong x[2*NFLOAT_MAX_ALLOC]={0},y[2*NFLOAT_MAX_ALLOC]={0},z[2*NFLOAT_MAX_ALLOC]={0};
            encode(NFLOAT_COMPLEX_RE(x,ctx),q[0],ctx); encode(NFLOAT_COMPLEX_IM(x,ctx),q[1],ctx);
            encode(NFLOAT_COMPLEX_RE(y,ctx),q[2],ctx); encode(NFLOAT_COMPLEX_IM(y,ctx),q[3],ctx);
            nfloat_ptr out=alias==1?x:alias==2?y:z;
            require(fesetround(modes[pattern%4])==0); int status=GR_UNABLE;
            switch(op) {
                case 0: status=gr_add(out,x,y,ctx); break;
                case 1: status=gr_sub(out,x,y,ctx); break;
                case 2: status=gr_mul(out,x,y,ctx); break;
                case 3: status=gr_sqr(out,x,ctx); break;
                case 4: status=gr_inv(out,x,ctx); break;
                case 5: status=gr_div(out,x,y,ctx); break;
                case 6: status=gr_conj(out,x,ctx); break;
                case 7: status=gr_neg(out,x,ctx); break;
                case 8: status=gr_re(out,x,ctx); break;
                case 9: status=gr_im(out,x,ctx); break;
                case 10: status=gr_abs(out,x,ctx); break;
                case 11: status=gr_sqrt(out,x,ctx); break;
                case 12: status=gr_rsqrt(out,x,ctx); break;
                case 13: status=gr_mul(out,x,x,ctx); break;
            }
            int ok=0,alias_ok=0,branch=1;
            if(status==GR_SUCCESS) {
                decode(got[0],NFLOAT_COMPLEX_RE(out,ctx),ctx); decode(got[1],NFLOAT_COMPLEX_IM(out,ctx),ctx);
                if(!alias) { for(int i=0;i<2;i++) mpq_set(baseline[i],got[i]); alias_ok=1; }
                else { alias_ok=mpq_equal(baseline[0],got[0])&&mpq_equal(baseline[1],got[1]); aliases++; }
                if(op==10) { branch=mpq_sgn(got[0])>=0&&!mpq_sgn(got[1]); product(residual,got,got); }
                else if(op==11||op==12) {
                    branch=mpq_sgn(got[0])>=0;
                    if(mpq_sgn(q[1])) branch &= mpq_sgn(got[1])==(op==12?-mpq_sgn(q[1]):mpq_sgn(q[1]));
                    else if(mpq_sgn(q[0])<0) branch &= !mpq_sgn(got[0])&&mpq_sgn(got[1])==(op==12?-1:1);
                    else branch &= !mpq_sgn(got[1]);
                    product(temp,got,got);
                    if(op==12) product(residual,temp,q); else for(int i=0;i<2;i++) mpq_set(residual[i],temp[i]);
                } else for(int i=0;i<2;i++) mpq_set(residual[i],got[i]);
                ok=close_pair(residual,expected,NFLOAT_CTX_PREC(ctx),op>=6&&op<=9);
            }
            int fenv_ok=fegetround()==modes[pattern%4]; rows++; failures+=!ok||!alias_ok||!branch||!fenv_ok||status!=GR_SUCCESS;
            printf("arithmetic,%ld,%d,%d,%d,%d,%d,%d,%d,%d\n",NFLOAT_CTX_NLIMBS(ctx),pattern,op,alias,status,ok,alias_ok,branch,fenv_ok);
        }
    }
    norm(expected[0],q); norm(expected[1],q+2); int exact_cmp=mpq_cmp(expected[0],expected[1]); exact_cmp=(exact_cmp>0)-(exact_cmp<0);
    ulong x[2*NFLOAT_MAX_ALLOC]={0},y[2*NFLOAT_MAX_ALLOC]={0};
    encode(NFLOAT_COMPLEX_RE(x,ctx),q[0],ctx); encode(NFLOAT_COMPLEX_IM(x,ctx),q[1],ctx);
    encode(NFLOAT_COMPLEX_RE(y,ctx),q[2],ctx); encode(NFLOAT_COMPLEX_IM(y,ctx),q[3],ctx);
    int actual=99; int status=gr_cmpabs(&actual,x,y,ctx); int ok=status==GR_SUCCESS&&actual==exact_cmp;
    rows++; comparisons++; failures+=!ok; printf("cmpabs,%ld,%d,%d,%d,%d\n",NFLOAT_CTX_NLIMBS(ctx),pattern,status,exact_cmp,ok);
    for(int i=0;i<4;i++) mpq_clear(q[i]);
    for(int i=0;i<2;i++) mpq_clears(expected[i],got[i],residual[i],temp[i],baseline[i],NULL);
    mpq_clear(denom);
}

int main(void)
{
    int original=fegetround(); require(original!=-1);
    for(slong limbs=NFLOAT_MIN_LIMBS;limbs<=NFLOAT_MAX_LIMBS;limbs++) {
        gr_ctx_t ctx; require(nfloat_complex_ctx_init(ctx,limbs*FLINT_BITS,0)==GR_SUCCESS);
        for(int pattern=0;pattern<32;pattern++) arithmetic(ctx,pattern);
        gr_ctx_clear(ctx);
    }
    require(fesetround(original)==0); flint_cleanup_master();
    printf("{\"suite\":\"nfloat-complex-controls\",\"limb_bits\":%d,\"rows\":%lu,\"comparisons\":%lu,\"aliases\":%lu,\"skipped_zero_divisors\":%lu,\"failures\":%lu}\n",FLINT_BITS,rows,comparisons,aliases,skips,failures);
    return failures!=0;
}
