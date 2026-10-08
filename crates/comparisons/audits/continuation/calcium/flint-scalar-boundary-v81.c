/* Reuse only the frozen polynomial/endpoint serializers; discard its old main. */
#define main unused_inverse_corpus_v80
#include "flint-qqbar-inverse-v80.c"
#undef main
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <float.h>
#include "fmpq.h"
_Static_assert(sizeof(double)==8 && DBL_MANT_DIG==53 && DBL_MAX_EXP==1024 && FLT_RADIX==2, "binary64 required");
_Static_assert(sizeof(float)==4 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128, "binary32 required");

static unsigned long count81;
static void integer81(const fmpz_t x)
{
    char *s=fmpz_get_str(NULL,10,x); printf("\"%s\"",s); flint_free(s);
}
static double bits81(uint64_t bits)
{
    double x; memcpy(&x,&bits,8); return x;
}
static void value81(const qqbar_t x, slong precision)
{
    acb_t z; acb_init(z); qqbar_enclosure_raw(z,x,precision);
    printf(",\"poly\":"); polynomial(QQBAR_POLY(x));
    printf(",\"real\":"); endpoints(acb_realref(z));
    printf(",\"imag\":"); endpoints(acb_imagref(z)); acb_clear(z);
}
static void scalar_imports81(void)
{
    qqbar_t x; int width,sign,ok; unsigned exp,k;
    qqbar_init(x);
    for(width=32;width<=64;width+=32)
        for(sign=0;sign<2;sign++)
            for(exp=0;exp<(width==32?256u:2048u);exp++)
                for(k=0;k<4;k++)
                {
                    unsigned fraction_bits=width==32?23:52;
                    uint64_t fractions[4]={0,1,UINT64_C(1)<<(fraction_bits-1),(UINT64_C(1)<<fraction_bits)-1};
                    uint64_t bits=((uint64_t)sign<<(width-1))|((uint64_t)exp<<fraction_bits)|fractions[k];
                    double input;
                    if(width==32){uint32_t b=(uint32_t)bits;float f;memcpy(&f,&b,4);input=(double)f;}
                    else input=bits81(bits);
                    qqbar_set_si(x,17); ok=qqbar_set_d(x,input);
                    printf("{\"family\":\"float\",\"width\":%d,\"bits\":\"%016" PRIx64 "\",\"ok\":%d",width,bits,ok);
                    if(ok)value81(x,128);
                    puts("}"); count81++;
                }
    qqbar_clear(x);
}
static void complex_imports81(void)
{
    static const uint64_t bits[]={UINT64_C(0),UINT64_C(0x8000000000000000),UINT64_C(1),UINT64_C(0x8000000000000001),
        UINT64_C(0x000fffffffffffff),UINT64_C(0x0010000000000000),UINT64_C(0x3fd5555555555555),UINT64_C(0xbfe0000000000000),
        UINT64_C(0x3ff0000000000000),UINT64_C(0x7fefffffffffffff),UINT64_C(0xffefffffffffffff),UINT64_C(0x7ff0000000000000),
        UINT64_C(0xfff0000000000000),UINT64_C(0x7ff8000000000000),UINT64_C(0x7ff0000000000001),UINT64_C(0xfff8000000001234)};
    qqbar_t x; size_t i,j;int ok;qqbar_init(x);
    for(i=0;i<16;i++)for(j=0;j<16;j++){
        qqbar_set_si(x,-19);ok=qqbar_set_re_im_d(x,bits81(bits[i]),bits81(bits[j]));
        printf("{\"family\":\"complex-float\",\"i\":%zu,\"j\":%zu,\"ok\":%d",i,j,ok);
        if(ok){value81(x,128);}puts("}");count81++;
    }
    qqbar_clear(x);
}
static void inverses81(void)
{
    static const slong shifts[]={-3,0,5};qqbar_t x;int op,k,scale,phase,exists,ok;size_t shift;
    qqbar_init(x);
    for(op=0;op<2;op++)for(k=0;k<24;k++)for(shift=0;shift<3;shift++)for(scale=1;scale<=3;scale+=2){
        exists=op?qqbar_csc_pi(x,(k+24*shifts[shift])*scale,12*scale):qqbar_sec_pi(x,(k+24*shifts[shift])*scale,12*scale);
        for(phase=0;phase<2;phase++){
            slong p;ulong q;
            printf("{\"family\":\"inverse\",\"op\":%d,\"k\":%d,\"shift\":%ld,\"scale\":%d,\"phase\":%d,\"exists\":%d",op,k,(long)shifts[shift],scale,phase,exists);
            if(exists){
                if(phase)qqbar_cache_enclosure(x,256);
                ok=op?qqbar_acsc_pi(&p,&q,x):qqbar_asec_pi(&p,&q,x);
                value81(x,128);printf(",\"recognized\":%d",ok);
                if(ok)printf(",\"p\":%ld,\"q\":%lu",(long)p,(unsigned long)q);
            }
            puts("}");count81++;
        }
    }
    for(op=0;op<2;op++)for(k=0;k<6;k++){
        slong p;ulong q;
        if(k==5)qqbar_i(x);else{qqbar_set_si(x,k==0?0:k==1?1:k==2?-1:k==3?1:-1);if(k>=3)qqbar_mul_2exp_si(x,x,-1);}
        ok=op?qqbar_acsc_pi(&p,&q,x):qqbar_asec_pi(&p,&q,x);
        printf("{\"family\":\"inverse-control\",\"op\":%d,\"k\":%d,\"recognized\":%d",op,k,ok);
        value81(x,128);if(ok)printf(",\"p\":%ld,\"q\":%lu",(long)p,(unsigned long)q);
        puts("}");count81++;
    }
    qqbar_clear(x);
}
static void rounding81(void)
{
    static const slong exponents[]={0,8,128,1024};qqbar_t x,t,im,numerator;fmpz_t base,floor,ceil,den,height;
    int b,kind,imag,phase;qqbar_init(x);qqbar_init(t);qqbar_init(im);qqbar_init(numerator);
    fmpz_init(base);fmpz_init(floor);fmpz_init(ceil);fmpz_init(den);fmpz_init(height);
    for(b=0;b<7;b++)for(kind=0;kind<23;kind++)for(imag=0;imag<2;imag++){
        if(b==0||b==6){fmpz_one(base);fmpz_mul_2exp(base,base,256);if(b==0)fmpz_neg(base,base);}
        else fmpz_set_si(base,(slong[]){0,-3,-1,0,1,3,0}[b]);
        qqbar_set_fmpz(x,base);
        if(kind){
            int sign=(kind%2)?1:-1;slong e;
            if(kind<=8){qqbar_one(t);e=(slong[]){1,8,128,1024}[(kind-1)/2];}
            else{qqbar_sqrt_ui(t,2);e=kind<=16?exponents[(kind-9)/2]:exponents[1+(kind-17)/2];}
            qqbar_mul_2exp_si(t,t,-e);if(sign<0)qqbar_neg(t,t);qqbar_add(x,x,t);
            if(kind>=17){qqbar_one(t);qqbar_mul_2exp_si(t,t,-1);qqbar_add(x,x,t);}
        }
        if(imag){qqbar_sqrt_ui(im,3);qqbar_i(t);qqbar_mul(im,im,t);qqbar_add(x,x,im);}
        for(phase=0;phase<2;phase++){
            if(phase)qqbar_cache_enclosure(x,1536);
            qqbar_floor(floor,x);qqbar_ceil(ceil,x);qqbar_denominator(den,x);qqbar_height(height,x);
            printf("{\"family\":\"round\",\"b\":%d,\"kind\":%d,\"imaginary\":%d,\"phase\":%d,\"floor\":",b,kind,imag,phase);
            integer81(floor);printf(",\"ceil\":");integer81(ceil);value81(x,2048);
            printf(",\"denominator\":");integer81(den);printf(",\"height\":");integer81(height);
            printf(",\"heightBits\":%ld",(long)qqbar_height_bits(x));
            if(phase){qqbar_set(numerator,x);qqbar_numerator(numerator,numerator);}else qqbar_numerator(numerator,x);
            printf(",\"numeratorPoly\":");polynomial(QQBAR_POLY(numerator));puts("}");count81++;
        }
    }
    qqbar_phi(x);qqbar_set_si(t,-3);qqbar_swap(x,t);
    printf("{\"family\":\"swap-integer\"");value81(x,128);qqbar_get_fmpz(base,x);printf(",\"integer\":");integer81(base);puts("}");count81++;
    qqbar_swap(x,t);printf("{\"family\":\"phi\"");value81(x,128);puts("}");count81++;
    qqbar_set_si(x,-7);qqbar_set_si(t,3);qqbar_div(x,x,t);
    {fmpq_t r;fmpq_init(r);qqbar_get_fmpq(r,x);printf("{\"family\":\"rational-extraction\",\"num\":");integer81(fmpq_numref(r));printf(",\"den\":");integer81(fmpq_denref(r));puts("}");fmpq_clear(r);count81++;}
    qqbar_clear(x);qqbar_clear(t);qqbar_clear(im);qqbar_clear(numerator);fmpz_clear(base);fmpz_clear(floor);fmpz_clear(ceil);fmpz_clear(den);fmpz_clear(height);
}
int main(void)
{
    scalar_imports81();complex_imports81();inverses81();rounding81();
    printf("{\"terminal\":true,\"rows\":%lu}\n",count81);flint_cleanup();return 0;
}
