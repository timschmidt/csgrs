/* Independent small-value control for instrumented binary64 rounding. */
#include <fenv.h>
#include <stdio.h>
#include <mpfr.h>
int main(void)
{
    const int modes[]={FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO};
    const mpfr_rnd_t rounds[]={MPFR_RNDN,MPFR_RNDD,MPFR_RNDU,MPFR_RNDZ};
    volatile double values[]={3.0,7.0,63.0},factor=1.0+1e-6;
    mpfr_t x,y,z;mpfr_init2(x,53);mpfr_init2(y,53);mpfr_init2(z,53);
    int original=fegetround(),failures=0;
    for(int mode=0;mode<4;mode++)for(int i=0;i<3;i++) {
        if(fesetround(modes[mode]))return 2;
        double a=values[i],b=factor;
        volatile double actual=a*b;
        mpfr_set_d(x,a,MPFR_RNDN);mpfr_set_d(y,b,MPFR_RNDN);
        mpfr_mul(z,x,y,rounds[mode]);double expected=mpfr_get_d(z,MPFR_RNDN);
        int same=actual==expected,preserved=fegetround()==modes[mode];
        failures+=!same||!preserved;
        printf("%d,%d,%a,%a,%d,%d\n",mode,i,(double)actual,expected,same,preserved);
    }
    mpfr_clear(x);mpfr_clear(y);mpfr_clear(z);mpfr_free_cache();
    if(fesetround(original))return 2;
    printf("{\"suite\":\"fenv-product\",\"rows\":12,\"failures\":%d}\n",failures);
    return failures!=0;
}
