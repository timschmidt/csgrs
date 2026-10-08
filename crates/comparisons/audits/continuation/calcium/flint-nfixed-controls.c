/* Bounded numerical controls for fixed-point arithmetic and advertised bounds.
   Inputs and intermediate witnesses stay far from overflow. No zero-length
   raw dots, invalid aliases, nonfinite values or crash reproduction. */
#include <fenv.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "gr.h"
#include "nfloat.h"
#include <gmp.h>

static unsigned long rows, checked, failed, bound_failures, cutoff_differences;
static uint64_t state = UINT64_C(0x243f6a8885a308d3);
static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
static void require(int ok) { if (!ok) { fputs("control setup failed\n",stderr); exit(2); } }
static ulong word(void) { state ^= state << 13; state ^= state >> 7; state ^= state << 17; return (ulong)state; }
static void integer(mpz_t z, const ulong *x, slong limbs)
{
    require(x[0] <= 1); mpz_import(z,limbs,-1,sizeof(ulong),0,0,x+1);
    if (x[0]) mpz_neg(z,z);
}
static void fill(ulong *x, slong limbs, int pattern, slong i, int side, unsigned top)
{
    x[0] = pattern == 0 ? 0 : pattern == 1 ? (ulong)((i+side)%2) : word() & 1;
    for (slong k=1;k<=limbs;k++) x[k] = pattern < 2 ? ~(ulong)0 : word();
    x[limbs] >>= top;
    if (pattern == 3 && i%3 == 0) for (slong k=1;k<=limbs;k++) x[k]=0;
}
static int error_ok(const ulong *x, const mpz_t exact_product_sum, slong limbs, double error, double *observed)
{
    mpz_t z; mpz_init(z); integer(z,x,limbs);
    mpz_mul_2exp(z,z,limbs*FLINT_BITS); mpz_sub(z,z,exact_product_sum); mpz_abs(z,z);
    mpq_t difference,allowed; mpq_inits(difference,allowed,NULL);
    mpz_set(mpq_numref(difference),z); mpz_set_ui(mpq_denref(difference),1);
    mpz_mul_2exp(mpq_denref(difference),mpq_denref(difference),limbs*FLINT_BITS);
    mpq_canonicalize(difference); mpq_set_d(allowed,error);
    int ok=mpq_cmp(difference,allowed)<=0; *observed=mpq_get_d(difference);
    mpq_clears(difference,allowed,NULL); mpz_clear(z); checked++; return ok;
}
static void dot_controls(void)
{
    const slong lengths[]={1,2,3,8,17};
    for (slong limbs=2;limbs<=8;limbs++) for(int li=0;li<5;li++)
    for(int pattern=0;pattern<4;pattern++) for(int layout=0;layout<4;layout++) {
        slong len=lengths[li],stride=(limbs+1)*(layout&1 ? 2 : 1);
        ulong *a=flint_calloc(len*stride,sizeof(ulong)),*b=flint_calloc(len*stride,sizeof(ulong));
        ulong out[NFLOAT_MAX_LIMBS+1]={0};
        for(slong i=0;i<len;i++) {fill(a+i*stride,limbs,pattern,i,0,12);fill(b+i*stride,limbs,pattern,i,1,12);}
        const ulong *x=layout>=2 ? a+(len-1)*stride : a;
        slong xs=layout>=2 ? -stride : stride;
        mpz_t sum,u,v;mpz_inits(sum,u,v,NULL);
        for(slong i=0;i<len;i++){integer(u,x+i*xs,limbs);integer(v,b+i*stride,limbs);mpz_addmul(sum,u,v);}
        require(fesetround(modes[pattern])==0);
        switch(limbs) {
          case 2:_nfixed_dot_2(out,x,xs,b,stride,len);break;
          case 3:_nfixed_dot_3(out,x,xs,b,stride,len);break;
          case 4:_nfixed_dot_4(out,x,xs,b,stride,len);break;
          case 5:_nfixed_dot_5(out,x,xs,b,stride,len);break;
          case 6:_nfixed_dot_6(out,x,xs,b,stride,len);break;
          case 7:_nfixed_dot_7(out,x,xs,b,stride,len);break;
          case 8:_nfixed_dot_8(out,x,xs,b,stride,len);break;
        }
        double observed,error=(double)((2*limbs-1)*len);
        int ok=error_ok(out,sum,limbs,error,&observed),preserved=fegetround()==modes[pattern];
        failed+=!ok||!preserved;rows++;
        printf("dot,%ld,%ld,%d,%d,%a,%a,%d,%d\n",limbs,len,pattern,layout,error,observed,ok,preserved);
        mpz_clears(sum,u,v,NULL);flint_free(a);flint_free(b);
    }
}
static void matrix_case(slong limbs,slong m,slong n,slong p,int pattern)
{
    slong stride=limbs+1;
    ulong *a=flint_calloc(m*n*stride,sizeof(ulong)),*b=flint_calloc(n*p*stride,sizeof(ulong));
    ulong *c=flint_calloc(m*p*stride,sizeof(ulong));
    mpz_t *ai=flint_malloc(m*n*sizeof(mpz_t)),*bi=flint_malloc(n*p*sizeof(mpz_t)),*sums=flint_malloc(m*p*sizeof(mpz_t));
    for(slong i=0;i<m*n;i++){fill(a+i*stride,limbs,pattern,i,0,20);mpz_init(ai[i]);integer(ai[i],a+i*stride,limbs);}
    for(slong i=0;i<n*p;i++){fill(b+i*stride,limbs,pattern,i,1,20);mpz_init(bi[i]);integer(bi[i],b+i*stride,limbs);}
    for(slong i=0;i<m;i++)for(slong j=0;j<p;j++) {
        mpz_init(sums[i*p+j]);for(slong k=0;k<n;k++)mpz_addmul(sums[i*p+j],ai[i*n+k],bi[k*p+j]);
    }
    for(int algorithm=0;algorithm<5;algorithm++) {
        double bound,error,max_observed=0,A=0x1p-20;
        require(fesetround(modes[pattern])==0);
        if(algorithm==0)_nfixed_mat_mul_bound_classical(&bound,&error,m,n,p,A,A,limbs);
        if(algorithm==1)_nfixed_mat_mul_bound_waksman(&bound,&error,m,n,p,A,A,limbs);
        if(algorithm==2||algorithm==3)_nfixed_mat_mul_bound_strassen(&bound,&error,m,n,p,A,A,algorithm==2?-1:2,limbs);
        if(algorithm==4)_nfixed_mat_mul_bound(&bound,&error,m,n,p,A,A,limbs);
        require(isfinite(bound)&&bound>=0&&bound<0.015625&&isfinite(error)&&error>=0);
        if(algorithm==0)_nfixed_mat_mul_classical(c,a,b,m,n,p,limbs);
        if(algorithm==1)_nfixed_mat_mul_waksman(c,a,b,m,n,p,limbs);
        if(algorithm==2||algorithm==3)_nfixed_mat_mul_strassen(c,a,b,m,n,p,algorithm==2?-1:2,limbs);
        if(algorithm==4)_nfixed_mat_mul(c,a,b,m,n,p,limbs);
        unsigned bad=0;
        for(slong i=0;i<m*p;i++){double observed;bad+=!error_ok(c+i*stride,sums[i],limbs,error,&observed);if(observed>max_observed)max_observed=observed;}
        int preserved=fegetround()==modes[pattern];failed+=bad!=0||!preserved;rows++;
        printf("matrix,%ld,%ld,%ld,%ld,%d,%d,%a,%a,%a,%u,%d\n",limbs,m,n,p,pattern,algorithm,bound,error,max_observed,bad,preserved);
    }
    for(slong i=0;i<m*n;i++)mpz_clear(ai[i]);
    for(slong i=0;i<n*p;i++)mpz_clear(bi[i]);
    for(slong i=0;i<m*p;i++)mpz_clear(sums[i]);
    flint_free(ai);flint_free(bi);flint_free(sums);flint_free(a);flint_free(b);flint_free(c);
}
static void bound_controls(void)
{
    const slong precisions[]={2,3,4,8,12,66};
    for(int pi=0;pi<6;pi++)for(int mode=0;mode<4;mode++) {
        slong limbs=precisions[pi]; double bound,error;
        ulong q[NFLOAT_MAX_LIMBS+1]={0},minus[NFLOAT_MAX_LIMBS+1]={0},s[NFLOAT_MAX_LIMBS+1]={0};
        q[limbs]=(ulong)1<<(FLINT_BITS-20);minus[limbs]=q[limbs];minus[0]=1;
        /* Native Strassen S4 = A22 - A21 + A12 - A11. Choose
           A22=A12=q, A21=A11=-q. The largest witness is 2^-18. */
        _nfixed_vec_sub(s,q,minus,1,limbs);_nfixed_vec_add(s,s,q,1,limbs);_nfixed_vec_sub(s,s,minus,1,limbs);
        mpz_t z,expected;mpz_inits(z,expected,NULL);integer(z,s,limbs);
        mpz_set_ui(expected,1);mpz_mul_2exp(expected,expected,limbs*FLINT_BITS-18);require(mpz_cmp(z,expected)==0);
        require(fesetround(modes[mode])==0);
        _nfixed_mat_mul_bound_strassen(&bound,&error,2,2,2,0x1p-20,0x1p-20,2,limbs);
        int encloses=bound>=0x1p-18,preserved=fegetround()==modes[mode];
        bound_failures+=!encloses;failed+=!encloses||!preserved;rows++;
        printf("bound,%ld,%d,%a,%a,%d,%d\n",limbs,mode,bound,0x1p-18,encloses,preserved);
        mpz_clears(z,expected,NULL);
    }
    for(int pi=0;pi<6;pi++)for(slong n=24;n<=60;n++) {
        slong limbs=precisions[pi],cutoff=limbs<=3?(n%2?57:50):(n%2?37:26);
        double automatic,ae,explicit_bound,ee;
        require(fesetround(FE_TONEAREST)==0);
        _nfixed_mat_mul_bound_strassen(&automatic,&ae,n,n,n,0x1p-20,0x1p-20,-1,limbs);
        _nfixed_mat_mul_bound_strassen(&explicit_bound,&ee,n,n,n,0x1p-20,0x1p-20,cutoff,limbs);
        int equal=automatic==explicit_bound&&ae==ee;cutoff_differences+=!equal;rows++;
        printf("cutoff,%ld,%ld,%ld,%a,%a,%a,%a,%d\n",limbs,n,cutoff,automatic,explicit_bound,ae,ee,equal);
    }
}
int main(void)
{
    int original=fegetround();require(original!=-1&&FLINT_BITS==64);
    const slong shapes[][3]={{1,1,1},{2,3,4},{3,4,2},{3,3,3},{8,8,8},{11,26,13},{26,26,26},{27,27,27},
        {36,36,36},{37,37,37},{50,50,50},{51,51,51},{56,56,56},{57,57,57},{58,57,59}};
    const slong precisions[]={2,3,4,8,12,22,47,66};
    dot_controls();
    for(int pi=0;pi<8;pi++)for(int si=0;si<(pi<5?15:4);si++)for(int pattern=0;pattern<4;pattern++)
        matrix_case(precisions[pi],shapes[si][0],shapes[si][1],shapes[si][2],pattern);
    bound_controls();require(fesetround(original)==0);flint_cleanup_master();
    printf("{\"suite\":\"nfixed-controls\",\"rows\":%lu,\"comparisons\":%lu,\"failed_cases\":%lu,\"bound_failures\":%lu,\"cutoff_differences\":%lu}\n",
           rows,checked,failed,bound_failures,cutoff_differences);
    return failed!=0;
}
