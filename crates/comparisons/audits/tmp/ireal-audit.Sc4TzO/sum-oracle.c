#include <mpfr.h>

/* A separate reference for the benchmark's finite sine sum. Input rationals
 * are enclosed with directed rounding. The global 1-Lipschitz bound for sin
 * covers input conversion error, independently of donor expression trees. */
int ireal_sum_oracle(int dependent, unsigned long n, unsigned long seed,
                     const char *lower, const char *upper) {
    mpq_t q,l,u;
    mpq_inits(q,l,u,NULL);
    if (mpq_set_str(l,lower,10)||mpq_set_str(u,upper,10)) {
        mpq_clears(q,l,u,NULL); return 2;
    }
    mpq_canonicalize(l); mpq_canonicalize(u);
    mpfr_t x,xlo,xhi,error,other,slo,shi,total_lo,total_hi;
    mpfr_inits2(1024,x,xlo,xhi,error,other,slo,shi,total_lo,total_hi,(mpfr_ptr)0);
    mpfr_set_zero(total_lo,1); mpfr_set_zero(total_hi,1);
    for (unsigned long k=1;k<=n;k++) {
        mpq_set_ui(q,(k+seed)%97+1,k%29+31);
        mpq_canonicalize(q);
        mpfr_set_q(x,q,MPFR_RNDN);
        mpfr_set_q(xlo,q,MPFR_RNDD); mpfr_set_q(xhi,q,MPFR_RNDU);
        mpfr_sub(error,x,xlo,MPFR_RNDU); mpfr_sub(other,xhi,x,MPFR_RNDU);
        mpfr_max(error,error,other,MPFR_RNDU);
        mpfr_sin(slo,x,MPFR_RNDD); mpfr_sin(shi,x,MPFR_RNDU);
        mpfr_sub(slo,slo,error,MPFR_RNDD); mpfr_add(shi,shi,error,MPFR_RNDU);
        unsigned long weight=dependent?n-k+1:1;
        mpfr_mul_ui(slo,slo,weight,MPFR_RNDD); mpfr_mul_ui(shi,shi,weight,MPFR_RNDU);
        mpfr_add(total_lo,total_lo,slo,MPFR_RNDD);
        mpfr_add(total_hi,total_hi,shi,MPFR_RNDU);
    }
    int result=2;
    if (mpfr_cmp_q(total_lo,l)>=0&&mpfr_cmp_q(total_hi,u)<=0) result=0;
    else if (mpfr_cmp_q(total_hi,l)<0||mpfr_cmp_q(total_lo,u)>0) result=1;
    mpfr_clears(x,xlo,xhi,error,other,slo,shi,total_lo,total_hi,(mpfr_ptr)0);
    mpq_clears(q,l,u,NULL);
    return result;
}
