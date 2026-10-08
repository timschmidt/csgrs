/* Independent reconstruction of selected fast-reals policies with stock MPFR.
 * This is NOT a native execution of the historical GHC/C-- binding. */
#include <mpfr.h>
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    mpz_t n, rounded;
    mpz_inits(n, rounded, NULL);
    mpfr_t x, y, z;
    mpfr_inits2(64, x, y, z, (mpfr_ptr) 0);
    unsigned total = 0, misses = 0, directed_failures = 0;
    const unsigned powers[] = {8, 32, 60, 63, 64, 65, 128, 256, 1024};
    const long offsets[] = {-17, -1, 0, 1, 17};
    for (unsigned i=0; i<sizeof powers/sizeof *powers; ++i)
      for (unsigned j=0; j<sizeof offsets/sizeof *offsets; ++j)
        for (int sign=-1; sign<=1; sign+=2) {
            mpz_set_ui(n, 1);
            mpz_mul_2exp(n, n, powers[i]);
            if (offsets[j] >= 0) mpz_add_ui(n, n, (unsigned long) offsets[j]);
            else mpz_sub_ui(n, n, (unsigned long) -offsets[j]);
            if (sign < 0) mpz_neg(n, n);
            mpfr_set_z(x, n, MPFR_RNDD);
            ++total;
            directed_failures += mpfr_cmp_z(x,n) > 0;
            if (mpfr_cmp_z(x,n) != 0) {
                ++misses;
                if (misses <= 4) {
                    mpfr_get_z(rounded,x,MPFR_RNDN);
                    gmp_printf("integer-point-miss original=%Zd rounded=%Zd\n",n,rounded);
                }
            }
        }
    printf("integer-policy checks=%u point-misses=%u MPFR-directed-failures=%u\n",
           total,misses,directed_failures);
    total=0; misses=0;
    for (long m=1; m<=257; ++m)
      for (unsigned p=2; p<=12; p+=2) {
          mpfr_set_prec(x,64);
          mpfr_set_si(x,-m,MPFR_RNDN);
          mpfr_set_prec(y,p);
          mpfr_neg(y,x,MPFR_RNDD); /* upper candidate in donor Interval.appAbs */
          mpfr_set_prec(z,64);
          mpfr_max(z,x,y,MPFR_RNDN);
          ++total;
          if (mpfr_cmp_si(z,m) < 0) {
              ++misses;
              if (misses<=4) mpfr_printf("abs-upper-miss abs(%ld) > %.0Rf at p=%u\n",-m,z,p);
          }
      }
    printf("interval-abs-policy checks=%u upper-misses=%u\n",total,misses);
    total=0; misses=0;
    for (long e=-128; e<=128; ++e) {
        mpfr_set_ui_2exp(x,1,e,MPFR_RNDN);
        long k = -mpfr_get_exp(x); /* donor appGetExp */
        mpfr_set_ui_2exp(y,1,k,MPFR_RNDN);
        mpfr_set_ui_2exp(z,1,k+1,MPFR_RNDN);
        ++total;
        misses += !(mpfr_cmp(y,x)<=0 && mpfr_cmp(x,z)<=0);
    }
    printf("declared-exponent-bound checks=%u failures=%u\n",total,misses);
    mpfr_clears(x,y,z,(mpfr_ptr) 0);
    mpz_clears(n,rounded,NULL);
    return directed_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
