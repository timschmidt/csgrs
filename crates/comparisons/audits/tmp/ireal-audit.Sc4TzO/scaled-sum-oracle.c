#include <mpfr.h>
#include <stdio.h>
#include <stdlib.h>

/* Independently reduce sum_i scale_i*prefix_i to a weighted finite sine sum.
 * Exact GMP suffix weights avoid depending on either expression tree order.
 * Directed MPFR sin plus a global Lipschitz input-error bound encloses terms.
 */
int main(int argc, char **argv) {
    if (argc != 6) return 3;
    unsigned long n = strtoul(argv[1], 0, 10);
    int step = atoi(argv[2]);
    unsigned long seed = strtoul(argv[3], 0, 10);
    if (n == 0 || n > 4096 || step < -8 || step > 8) return 3;
    mpq_t q, lower, upper, scale, factor, remaining;
    mpq_inits(q, lower, upper, scale, factor, remaining, NULL);
    if (mpq_set_str(lower, argv[4], 10) || mpq_set_str(upper, argv[5], 10)) return 3;
    mpq_canonicalize(lower); mpq_canonicalize(upper);
    mpq_set_ui(scale, 1, 1); mpq_set_ui(factor, 1, 1);
    if (step >= 0) mpq_div_2exp(factor, factor, (unsigned)step);
    else mpq_mul_2exp(factor, factor, (unsigned)-step);
    for (unsigned long i=0; i<n; i++) {
        mpq_add(remaining, remaining, scale);
        mpq_mul(scale, scale, factor);
    }
    mpq_set_ui(scale, 1, 1);
    mpfr_prec_t precision = 2048 + (mpfr_prec_t)abs(step) * n;
    mpfr_t x, xlo, xhi, error, other, slo, shi, total_lo, total_hi;
    mpfr_inits2(precision, x, xlo, xhi, error, other, slo, shi, total_lo, total_hi, (mpfr_ptr)0);
    mpfr_set_zero(total_lo, 1); mpfr_set_zero(total_hi, 1);
    for (unsigned long k=1; k<=n; k++) {
        mpq_set_ui(q, (k+seed)%97+1, k%29+31); mpq_canonicalize(q);
        mpfr_set_q(x, q, MPFR_RNDN);
        mpfr_set_q(xlo, q, MPFR_RNDD); mpfr_set_q(xhi, q, MPFR_RNDU);
        mpfr_sub(error, x, xlo, MPFR_RNDU); mpfr_sub(other, xhi, x, MPFR_RNDU);
        mpfr_max(error, error, other, MPFR_RNDU);
        mpfr_sin(slo, x, MPFR_RNDD); mpfr_sin(shi, x, MPFR_RNDU);
        mpfr_sub(slo, slo, error, MPFR_RNDD); mpfr_add(shi, shi, error, MPFR_RNDU);
        mpfr_mul_q(slo, slo, remaining, MPFR_RNDD);
        mpfr_mul_q(shi, shi, remaining, MPFR_RNDU);
        mpfr_add(total_lo, total_lo, slo, MPFR_RNDD);
        mpfr_add(total_hi, total_hi, shi, MPFR_RNDU);
        mpq_sub(remaining, remaining, scale);
        mpq_mul(scale, scale, factor);
    }
    int result = 2;
    if (mpfr_cmp_q(total_lo, lower)>=0 && mpfr_cmp_q(total_hi, upper)<=0) result = 0;
    else if (mpfr_cmp_q(total_hi, lower)<0 || mpfr_cmp_q(total_lo, upper)>0) result = 1;
    if (result) fprintf(stderr, "scaled finite-sum oracle status=%d n=%lu step=%d\n", result, n, step);
    mpfr_clears(x, xlo, xhi, error, other, slo, shi, total_lo, total_hi, (mpfr_ptr)0);
    mpq_clears(q, lower, upper, scale, factor, remaining, NULL);
    return result;
}
