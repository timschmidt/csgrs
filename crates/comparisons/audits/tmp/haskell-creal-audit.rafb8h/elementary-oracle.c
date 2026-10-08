#include <mpfr.h>
#include <string.h>

/* Independent directed MPFR enclosure for exact dyadic inputs.
 * 0 certified containment; 1 certified exclusion; 2 unresolved/invalid. */
int creal_oracle(const char *op, const char *point,
                const char *lower, const char *upper) {
    mpq_t q, l, u;
    mpq_inits(q, l, u, NULL);
    if (mpq_set_str(q, point, 10) || mpq_set_str(l, lower, 10) ||
        mpq_set_str(u, upper, 10)) {
        mpq_clears(q, l, u, NULL);
        return 2;
    }
    mpq_canonicalize(q); mpq_canonicalize(l); mpq_canonicalize(u);
    int result = 2;
    for (mpfr_prec_t bits = 512; bits <= 4096 && result == 2; bits *= 2) {
        mpfr_t x, a, b;
        mpfr_inits2(bits, x, a, b, (mpfr_ptr)0);
        if (mpfr_set_q(x, q, MPFR_RNDN) != 0) {
            mpfr_clears(x, a, b, (mpfr_ptr)0);
            break;
        }
        if (!strcmp(op, "pi")) {
            mpfr_const_pi(a, MPFR_RNDD); mpfr_const_pi(b, MPFR_RNDU);
        } else {
            int (*f)(mpfr_ptr, mpfr_srcptr, mpfr_rnd_t) = NULL;
            if (!strcmp(op,"exp")) f = mpfr_exp;
            if (!strcmp(op,"log")) f = mpfr_log;
            if (!strcmp(op,"sin")) f = mpfr_sin;
            if (!strcmp(op,"cos")) f = mpfr_cos;
            if (!strcmp(op,"atan")) f = mpfr_atan;
            if (!strcmp(op,"asin")) f = mpfr_asin;
            if (!strcmp(op,"acos")) f = mpfr_acos;
            if (!strcmp(op,"sqrt")) f = mpfr_sqrt;
            if (!strcmp(op,"tan")) f = mpfr_tan;
            if (!strcmp(op,"sinh")) f = mpfr_sinh;
            if (!strcmp(op,"cosh")) f = mpfr_cosh;
            if (!strcmp(op,"tanh")) f = mpfr_tanh;
            if (!strcmp(op,"asinh")) f = mpfr_asinh;
            if (!strcmp(op,"acosh")) f = mpfr_acosh;
            if (!strcmp(op,"atanh")) f = mpfr_atanh;
            if (!f) {
                mpfr_clears(x, a, b, (mpfr_ptr)0);
                break;
            }
            f(a, x, MPFR_RNDD); f(b, x, MPFR_RNDU);
        }
        if (!mpfr_nan_p(a) && !mpfr_nan_p(b)) {
            if (mpfr_cmp_q(a,l) >= 0 && mpfr_cmp_q(b,u) <= 0) result = 0;
            else if (mpfr_cmp_q(b,l) < 0 || mpfr_cmp_q(a,u) > 0) result = 1;
        }
        mpfr_clears(x, a, b, (mpfr_ptr)0);
    }
    mpq_clears(q, l, u, NULL);
    return result;
}
