/* Bounded valid-input numerical qualification, not a timing benchmark.
   GMP full integer products are independent of the truncated FLINT kernels.
   B = 2^64; q = B^(n-1); R comprises n high limbs plus the returned guard.
   Check 0 <= floor(P*2^shift/q)-R <= (n+2)*2^shift + (2^shift-1).
   Also report the stronger full-product residual bound separately.
   Inputs/outputs never alias; n is 1..2049 and normalised inputs have top bits set. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gmp.h>
#include "mpn_extras.h"

_Static_assert(FLINT_BITS == 64 && GMP_NUMB_BITS == 64, "64-bit zero-nail build required");
#define MAX_N 2049
#define PATTERNS 32
static size_t observations, failures, fractional_failures, exact_outputs, shifted_outputs;

static uint64_t mix(uint64_t x)
{
    x += UINT64_C(0x9e3779b97f4a7c15);
    x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
    return x ^ (x >> 31);
}

static mp_limb_t word(size_t n, unsigned p, size_t i)
{
    const mp_limb_t top = UWORD(1) << 63, ones = ~(mp_limb_t) 0;
    switch (p)
    {
        case 0: return 0;
        case 1: return i == 0 ? 1 : 0;
        case 2: return ones;
        case 3: return i + 1 == n ? top : 0;
        case 4: return (i & 1) ? 0 : ones;
        case 5: return UINT64_C(0xaaaaaaaaaaaaaaaa);
        case 6: return UINT64_C(0x5555555555555555);
        case 7: return i < n / 2 ? ones : 0;
        case 8: return i + 1 == n ? ones : 1;
        case 9: return i + 1 == n ? top : ones;
        case 10: return i == n / 2 ? top : 0;
        case 11: return i == 0 ? ones : top;
        case 12: return UWORD(1) << ((i * 7 + n) % 64);
        case 13: return ~(UWORD(1) << ((i * 13 + n) % 64));
        case 14: return i + 1 == n ? top - 1 : ones;
        case 15: return i + 1 == n ? top + 1 : 0;
        default: return mix((uint64_t) n * 31337 + (uint64_t) p * 1000003 + i);
    }
}

static void decode(mpz_t z, mp_srcptr limbs, size_t n)
{
    mpz_import(z, n, -1, sizeof(mp_limb_t), 0, 0, limbs);
}

static void check(const char *op, size_t n, unsigned pattern, int normalised,
                  unsigned shift, mp_srcptr r, const mpz_t product, int require_exact,
                  int require_top)
{
    mpz_t got, scaled, expected, deficit, residual, bound;
    const mp_bitcnt_t cut = (mp_bitcnt_t) (n - 1) * FLINT_BITS;
    unsigned long bound_ulps = ((unsigned long) n + 2) * (1UL << shift);
    int top_ok = !require_top || ((r[n] >> 63) == 1);
    mpz_inits(got, scaled, expected, deficit, residual, bound, NULL);
    decode(got, r, n + 1);
    mpz_mul_2exp(scaled, product, shift);
    mpz_fdiv_q_2exp(expected, scaled, cut);
    mpz_sub(deficit, expected, got);
    int exact = mpz_sgn(deficit) == 0;
    int ok = top_ok && mpz_sgn(deficit) >= 0 &&
             mpz_cmp_ui(deficit, bound_ulps + ((1UL << shift) - 1)) <= 0 &&
             (!require_exact || exact);
    mpz_mul_2exp(residual, got, cut);
    mpz_sub(residual, scaled, residual);
    mpz_set_ui(bound, bound_ulps);
    mpz_mul_2exp(bound, bound, cut);
    int fractional_ok = mpz_sgn(residual) >= 0 && mpz_cmp(residual, bound) <= 0;
    gmp_printf("{\"op\":\"%s\",\"n\":%zu,\"pattern\":%u,\"normalised_input\":%d,"
               "\"shift\":%u,\"deficit\":\"%Zd\",\"bound\":%lu,\"exact\":%d,"
               "\"require_exact\":%d,\"top_ok\":%d,\"fractional_ok\":%d,\"ok\":%d}\n",
               op, n, pattern, normalised, shift, deficit,
               bound_ulps + ((1UL << shift) - 1), exact, require_exact, top_ok, fractional_ok, ok);
    observations++;
    failures += !ok;
    fractional_failures += !fractional_ok;
    exact_outputs += exact;
    shifted_outputs += shift != 0;
    mpz_clears(got, scaled, expected, deficit, residual, bound, NULL);
}

int main(void)
{
    const size_t extra[] = {129,255,256,257,511,512,513,1023,1024,1025,1999,2000,2001,2047,2048,2049};
    size_t lengths[144], index = 0;
    mp_ptr a = calloc(MAX_N, sizeof(mp_limb_t)), b = calloc(MAX_N, sizeof(mp_limb_t));
    mp_ptr r = calloc(MAX_N + 1, sizeof(mp_limb_t)), scratch = calloc(2 * MAX_N, sizeof(mp_limb_t));
    if (!a || !b || !r || !scratch) return 2;
    mpz_t x, y, product, square, after;
    mpz_inits(x, y, product, square, after, NULL);
    for (size_t n = 1; n <= 128; n++) lengths[index++] = n;
    for (size_t i = 0; i < sizeof(extra) / sizeof(extra[0]); i++) lengths[index++] = extra[i];
    if (index != sizeof(lengths) / sizeof(lengths[0])) return 2;
    printf("{\"configuration\":true,\"limb_bits\":%d,\"lengths\":%zu,\"patterns\":%d,"
           "\"mul_table\":%d,\"sqr_table\":%d,\"mulders_mul\":%d,\"mulders_sqr\":%d,"
           "\"full_mul\":%d,\"full_sqr\":%d,\"fft_small\":0}\n",
           FLINT_BITS, index, PATTERNS, FLINT_MPN_MULHIGH_FUNC_TAB_WIDTH,
           FLINT_MPN_SQRHIGH_FUNC_TAB_WIDTH, FLINT_MPN_MULHIGH_MULDERS_CUTOFF,
           FLINT_MPN_SQRHIGH_MULDERS_CUTOFF, FLINT_MPN_MULHIGH_MUL_CUTOFF,
           FLINT_MPN_SQRHIGH_SQR_CUTOFF);
#if FLINT_HAVE_FFT_SMALL
#error This qualification targets the existing non-FFT build
#endif
    for (size_t k = 0; k < index; k++)
    for (unsigned p = 0; p < PATTERNS; p++)
    for (int norm = 0; norm <= 1; norm++)
    {
        const size_t n = lengths[k];
        for (size_t i = 0; i < n; i++)
        {
            a[i] = word(n, p, i);
            b[i] = p == 8 ? a[i] : p == 9 ? word(n, p, n - 1 - i) :
                   p == 10 ? ~a[i] : word(n, (p * 7 + 3) % PATTERNS, i);
        }
        if (norm) { a[n - 1] |= UWORD(1) << 63; b[n - 1] |= UWORD(1) << 63; }
        decode(x, a, n); decode(y, b, n);
        mpz_mul(product, x, y); mpz_mul(square, x, x);
        memset(r, 0xa5, (n + 1) * sizeof(mp_limb_t));
        r[0] = flint_mpn_mulhigh_n(r + 1, a, b, (mp_size_t) n);
        check("mul", n, p, norm, 0, r, product, n > FLINT_MPN_MULHIGH_MUL_CUTOFF, 0);
        memset(r, 0x5a, (n + 1) * sizeof(mp_limb_t));
        r[0] = flint_mpn_sqrhigh(r + 1, a, (mp_size_t) n);
        check("square", n, p, norm, 0, r, square, n > FLINT_MPN_SQRHIGH_SQR_CUTOFF, 0);
        memset(scratch, 0xa5, 2 * n * sizeof(mp_limb_t));
        flint_mpn_mul_or_mulhigh_n(scratch, a, b, (mp_size_t) n);
        check("mul-scratch", n, p, norm, 0, scratch + n - 1, product,
              n >= FLINT_MPN_MULHIGH_MUL_CUTOFF, 0);
        if (n <= 128)
        {
            memset(r, 0x5a, (n + 1) * sizeof(mp_limb_t));
            r[0] = _flint_mpn_mulhigh_n_naive(r + 1, a, b, (mp_size_t) n);
            check("mul-naive", n, p, norm, 0, r, product, n < 3, 0);
            memset(r, 0xa5, (n + 1) * sizeof(mp_limb_t));
            r[0] = _flint_mpn_mulhigh_n_recursive(r + 1, a, b, (mp_size_t) n);
            check("mul-recursive", n, p, norm, 0, r, product, n < 3, 0);
        }
        if (norm)
        {
            mp_limb_pair_t result;
            memset(r, 0x5a, (n + 1) * sizeof(mp_limb_t));
            result = flint_mpn_mulhigh_normalised(r + 1, a, b, (mp_size_t) n);
            if (result.m2 > 1) return 3;
            r[0] = result.m1;
            check("mul-normalised", n, p, norm, (unsigned) result.m2, r, product, 0, 1);
            memset(r, 0xa5, (n + 1) * sizeof(mp_limb_t));
            result = flint_mpn_sqrhigh_normalised(r + 1, a, (mp_size_t) n);
            if (result.m2 > 1) return 3;
            r[0] = result.m1;
            check("square-normalised", n, p, norm, (unsigned) result.m2, r, square, 0, 1);
        }
        decode(after, a, n); if (mpz_cmp(after, x)) return 4;
        decode(after, b, n); if (mpz_cmp(after, y)) return 4;
    }
    mpz_clears(x, y, product, square, after, NULL);
    free(a); free(b); free(r); free(scratch); flint_cleanup_master();
    printf("{\"summary\":true,\"observations\":%zu,\"failures\":%zu,\"fractional_failures\":%zu,"
           "\"exact_outputs\":%zu,\"shifted_outputs\":%zu}\n",
           observations, failures, fractional_failures, exact_outputs, shifted_outputs);
    return failures || fractional_failures ? 1 : 0;
}
