/* Audit-only finite correctly-rounded scalar controls, independent GMP oracle.
   No invalid precisions, raw overlapping limbs, huge exponents or donor edits.
   The compact trace is a non-cryptographic cross-language consistency check;
   every donor result is also compared in full against the exact GMP oracle. */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <gmp.h>
#include "arf.h"

#define OPS 10
static const char *names[OPS] = {"set", "set-inplace", "neg", "neg-inplace",
    "mul", "mul-swapped", "mul-inplace", "mpfr-mul", "square", "mpfr-square"};
static const int pairs[][2] = {{1,1},{1,2},{2,2},{2,3},{3,7},{19,20},{20,20},
    {20,21},{25,25},{25,26},{26,26},{26,27},{40,41},{499,500},{500,500},
    {500,501},{1000,1},{1001,1}};
typedef struct { uint64_t count, exact, tie_even, tie_odd, carry, failures, trace; } stats;
static stats total[OPS];
static uint64_t input_checks, input_failures;
static mpz_t magnitude, quotient, remainder, half, expected, decoded, scratch;

static void canonical(mpz_t z, slong *e)
{
    if (mpz_sgn(z) == 0) { *e = 0; return; }
    mp_bitcnt_t t = mpz_scan1(z, 0);
    mpz_tdiv_q_2exp(z, z, t);
    *e += (slong)t;
}

/* No ARF conversion/rounding helper is used to decode results. */
static int decode(mpz_t z, slong *e, const arf_t x)
{
    if (arf_is_zero(x)) { mpz_set_ui(z, 0); *e = 0; return 1; }
    if (!arf_is_finite(x) || COEFF_IS_MPZ(ARF_EXP(x))) return 0;
    nn_srcptr limbs; slong n;
    ARF_GET_MPN_READONLY(limbs, n, x);
    mpz_import(z, (size_t)n, -1, sizeof(ulong), 0, 0, limbs);
    if (ARF_SGNBIT(x)) mpz_neg(z, z);
    *e = ARF_EXP(x) - n * FLINT_BITS;
    canonical(z, e);
    return 1;
}

static void input_check(const arf_t x, const mpz_t a, slong e)
{
    slong actual_e = 0;
    mpz_set(scratch, a); canonical(scratch, &e);
    input_checks++;
    input_failures += !decode(decoded, &actual_e, x) || actual_e != e || mpz_cmp(decoded, scratch) != 0;
}

static uint64_t mix(uint64_t h, uint64_t v)
{
    /* Explicit little-endian encoding, independent of host byte order. */
    for (unsigned i=0; i<8; i++) { h ^= v & 255; h *= UINT64_C(1099511628211); v >>= 8; }
    return h;
}

static void check(stats *s, const arf_t z, int ret, const mpz_t value,
    slong exponent, slong precision, arf_rnd_t mode)
{
    int sign = mpz_sgn(value) < 0, inex = 0, increment = 0, tie = 0, odd = 0, carry = 0;
    slong bits = mpz_sgn(value) == 0 ? 0 : (slong)mpz_sizeinbase(value, 2);
    slong drop = precision == ARF_PREC_EXACT || bits <= precision ? 0 : bits - precision;
    mpz_abs(magnitude, value);
    mpz_fdiv_q_2exp(quotient, magnitude, (mp_bitcnt_t)drop);
    mpz_fdiv_r_2exp(remainder, magnitude, (mp_bitcnt_t)drop);
    inex = mpz_sgn(remainder) != 0;
    if (inex)
    {
        if (mode == ARF_RND_UP) increment = 1;
        if (mode == ARF_RND_FLOOR) increment = sign;
        if (mode == ARF_RND_CEIL) increment = !sign;
        if (mode == ARF_RND_NEAR)
        {
            mpz_set_ui(half, 0); mpz_setbit(half, (mp_bitcnt_t)(drop - 1));
            int c = mpz_cmp(remainder, half);
            tie = c == 0; odd = mpz_odd_p(quotient) != 0;
            increment = c > 0 || (tie && odd);
        }
    }
    if (increment)
    {
        mpz_add_ui(quotient, quotient, 1);
        carry = (slong)mpz_sizeinbase(quotient, 2) > precision;
    }
    mpz_set(expected, quotient);
    if (sign) mpz_neg(expected, expected);
    slong expected_e = exponent + drop, actual_e = 0;
    canonical(expected, &expected_e);
    int valid = decode(decoded, &actual_e, z);
    int failed = !valid || actual_e != expected_e || mpz_cmp(decoded, expected) != 0 || ret != inex;
    s->count++; s->exact += !inex; s->tie_even += tie && !odd;
    s->tie_odd += tie && odd; s->carry += carry; s->failures += failed;
    if (failed) fprintf(stderr, "rounding mismatch at group output %" PRIu64 ", p=%ld mode=%d\n", s->count, precision, mode);
    /* Two residues plus sign, exponent, bit length and exactness. This compact
       trace is not a replacement for the full mpz comparison above. */
    s->trace = mix(s->trace, (uint64_t)actual_e);
    s->trace = mix(s->trace, (uint64_t)(mpz_sgn(decoded) + 1));
    s->trace = mix(s->trace, mpz_sgn(decoded) ? mpz_sizeinbase(decoded, 2) : 0);
    s->trace = mix(s->trace, mpz_fdiv_ui(decoded, 4294967291UL));
    s->trace = mix(s->trace, mpz_fdiv_ui(decoded, 4294967279UL));
    s->trace = mix(s->trace, (uint64_t)ret);
}

static void pattern(mpz_t a, int limbs, int p, int side)
{
    const mp_bitcnt_t bits = (mp_bitcnt_t)limbs * 64;
    mpz_set_ui(a, 0);
    if (p == 0) return;
    if (p == 1) { mpz_setbit(a, bits-1); return; }
    if (p == 2) { mpz_setbit(a, bits); mpz_sub_ui(a, a, 1); return; }
    if (p == 3) { mpz_setbit(a, bits-1); mpz_add_ui(a, a, 1); return; }
    if (p == 4 || p == 5) {
        mpz_set_ui(a, p == 4 ? 5 : 7); mpz_mul_2exp(a, a, bits-3); return;
    }
    if (p == 6) { mpz_setbit(a, bits); mpz_sub_ui(a, a, 3); return; }
    if (p == 7) {
        mpz_setbit(a, bits-1); mpz_setbit(a, bits/2); mpz_setbit(a, bits/2-2); return;
    }
    for (int i=0; i<limbs; i++) {
        uint64_t w = UINT64_C(0x9e3779b97f4a7c15) * (uint64_t)(i + 1 + 17*side);
        w ^= UINT64_C(0xd1b54a32d192ed03) * (uint64_t)(p+1);
        mpz_mul_2exp(a, a, 64); mpz_add_ui(a, a, (ulong)w);
    }
    mpz_setbit(a, bits-1); mpz_setbit(a, 0);
    if (p == 9) mpz_clrbit(a, bits-1); /* leading-bit renormalisation */
}

static void emit(const char *kind, int pair, int pat, int sign, int op, const stats *s)
{
    printf("{\"kind\":\"%s\",\"pair\":%d,\"pattern\":%d,\"sign\":%d,\"op\":\"%s\","
        "\"count\":%" PRIu64 ",\"exact\":%" PRIu64 ",\"tieEven\":%" PRIu64
        ",\"tieOdd\":%" PRIu64 ",\"carry\":%" PRIu64 ",\"failures\":%" PRIu64
        ",\"trace\":\"%016" PRIx64 "\"}\n", kind,pair,pat,sign,names[op],s->count,s->exact,
        s->tie_even,s->tie_odd,s->carry,s->failures,s->trace);
}

int main(void)
{
    if (FLINT_BITS != 64 || sizeof(ulong) != 8 || GMP_NUMB_BITS != 64) return 2;
    mpz_inits(magnitude,quotient,remainder,half,expected,decoded,scratch,NULL);
    mpz_t a,b,neg_a,product,square;
    mpz_inits(a,b,neg_a,product,square,NULL);
    arf_t x,y,z;
    arf_init(x); arf_init(y); arf_init(z);
    for (size_t j=0; j<sizeof(pairs)/sizeof(pairs[0]); j++)
    for (int p=0; p<10; p++)
    for (int signs=0; signs<4; signs++)
    {
        pattern(a,pairs[j][0],p,0); pattern(b,pairs[j][1],p,1);
        if (signs & 1) mpz_neg(a,a);
        if (signs & 2) mpz_neg(b,b);
        mpz_neg(neg_a,a); mpz_mul(product,a,b); mpz_mul(square,a,a);
        slong ae = (p%3-1)*257, be = ((p+1)%3-1)*131;
        arf_set_mpz(x,a); arf_mul_2exp_si(x,x,ae);
        arf_set_mpz(y,b); arf_mul_2exp_si(y,y,be);
        input_check(x,a,ae); input_check(y,b,be);
        slong size_bits = (pairs[j][0] + pairs[j][1])*64;
        slong cut = size_bits * 4 / 5;
        slong precisions[] = {1,2,3,53,63,64,65,127,128,129,cut-1,cut,cut+1,
            size_bits-1,size_bits,size_bits+1,ARF_PREC_EXACT};
        stats group[OPS] = {{0}};
        for (int op=0; op<OPS; op++) group[op].trace = UINT64_C(14695981039346656037);
        for (size_t k=0; k<sizeof(precisions)/sizeof(precisions[0]); k++)
        for (int r=0; r<5; r++)
        for (int op=0; op<OPS; op++)
        {
            slong prec = precisions[k]; arf_rnd_t rnd = (arf_rnd_t)r; int ret;
            /* Seed a large finite destination to exercise demotion/reuse. */
            arf_set_mpz(z,square);
            if (op==0) ret=arf_set_round(z,x,prec,rnd);
            else if (op==1) { arf_set(z,x); ret=arf_set_round(z,z,prec,rnd); }
            else if (op==2) ret=arf_neg_round(z,x,prec,rnd);
            else if (op==3) { arf_set(z,x); ret=arf_neg_round(z,z,prec,rnd); }
            else if (op==4) ret=arf_mul(z,x,y,prec,rnd);
            else if (op==5) ret=arf_mul(z,y,x,prec,rnd);
            else if (op==6) { arf_set(z,x); ret=arf_mul(z,z,y,prec,rnd); }
            else if (op==7) ret=arf_mul_via_mpfr(z,x,y,prec,rnd);
            else if (op==8) ret=arf_mul(z,x,x,prec,rnd);
            else ret=arf_mul_via_mpfr(z,x,x,prec,rnd);
            check(&group[op],z,ret,op<2?a:op<4?neg_a:op<8?product:square,
                op<4?ae:op<8?ae+be:2*ae,prec,rnd);
        }
        input_check(x,a,ae); input_check(y,b,be);
        for (int op=0; op<OPS; op++) {
            emit("group",(int)j,p,signs,op,&group[op]);
            total[op].count+=group[op].count; total[op].exact+=group[op].exact;
            total[op].tie_even+=group[op].tie_even; total[op].tie_odd+=group[op].tie_odd;
            total[op].carry+=group[op].carry; total[op].failures+=group[op].failures;
        }
    }
    uint64_t failures=input_failures, count=0;
    for (int op=0; op<OPS; op++) {
        emit("total",-1,-1,-1,op,&total[op]); failures+=total[op].failures; count+=total[op].count;
    }
    printf("{\"kind\":\"summary\",\"outputs\":%" PRIu64 ",\"inputChecks\":%" PRIu64
        ",\"inputFailures\":%" PRIu64 ",\"failures\":%" PRIu64 "}\n",count,input_checks,input_failures,failures);
    arf_clear(x); arf_clear(y); arf_clear(z);
    mpz_clears(a,b,neg_a,product,square,magnitude,quotient,remainder,half,expected,decoded,scratch,NULL);
    flint_cleanup_master();
    return failures ? 1 : 0;
}
