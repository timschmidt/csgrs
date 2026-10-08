/* Independent exact-rational controls for the documented finite directed subset.
   This is not a reproducer for the separate field-relation assertion. */
#include <fenv.h>
#include <stdio.h>
#include <stdlib.h>
#include "gr.h"
#include "gr_mat.h"
#include "arf.h"
#include "nfloat.h"
#include "fmpq.h"

static unsigned long rows, comparisons, failures, equalities;
static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};

static void require(int ok)
{
    if (!ok) { fputs("control setup or output-domain failure\n", stderr); exit(2); }
}

/* Import the documented packed representation, without using donor arithmetic,
   export rounding or an approximate oracle. Never called on a failed output. */
static void to_q(mpq_t q, nfloat_srcptr x, gr_ctx_t ctx)
{
    if (NFLOAT_IS_ZERO(x)) { mpq_set_ui(q, 0, 1); return; }
    require(!NFLOAT_IS_SPECIAL(x) && NFLOAT_EXP(x) >= -200000 && NFLOAT_EXP(x) <= 200000);
    require(NFLOAT_SGNBIT(x) <= 1 && LIMB_MSB_IS_SET(NFLOAT_D(x)[NFLOAT_CTX_NLIMBS(ctx)-1]));
    mpz_import(mpq_numref(q), NFLOAT_CTX_NLIMBS(ctx), -1, sizeof(ulong), 0, 0, NFLOAT_D(x));
    mpz_set_ui(mpq_denref(q), 1);
    slong shift = NFLOAT_EXP(x) - NFLOAT_CTX_PREC(ctx);
    if (shift >= 0) mpz_mul_2exp(mpq_numref(q), mpq_numref(q), shift);
    else mpz_mul_2exp(mpq_denref(q), mpq_denref(q), -shift);
    if (NFLOAT_SGNBIT(x)) mpz_neg(mpq_numref(q), mpq_numref(q));
    mpq_canonicalize(q);
}

static int from_q(nfloat_ptr x, const mpq_t q, gr_ctx_t ctx)
{
    fmpq_t f; fmpq_init(f); fmpq_set_mpq(f, q);
    int status = nfloat_set_fmpq(x, f, ctx);
    fmpq_clear(f); return status;
}

static void dyadic(mpq_t q, slong bits, ulong odd, slong shift, int negative)
{
    mpz_set_ui(mpq_numref(q), 1);
    mpz_mul_2exp(mpq_numref(q), mpq_numref(q), bits);
    mpz_sub_ui(mpq_numref(q), mpq_numref(q), odd);
    mpz_set_ui(mpq_denref(q), 1);
    mpz_mul_2exp(mpq_denref(q), mpq_denref(q), bits);
    if (shift >= 0) mpz_mul_2exp(mpq_numref(q), mpq_numref(q), shift);
    else mpz_mul_2exp(mpq_denref(q), mpq_denref(q), -shift);
    if (negative) mpz_neg(mpq_numref(q), mpq_numref(q));
    mpq_canonicalize(q);
}

static int direction_ok(int cmp, int direction)
{
    comparisons++; equalities += cmp == 0;
    return direction ? cmp >= 0 : cmp <= 0;
}

static void report(const char *suite, slong limbs, int direction, int pattern,
                   int a, int b, int c, int d, int status, int checked, int ok, int mode)
{
    int preserved = fegetround() == modes[mode];
    failures += status != GR_SUCCESS || !ok || !preserved;
    rows++;
    printf("%s,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", suite, limbs,
           direction, pattern, a, b, c, d, status, checked, ok, preserved);
}

static void scalar_controls(gr_ctx_t ctx, int direction)
{
    const slong bits = NFLOAT_CTX_PREC(ctx), limbs = NFLOAT_CTX_NLIMBS(ctx);
    mpq_t xq, yq, expected, got, scratch;
    mpq_inits(xq, yq, expected, got, scratch, NULL);
    for (int pattern = 0; pattern < 8; pattern++)
    for (int op = 0; op < 10; op++)
    for (int alias = 0; alias < 3; alias++)
    {
        ulong x[NFLOAT_MAX_ALLOC] = {0}, y[NFLOAT_MAX_ALLOC] = {0}, z[NFLOAT_MAX_ALLOC] = {0};
        dyadic(xq, bits, pattern == 6 ? 3 : 1, pattern == 7 ? -bits : 0, pattern & 1);
        dyadic(yq, bits, 3, pattern >= 4 ? -bits-3 : pattern == 2 ? 1 : 0, (pattern >> 1) & 1);
        if (pattern == 3) mpq_neg(yq, xq);
        if (op >= 8) mpq_abs(xq, xq);
        require(from_q(x, xq, ctx) == GR_SUCCESS && from_q(y, yq, ctx) == GR_SUCCESS);
        /* The scalar corpus inputs are exactly representable dyadics. */
        to_q(scratch, x, ctx); require(mpq_equal(scratch, xq));
        to_q(scratch, y, ctx); require(mpq_equal(scratch, yq));
        mpq_set_ui(expected, 3, 8); require(from_q(z, expected, ctx) == GR_SUCCESS);
        nfloat_ptr out = alias == 1 ? x : alias == 2 ? y : z;
        to_q(expected, out, ctx);
        if (op == 0) mpq_add(expected, xq, yq);
        if (op == 1) mpq_sub(expected, xq, yq);
        if (op == 2) mpq_mul(expected, xq, yq);
        if (op == 3) mpq_div(expected, xq, yq);
        if (op == 4 || op == 5) {
            mpq_mul(scratch, xq, yq);
            if (op == 4) mpq_add(expected, expected, scratch);
            else mpq_sub(expected, expected, scratch);
        }
        if (op == 6) mpq_mul(expected, xq, xq);
        if (op == 7) mpq_inv(expected, xq);
        int mode = pattern % 4; require(fesetround(modes[mode]) == 0);
        int status = GR_UNABLE;
        switch (op) {
            case 0: status = nfloat_add(out, x, y, ctx); break;
            case 1: status = nfloat_sub(out, x, y, ctx); break;
            case 2: status = nfloat_mul(out, x, y, ctx); break;
            case 3: status = nfloat_div(out, x, y, ctx); break;
            case 4: status = nfloat_addmul(out, x, y, ctx); break;
            case 5: status = nfloat_submul(out, x, y, ctx); break;
            case 6: status = nfloat_sqr(out, x, ctx); break;
            case 7: status = nfloat_inv(out, x, ctx); break;
            case 8: status = nfloat_sqrt(out, x, ctx); break;
            case 9: status = nfloat_rsqrt(out, x, ctx); break;
        }
        int ok = 0;
        if (status == GR_SUCCESS) {
            to_q(got, out, ctx);
            int nonnegative = mpq_sgn(got) >= 0;
            if (op >= 8) {
                mpq_mul(got, got, got);
                if (op == 8) mpq_set(expected, xq);
                else { mpq_mul(got, got, xq); mpq_set_ui(expected, 1, 1); }
            }
            ok = direction_ok(mpq_cmp(got, expected), direction) && (op < 8 || nonnegative);
        }
        report("scalar", limbs, direction, pattern, op, alias, 0, 0, status,
               status == GR_SUCCESS, ok, mode);
    }
    mpq_clears(xq, yq, expected, got, scratch, NULL);
}

static void conversion_controls(gr_ctx_t ctx, int direction)
{
    slong bits = NFLOAT_CTX_PREC(ctx), limbs = NFLOAT_CTX_NLIMBS(ctx);
    gr_ctx_t other; require(nfloat_ctx_init(other, limbs == NFLOAT_MAX_LIMBS ? bits-FLINT_BITS : bits+FLINT_BITS, 0) == GR_SUCCESS);
    mpq_t expected, got; mpq_inits(expected, got, NULL);
    fmpq_t fq; fmpq_init(fq);
    arf_t ar; arf_init(ar);
    for (int pattern = 0; pattern < 8; pattern++)
    for (int kind = 0; kind < 5; kind++) {
        ulong out[NFLOAT_MAX_ALLOC] = {0}, input[NFLOAT_MAX_ALLOC] = {0};
        dyadic(expected, bits+7, 2*pattern+1, 0, pattern & 1);
        if (kind == 0) {
            mpz_mul_ui(mpq_denref(expected), mpq_denref(expected), 3);
            mpq_canonicalize(expected);
        }
        if (kind == 2) { mpz_set_ui(mpq_denref(expected), 1); }
        if (kind == 3) {
            require(from_q(input, expected, other) == GR_SUCCESS);
            to_q(expected, input, other);
        }
        double d = pattern & 1 ? -0.1 : 0.1;
        if (kind == 4) mpq_set_d(expected, d);
        fmpq_set_mpq(fq, expected);
        if (kind == 1) arf_set_fmpq(ar, fq, ARF_PREC_EXACT, ARF_RND_NEAR);
        int mode = pattern % 4; require(fesetround(modes[mode]) == 0);
        int status = kind == 0 ? nfloat_set_fmpq(out, fq, ctx) :
                     kind == 1 ? nfloat_set_arf(out, ar, ctx) :
                     kind == 2 ? nfloat_set_fmpz(out, fmpq_numref(fq), ctx) :
                     kind == 3 ? nfloat_set_other(out, input, other, ctx) : nfloat_set_d(out, d, ctx);
        int ok = 0;
        if (status == GR_SUCCESS) { to_q(got, out, ctx); ok = direction_ok(mpq_cmp(got, expected), direction); }
        report("conversion", limbs, direction, pattern, kind, 0, 0, 0, status, status == GR_SUCCESS, ok, mode);
    }
    arf_clear(ar); fmpq_clear(fq); mpq_clears(expected, got, NULL); gr_ctx_clear(other);
}

static void dot_controls(gr_ctx_t ctx, int direction)
{
    const slong lengths[] = {0, 1, 2, 3, 8, 17};
    slong bits = NFLOAT_CTX_PREC(ctx), limbs = NFLOAT_CTX_NLIMBS(ctx), stride = NFLOAT_CTX_DATA_NLIMBS(ctx);
    mpq_t xq, yq, initial, expected, scratch, got;
    mpq_inits(xq, yq, initial, expected, scratch, got, NULL);
    for (int li = 0; li < 6; li++)
    for (int pattern = 0; pattern < 4; pattern++) {
        slong len = lengths[li];
        ulong *x = flint_calloc((len+2)*stride, sizeof(ulong));
        ulong *yp = flint_calloc((len+2)*stride, sizeof(ulong)), *y = yp+stride;
        /* Padding keeps the empty reverse-dot address inside allocated storage. */
        for (slong i = 0; i < len; i++) {
            dyadic(xq, bits, 2*i+1, pattern == 2 ? -i*bits : 0, (pattern == 1 || pattern == 3) && i % 2);
            dyadic(yq, bits, 3, pattern == 3 ? i : 0, 0);
            if (pattern == 0 && i % 3 == 0) mpq_set_ui(xq, 0, 1);
            require(from_q(x+i*stride, xq, ctx) == GR_SUCCESS && from_q(y+i*stride, yq, ctx) == GR_SUCCESS);
        }
        for (int have_initial = 0; have_initial < 2; have_initial++)
        for (int subtract = 0; subtract < 2; subtract++)
        for (int reverse = 0; reverse < 2; reverse++)
        for (int alias = 0; alias < 2; alias++) {
            ulong z[NFLOAT_MAX_ALLOC] = {0}, init[NFLOAT_MAX_ALLOC] = {0};
            mpq_set_ui(initial, 3, 8); require(from_q(init, initial, ctx) == GR_SUCCESS);
            mpq_set_ui(expected, 0, 1);
            for (slong i = 0; i < len; i++) {
                to_q(xq, x+i*stride, ctx); to_q(yq, y+(reverse ? len-1-i : i)*stride, ctx);
                mpq_mul(scratch, xq, yq); mpq_add(expected, expected, scratch);
            }
            if (subtract) mpq_neg(expected, expected);
            if (have_initial) mpq_add(expected, expected, initial);
            nfloat_ptr out = alias ? init : z;
            int mode = pattern; require(fesetround(modes[mode]) == 0);
            int status = reverse ? _nfloat_vec_dot_rev(out, have_initial ? init : NULL, subtract, x, y, len, ctx) :
                                   _nfloat_vec_dot(out, have_initial ? init : NULL, subtract, x, y, len, ctx);
            int ok = 0;
            if (status == GR_SUCCESS) { to_q(got, out, ctx); ok = direction_ok(mpq_cmp(got, expected), direction); }
            report("dot", limbs, direction, pattern, (int) len, have_initial, subtract, 2*reverse+alias,
                   status, status == GR_SUCCESS, ok, mode);
        }
        flint_free(x); flint_free(yp);
    }
    mpq_clears(xq, yq, initial, expected, scratch, got, NULL);
}

static void matrix_controls(gr_ctx_t ctx, int direction)
{
    const slong shapes[][3] = {{0,0,0},{2,0,3},{1,1,1},{2,3,4},{4,2,3},{8,17,5},{17,8,17},{3,3,3},{8,8,8}};
    slong bits = NFLOAT_CTX_PREC(ctx), limbs = NFLOAT_CTX_NLIMBS(ctx);
    mpq_t xq, yq, scratch, got; mpq_inits(xq, yq, scratch, got, NULL);
    for (int shape = 0; shape < 9; shape++)
    for (int pattern = 0; pattern < 3; pattern++)
    for (int alias = 0; alias < (shape >= 7 ? 3 : 1); alias++) {
        slong m = shapes[shape][0], n = shapes[shape][1], p = shapes[shape][2];
        gr_mat_t a, b, c; gr_mat_init(a,m,n,ctx); gr_mat_init(b,n,p,ctx); gr_mat_init(c,m,p,ctx);
        mpq_t *expected = flint_malloc((m*p+1)*sizeof(mpq_t));
        for (slong i = 0; i < m*p; i++) mpq_init(expected[i]);
        for (slong i = 0; i < m; i++) for (slong j = 0; j < n; j++) {
            dyadic(xq,bits,2*j+1,pattern == 2 ? -j*bits : 0,pattern == 1 && (i+j)%2);
            if (pattern == 0 && (i+j)%3 == 0) mpq_set_ui(xq,0,1);
            require(from_q(gr_mat_entry_ptr(a,i,j,ctx),xq,ctx) == GR_SUCCESS);
        }
        for (slong i = 0; i < n; i++) for (slong j = 0; j < p; j++) {
            dyadic(yq,bits,2*j+3,0,pattern == 1 && j%2);
            require(from_q(gr_mat_entry_ptr(b,i,j,ctx),yq,ctx) == GR_SUCCESS);
        }
        for (slong i = 0; i < m; i++) for (slong j = 0; j < p; j++) for (slong k = 0; k < n; k++) {
            to_q(xq,gr_mat_entry_ptr(a,i,k,ctx),ctx); to_q(yq,gr_mat_entry_ptr(b,k,j,ctx),ctx);
            mpq_mul(scratch,xq,yq); mpq_add(expected[i*p+j],expected[i*p+j],scratch);
        }
        gr_mat_struct *out = alias == 1 ? a : alias == 2 ? b : c;
        int mode = (shape+pattern)%4; require(fesetround(modes[mode]) == 0);
        int status = nfloat_mat_mul(out,a,b,ctx), ok = status == GR_SUCCESS, checked = 0;
        if (status == GR_SUCCESS) for (slong i = 0; i < m; i++) for (slong j = 0; j < p; j++) {
            to_q(got,gr_mat_entry_ptr(out,i,j,ctx),ctx);
            ok &= direction_ok(mpq_cmp(got,expected[i*p+j]),direction); checked++;
        }
        report("matrix",limbs,direction,pattern,shape,alias,0,0,status,checked,ok,mode);
        for (slong i = 0; i < m*p; i++) mpq_clear(expected[i]);
        flint_free(expected); gr_mat_clear(a,ctx); gr_mat_clear(b,ctx); gr_mat_clear(c,ctx);
    }
    mpq_clears(xq,yq,scratch,got,NULL);
}

int main(void)
{
    int original = fegetround(); require(original != -1);
    puts("suite,limbs,direction,pattern,a,b,c,d,status,checked,ok,round_preserved");
    for (slong limbs = NFLOAT_MIN_LIMBS; limbs <= NFLOAT_MAX_LIMBS; limbs++)
    for (int direction = 0; direction < 2; direction++) {
        gr_ctx_t ctx;
        require(nfloat_ctx_init(ctx,limbs*FLINT_BITS,direction ? NFLOAT_RND_CEIL : NFLOAT_RND_FLOOR) == GR_SUCCESS);
        scalar_controls(ctx,direction); conversion_controls(ctx,direction);
        if (limbs <= 5 || limbs == 8 || limbs == 16 || limbs == 32 || limbs == NFLOAT_MAX_LIMBS) {
            dot_controls(ctx,direction); matrix_controls(ctx,direction);
        }
        gr_ctx_clear(ctx);
    }
    require(fesetround(original) == 0); flint_cleanup_master();
    printf("{\"suite\":\"nfloat-controls\",\"limb_bits\":%d,\"rows\":%lu,\"comparisons\":%lu,\"equalities\":%lu,\"failures\":%lu}\n",
           FLINT_BITS,rows,comparisons,equalities,failures);
    return failures != 0;
}
