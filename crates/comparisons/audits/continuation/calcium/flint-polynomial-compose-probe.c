#define main original_dense_series_main
#include "flint-polynomial-series-probe.c"
#undef main

// Sum a_i*b(x)^i using only the independently defined scalar convolution.
static void composition(ca_poly_t out, const ca_poly_t a, const ca_poly_t b, ca_ctx_t ctx)
{
    ca_poly_t power, next;
    ca_t term, zero;
    ca_poly_init(power, ctx); ca_poly_init(next, ctx); ca_init(term, ctx); ca_init(zero, ctx);
    ca_poly_one(power, ctx); ca_poly_zero(out, ctx);
    slong limit = a->length && b->length ? (a->length-1)*(b->length-1)+1 : 1;
    for (slong i=0; i<a->length; i++) {
        for (slong j=0; j<power->length; j++) {
            ca_mul(term, a->coeffs+i, power->coeffs+j, ctx);
            ca_add(term, term, at(out,j,zero), ctx);
            ca_poly_set_coeff_ca(out,j,term,ctx);
        }
        product(next,power,b,limit,ctx); ca_poly_swap(power,next,ctx);
    }
    ca_poly_clear(power,ctx); ca_poly_clear(next,ctx); ca_clear(term,ctx); ca_clear(zero,ctx);
}
int main(void)
{
    const slong outer_lengths[]={0,1,2,7,8,21}, inner_lengths[]={0,1,2,5};
    ca_ctx_t ctx, other_ctx; ca_ctx_init(ctx); ca_ctx_init(other_ctx);
    ca_t quadratic, quartic, other, c;
    ca_init(quadratic,ctx); ca_init(quartic,ctx); ca_init(other,ctx); ca_init(c,ctx);
    ca_sqrt_ui(quadratic,2,ctx); ca_sqrt_ui(other,3,ctx);
    qqbar_t a2,a3; qqbar_init(a2); qqbar_init(a3);
    qqbar_sqrt_ui(a2,2); qqbar_sqrt_ui(a3,3); qqbar_add(a2,a2,a3);
    ca_set_qqbar(quartic,a2,ctx); qqbar_clear(a2); qqbar_clear(a3);
    int cases=0;
    puts("mode,outer_length,inner_length,shape,checks,failures");
    for(int mode=0;mode<4;mode++) for(int oi=0;oi<6;oi++)
    for(int ii=0;ii<4;ii++) for(int shape=0;shape<4;shape++) {
        const slong alen=outer_lengths[oi], blen=inner_lengths[ii];
        ca_poly_t a,b,ref,actual,alias,cross;
        ca_poly_init(a,ctx); ca_poly_init(b,ctx); ca_poly_init(ref,ctx);
        ca_poly_init(actual,ctx); ca_poly_init(alias,ctx); ca_poly_init(cross,other_ctx);
        input(a,mode,alen,2,quadratic,quartic,other,ctx);
        input(b,mode,blen,shape-1,quadratic,quartic,other,ctx);
        if(!alen) ca_poly_zero(a,ctx);
        if(!blen) ca_poly_zero(b,ctx);
        if(shape && blen>1) {
            ca_poly_zero(b,ctx); ca_set_si(c,shape-1,ctx); ca_poly_set_coeff_ca(b,0,c,ctx);
            if(shape==3) ca_set(c,quadratic,ctx); else ca_set_si(c,shape==1?1:-1,ctx);
            ca_poly_set_coeff_ca(b,blen-1,c,ctx);
        }
        int before_checks=checks,before_failures=failures;
        composition(ref,a,b,ctx);
        ca_poly_compose(actual,a,b,ctx); equal(actual,ref,ctx);
        ca_poly_set(alias,a,ctx); ca_poly_compose(alias,alias,b,ctx); equal(alias,ref,ctx);
        ca_poly_set(alias,b,ctx); ca_poly_compose(alias,a,alias,ctx); equal(alias,ref,ctx);
        slong n=alen+3;
        ca_poly_zero(ref,ctx);
        for(slong i=0;i<a->length;i++) ca_poly_set_coeff_ca(ref,n-1-i,a->coeffs+i,ctx);
        ca_poly_reverse(actual,a,n,ctx); equal(actual,ref,ctx);
        ca_poly_set(alias,a,ctx); ca_poly_reverse(alias,alias,n,ctx); equal(alias,ref,ctx);
        ca_poly_zero(ref,ctx);
        for(slong i=0;i<a->length;i++) ca_poly_set_coeff_ca(ref,i+3,a->coeffs+i,ctx);
        ca_poly_shift_left(actual,a,3,ctx); equal(actual,ref,ctx);
        ca_poly_set(alias,a,ctx); ca_poly_shift_left(alias,alias,3,ctx); equal(alias,ref,ctx);
        ca_poly_shift_right(alias,alias,3,ctx); equal(alias,a,ctx);
        ca_poly_zero(ref,ctx);
        for(slong i=3;i<a->length;i++) ca_poly_set_coeff_ca(ref,i-3,a->coeffs+i,ctx);
        ca_poly_shift_right(actual,a,3,ctx); equal(actual,ref,ctx);
        ca_poly_set(alias,a,ctx); ca_poly_shift_right(alias,alias,3,ctx); equal(alias,ref,ctx);
        ca_poly_transfer(actual,ctx,a,ctx); equal(actual,a,ctx);
        ca_poly_transfer(cross,other_ctx,a,ctx); ca_poly_transfer(actual,ctx,cross,other_ctx); equal(actual,a,ctx);
        printf("%d,%ld,%ld,%d,%d,%d\n",mode,(long)alen,(long)blen,shape,checks-before_checks,failures-before_failures);
        cases++;
        ca_poly_clear(a,ctx); ca_poly_clear(b,ctx); ca_poly_clear(ref,ctx);
        ca_poly_clear(actual,ctx); ca_poly_clear(alias,ctx); ca_poly_clear(cross,other_ctx);
    }
    ca_clear(quadratic,ctx); ca_clear(quartic,ctx); ca_clear(other,ctx); ca_clear(c,ctx);
    ca_ctx_clear(other_ctx); ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"polynomial-compose\",\"cases\":%d,\"checks\":%d,\"failures\":%d}\n",cases,checks,failures);
    return failures != 0;
}
