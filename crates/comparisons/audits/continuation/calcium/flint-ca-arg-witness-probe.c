/* Supplemental exact-value witness: Arg of a negative rational is pi.
   Algebraicity predicates may be Unknown even for this known transcendental. */
#include <stdio.h>
#include "ca.h"
#include "gr.h"

int main(void)
{
    unsigned rows=0,failures=0;
    puts("domain,numerator,alias,status,equals_pi");
    for(int domain=0;domain<4;domain++) {
        gr_ctx_t ctx;
        if(domain==0) gr_ctx_init_real_ca(ctx);
        if(domain==1) gr_ctx_init_complex_ca(ctx);
        if(domain==2) gr_ctx_init_real_algebraic_ca(ctx);
        if(domain==3) gr_ctx_init_complex_algebraic_ca(ctx);
        ca_ctx_struct *cc=(ca_ctx_struct *)GR_CTX_DATA_AS_PTR(ctx);
        ca_t expected; ca_init(expected,cc); ca_pi(expected,cc);
        for(slong numerator=-4;numerator<=-1;numerator++) {
            if(numerator==-3) continue;
            for(int alias=0;alias<3;alias++) {
                gr_ptr x,y;GR_TMP_INIT2(x,y,ctx);
                GR_MUST_SUCCEED(gr_set_si(x,numerator,ctx));GR_MUST_SUCCEED(gr_div_ui(x,x,2,ctx));
                GR_MUST_SUCCEED(gr_set_ui(y,alias==0 ? 0 : 7,ctx));
                gr_ptr out=alias==2 ? x : y;int status=gr_arg(out,x,ctx);
                truth_t eq=status==GR_SUCCESS ? ca_check_equal(out,expected,cc) : T_UNKNOWN;
                printf("%d,%ld,%d,%d,%s\n",domain,(long)numerator,alias,status,
                    eq==T_TRUE ? "True" : eq==T_FALSE ? "False" : "Unknown");
                rows++;failures+=status!=GR_SUCCESS||eq!=T_TRUE;
                GR_TMP_CLEAR2(x,y,ctx);
            }
        }
        ca_clear(expected,cc);gr_ctx_clear(ctx);
    }
    flint_cleanup();
    printf("{\"suite\":\"ca-arg-witness\",\"rows\":%u,\"failures\":%u}\n",rows,failures);
    return failures ? 1 : 0;
}
