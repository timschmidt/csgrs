/* Numerical type-stability controls over valid rational inputs. Only results
   reporting GR_SUCCESS are inspected. No invalid pointer/failure-state use. */
#include <stdio.h>
#include "ca.h"
#include "gr.h"
#include "gr_special.h"

static const char *truth(truth_t t)
{ return t==T_TRUE ? "True" : t==T_FALSE ? "False" : "Unknown"; }

int main(void)
{
    const slong numerators[]={-4,-2,-1,0,1,2,4};
    const char *names[]={"asin","acos","arg","sqrt","log","exp","pow_half"};
    unsigned rows=0,incorrect=0,unresolved=0;
    puts("family,domain,operation,numerator,alias,status,real,algebraic,special");
    for(int domain=0;domain<4;domain++) {
        gr_ctx_t ctx;
        if(domain==0) gr_ctx_init_real_ca(ctx);
        if(domain==1) gr_ctx_init_complex_ca(ctx);
        if(domain==2) gr_ctx_init_real_algebraic_ca(ctx);
        if(domain==3) gr_ctx_init_complex_algebraic_ca(ctx);
        truth_t rv=gr_ctx_is_real_vector_space(ctx);
        printf("property,%d,real_vector_space,0,0,na,%s,na,na\n",domain,truth(rv));
        rows++; if(rv==T_UNKNOWN) unresolved++; else incorrect+=(rv!=(domain<2 ? T_TRUE : T_FALSE));
        ca_ctx_struct *ca_ctx=(ca_ctx_struct *)GR_CTX_DATA_AS_PTR(ctx);
        for(unsigned i=0;i<sizeof(numerators)/sizeof(*numerators);i++)for(int op=0;op<7;op++)
        for(int alias=0;alias<3;alias++) {
            if(op==2 && numerators[i]==0) continue; /* no Arg(0) convention assumption */
            gr_ptr x,y; GR_TMP_INIT2(x,y,ctx);
            GR_MUST_SUCCEED(gr_set_si(x,numerators[i],ctx));
            GR_MUST_SUCCEED(gr_div_ui(x,x,2,ctx));
            GR_MUST_SUCCEED(gr_set_ui(y,alias==0 ? 0 : 7,ctx));
            gr_ptr out=alias==2 ? x : y; int status=GR_UNABLE;
            if(op==0) status=gr_asin(out,x,ctx);
            if(op==1) status=gr_acos(out,x,ctx);
            if(op==2) status=gr_arg(out,x,ctx);
            if(op==3) status=gr_sqrt(out,x,ctx);
            if(op==4) status=gr_log(out,x,ctx);
            if(op==5) status=gr_exp(out,x,ctx);
            if(op==6) {
                fmpq_t half;fmpq_init(half);fmpq_set_si(half,1,2);
                status=gr_pow_fmpq(out,x,half,ctx);fmpq_clear(half);
            }
            if(status==GR_SUCCESS) {
                truth_t real=ca_check_is_real(out,ca_ctx),alg=ca_check_is_algebraic(out,ca_ctx);
                int special=ca_is_special(out,ca_ctx),need_real=domain==0||domain==2,need_alg=domain>=2;
                printf("operation,%d,%s,%ld,%d,%d,%s,%s,%d\n",domain,names[op],(long)numerators[i],alias,status,truth(real),truth(alg),special);
                if(special || (need_real&&real==T_FALSE) || (need_alg&&alg==T_FALSE)) incorrect++;
                else if((need_real&&real==T_UNKNOWN)||(need_alg&&alg==T_UNKNOWN)) unresolved++;
            } else printf("operation,%d,%s,%ld,%d,%d,na,na,na\n",domain,names[op],(long)numerators[i],alias,status);
            rows++; GR_TMP_CLEAR2(x,y,ctx);
        }
        gr_ctx_clear(ctx);
    }
    flint_cleanup();
    printf("{\"suite\":\"ca-domain\",\"rows\":%u,\"incorrect\":%u,\"unresolved\":%u}\n",rows,incorrect,unresolved);
    return incorrect||unresolved ? 1 : 0;
}
