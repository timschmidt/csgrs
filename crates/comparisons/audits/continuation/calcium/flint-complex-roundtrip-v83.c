#define main symbolic_boundary_unused_main
#include "flint-symbolic-boundary-fixed-v83.c"
#undef main

static void apply_component(qqbar_t out,const qqbar_t x,int op)
{
    if(op==0)qqbar_re(out,x);
    else if(op==1)qqbar_im(out,x);
    else if(op==2)qqbar_abs(out,x);
    else if(op==3)qqbar_abs2(out,x);
    else qqbar_sgn(out,x);
}
int main(void)
{
    const slong numerators[]={-7,0,1,5};
    const ulong denominators[]={3,1,1,2};
    const ulong flags[]={0,QQBAR_FORMULA_GAUSSIANS,QQBAR_FORMULA_QUADRATICS,QQBAR_FORMULA_CYCLOTOMICS,QQBAR_FORMULA_DEFLATION,QQBAR_FORMULA_SEPARATION,QQBAR_FORMULA_ALL};
    int k,scale,op,alias,method,part,ok;unsigned long rows=0;
    qqbar_t original,x,y,z;fexpr_t expr;
    qqbar_init(original);qqbar_init(x);qqbar_init(y);qqbar_init(z);fexpr_init(expr);
    for(scale=0;scale<4;scale++)for(k=0;k<24;k++)
    {
        qqbar_exp_pi_i(original,k,12);qqbar_mul_si(original,original,numerators[scale]);qqbar_div_ui(original,original,denominators[scale]);
        for(op=0;op<5;op++)for(alias=0;alias<2;alias++)
        {
            qqbar_set(x,original);
            apply_component(alias?x:y,x,op);
            printf("{\"family\":\"component\",\"scale\":%d,\"k\":%d,\"op\":%d,\"alias\":%d",scale,k,op,alias);
            value(alias?x:y);puts("}");rows++;
        }
        for(alias=0;alias<3;alias++)
        {
            qqbar_set(x,original);
            if(alias==0)qqbar_re_im(y,z,x);
            else if(alias==1)qqbar_re_im(x,z,x);
            else qqbar_re_im(y,x,x);
            for(part=0;part<2;part++)
            {
                printf("{\"family\":\"pair\",\"scale\":%d,\"k\":%d,\"alias\":%d,\"part\":%d",scale,k,alias,part);
                value(part?(alias==2?x:z):(alias==1?x:y));puts("}");rows++;
            }
        }
        for(method=0;method<10;method++)
        {
            ok=1;
            if(method==0)qqbar_get_fexpr_repr(expr,original);
            else if(method==1)qqbar_get_fexpr_root_indexed(expr,original);
            else if(method==2)qqbar_get_fexpr_root_nearest(expr,original);
            else ok=qqbar_get_fexpr_formula(expr,original,flags[method-3]);
            printf("{\"family\":\"roundtrip\",\"scale\":%d,\"k\":%d,\"method\":%d,\"generated\":%d",scale,k,method,ok);
            if(ok){
                qqbar_set_si(x,7);ok=qqbar_set_fexpr(x,expr);printf(",\"parsed\":%d",ok);
                if(ok)value(x);
            }
            puts("}");rows++;
        }
    }
    qqbar_clear(original);qqbar_clear(x);qqbar_clear(y);qqbar_clear(z);fexpr_clear(expr);flint_cleanup();
    if(rows!=2496)abort();
    printf("{\"terminal\":true,\"rows\":%lu}\n",rows);return 0;
}
