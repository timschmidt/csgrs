/* Reuse the frozen, independently cross-checked GMP rounding/decoding oracle.
   Its old main is not executed. All new operands and storage are bounded. */
#define main arf_rounding_previous_main
#include "flint-arf-rounding-controls.c"
#undef main

#define FUSED_OPS 24
static const int lengths[] = {1,2,3,19,20,21,111,112,113,249,250,251};
static const int signs_selected[] = {0,1,6,15};
static const char *fused_names[FUSED_OPS] = {
 "complex-re","complex-im","fallback-re","fallback-im",
 "complex-left-re","complex-left-im","complex-right-re","complex-right-im",
 "square-re","square-im","square-inplace-re","square-inplace-im",
 "fma","fma-x","fma-initial","addmul","submul","sosq",
 "add","sub","sum","sum-reversed","dot","dot-initial-sub"};
static stats fused_total[FUSED_OPS];

static void fixture(mpz_t *v, int n, int family, int signs)
{
    mpz_t t; mpz_init(t);
    mpz_setbit(t, (mp_bitcnt_t)n*64-1);
    mpz_setbit(t, (mp_bitcnt_t)n*32);
    mpz_add_ui(t,t,3);
    mpz_add_ui(v[0],t,1); mpz_set(v[1],t);
    mpz_sub_ui(v[2],t,1); mpz_set(v[3],t);
    if (family==1) { mpz_neg(v[2],t); mpz_sub_ui(v[3],t,1); }
    if (family==2) { mpz_set(v[2],t); mpz_add_ui(v[3],t,1); }
    if (family==3) { mpz_set_ui(v[0],0); mpz_set_ui(v[3],0); }
    if ((family>=4 && family<=6) || family>=10) {
        int gap=family==4?63:family==5?64:family==6?65:family==10?129:257;
        mpz_mul_2exp(v[1],v[1],(mp_bitcnt_t)gap);
        mpz_mul_2exp(v[2],v[2],(mp_bitcnt_t)gap);
    }
    if (family==7 || family==8) {
        int short_n=n-(family==7?3:2); if(short_n<1)short_n=1;
        mpz_set_ui(v[1],0); mpz_setbit(v[1],(mp_bitcnt_t)short_n*64-1);
        mpz_add_ui(v[1],v[1],5); mpz_add_ui(v[3],v[1],2);
    }
    if (family==9)for(int i=0;i<4;i++)mpz_set_ui(v[i],0);
    for(int i=0;i<4;i++)if(signs&(1<<i))mpz_neg(v[i],v[i]);
    mpz_clear(t);
}

static void set_scaled(arf_t x,const mpz_t a,slong exponent)
{
    arf_set_mpz(x,a); arf_mul_2exp_si(x,x,exponent);
}

static void emit_fused(const char *kind,int n,int family,int signs,int op,const stats *s)
{
    printf("{\"kind\":\"%s\",\"n\":%d,\"family\":%d,\"sign\":%d,\"op\":\"%s\","
        "\"count\":%" PRIu64 ",\"exact\":%" PRIu64 ",\"tieEven\":%" PRIu64
        ",\"tieOdd\":%" PRIu64 ",\"carry\":%" PRIu64 ",\"failures\":%" PRIu64
        ",\"trace\":\"%016" PRIx64 "\"}\n",kind,n,family,signs,fused_names[op],s->count,s->exact,
        s->tie_even,s->tie_odd,s->carry,s->failures,s->trace);
}

int main(void)
{
    if(FLINT_BITS!=64 || sizeof(ulong)!=8 || GMP_NUMB_BITS!=64)return 2;
    mpz_inits(magnitude,quotient,remainder,half,expected,decoded,scratch,NULL);
    mpz_t v[4],ref[10],ab,initial,ac,bd,aa,bb,one,minus_one;
    for(int i=0;i<4;i++)mpz_init(v[i]);
    for(int i=0;i<10;i++)mpz_init(ref[i]);
    mpz_inits(ab,initial,ac,bd,aa,bb,one,minus_one,NULL);
    mpz_set_ui(one,1);mpz_set_si(minus_one,-1);
    arf_struct x[4],terms[4],reverse[4],dx[2],dy[2];
    arf_t init,e,f;
    for(int i=0;i<4;i++){arf_init(x+i);arf_init(terms+i);arf_init(reverse+i);}
    for(int i=0;i<2;i++){arf_init(dx+i);arf_init(dy+i);}
    arf_init(init);arf_init(e);arf_init(f);
    uint64_t flag_failures=0;
    for(size_t ni=0;ni<sizeof(lengths)/sizeof(lengths[0]);ni++)
    for(int family=0;family<12;family++)
    for(size_t si=0;si<sizeof(signs_selected)/sizeof(signs_selected[0]);si++)
    {
        int n=lengths[ni],signs=signs_selected[si];
        slong unit_exp=(family%3-1)*193,product_exp=2*unit_exp;
        fixture(v,n,family,signs);
        mpz_mul(ab,v[0],v[1]);mpz_set(initial,ab);
        if(family%2==0)mpz_neg(initial,initial);
        mpz_add_ui(initial,initial,(ulong)(family%3+1));
        mpz_mul(ac,v[0],v[2]);mpz_mul(bd,v[1],v[3]);
        mpz_sub(ref[0],ac,bd);
        mpz_mul(ref[1],v[0],v[3]);mpz_mul(scratch,v[1],v[2]);mpz_add(ref[1],ref[1],scratch);
        mpz_mul(aa,v[0],v[0]);mpz_mul(bb,v[1],v[1]);
        mpz_sub(ref[2],aa,bb);mpz_mul_2exp(ref[3],ab,1);
        mpz_add(ref[4],initial,ab);mpz_sub(ref[5],initial,ab);
        mpz_add(ref[6],aa,bb);mpz_add(ref[7],v[0],v[1]);mpz_sub(ref[8],v[0],v[1]);
        mpz_sub(ref[9],initial,ref[0]);
        for(int i=0;i<4;i++)set_scaled(x+i,v[i],unit_exp);
        set_scaled(init,initial,product_exp);
        set_scaled(terms,ab,product_exp);set_scaled(terms+1,initial,product_exp);
        set_scaled(terms+2,one,product_exp);set_scaled(terms+3,minus_one,product_exp);
        for(int i=0;i<4;i++)arf_set(reverse+i,terms+3-i);
        arf_set(dx,x);arf_set(dx+1,x+1);arf_set(dy,x+2);arf_neg(dy+1,x+3);
        stats group[FUSED_OPS]={{0}};
        for(int op=0;op<FUSED_OPS;op++)group[op].trace=UINT64_C(14695981039346656037);
        for(int pass=0;pass<2;pass++)
        {
            for(int i=0;i<4;i++)input_check(x+i,v[i],unit_exp);
            input_check(init,initial,product_exp);
            const mpz_t *tv[4]={&ab,&initial,&one,&minus_one};
            for(int i=0;i<4;i++){
                input_check(terms+i,*tv[i],product_exp);
                input_check(reverse+i,*tv[3-i],product_exp);
            }
            input_check(dx,v[0],unit_exp);input_check(dx+1,v[1],unit_exp);
            input_check(dy,v[2],unit_exp);
            mpz_neg(ac,v[3]);input_check(dy+1,ac,unit_exp);
            if(pass)break;
            slong precisions[]={1,2,3,53,63,64,65,127,128,129,2*n*64,ARF_PREC_EXACT};
            for(size_t pi=0;pi<sizeof(precisions)/sizeof(precisions[0]);pi++)
            for(int mode=0;mode<5;mode++)
            {
                slong p=precisions[pi];arf_rnd_t rnd=(arf_rnd_t)mode;int r;
                for(int route=0;route<6;route++)
                {
                    arf_set(e,terms);arf_set(f,terms);
                    if(route==0)r=arf_complex_mul(e,f,x,x+1,x+2,x+3,p,rnd);
                    else if(route==1)r=arf_complex_mul_fallback(e,f,x,x+1,x+2,x+3,p,rnd);
                    else if(route==2){arf_set(e,x);arf_set(f,x+1);r=arf_complex_mul(e,f,e,f,x+2,x+3,p,rnd);}
                    else if(route==3){arf_set(e,x+2);arf_set(f,x+3);r=arf_complex_mul(e,f,x,x+1,e,f,p,rnd);}
                    else if(route==4)r=arf_complex_sqr(e,f,x,x+1,p,rnd);
                    else {arf_set(e,x);arf_set(f,x+1);r=arf_complex_sqr(e,f,e,f,p,rnd);}
                    flag_failures+=(r<0 || r>3);
                    check(&group[2*route],e,r&1,ref[route<4?0:2],product_exp,p,rnd);
                    check(&group[2*route+1],f,r>>1,ref[route<4?1:3],product_exp,p,rnd);
                }
                for(int op=12;op<FUSED_OPS;op++)
                {
                    arf_set(e,terms);
                    if(op==12)r=arf_fma(e,x,x+1,init,p,rnd);
                    else if(op==13){arf_set(e,x);r=arf_fma(e,e,x+1,init,p,rnd);}
                    else if(op==14){arf_set(e,init);r=arf_fma(e,x,x+1,e,p,rnd);}
                    else if(op==15){arf_set(e,init);r=arf_addmul(e,x,x+1,p,rnd);}
                    else if(op==16){arf_set(e,init);r=arf_submul(e,x,x+1,p,rnd);}
                    else if(op==17)r=arf_sosq(e,x,x+1,p,rnd);
                    else if(op==18)r=arf_add(e,x,x+1,p,rnd);
                    else if(op==19)r=arf_sub(e,x,x+1,p,rnd);
                    else if(op==20)r=arf_sum(e,terms,4,p,rnd);
                    else if(op==21)r=arf_sum(e,reverse,4,p,rnd);
                    else if(op==22)r=arf_dot(e,NULL,0,dx,1,dy,1,2,p,rnd);
                    else r=arf_dot(e,init,1,dx,1,dy,1,2,p,rnd);
                    int index=op<=15?4:op==16?5:op==17?6:op==18?7:op==19?8:op<=21?4:op==22?0:9;
                    flag_failures+=(r<0 || r>1);
                    check(&group[op],e,r,ref[index],op==18||op==19?unit_exp:product_exp,p,rnd);
                }
            }
        }
        for(int op=0;op<FUSED_OPS;op++){
            emit_fused("group",n,family,signs,op,&group[op]);
            fused_total[op].count+=group[op].count;fused_total[op].exact+=group[op].exact;
            fused_total[op].tie_even+=group[op].tie_even;fused_total[op].tie_odd+=group[op].tie_odd;
            fused_total[op].carry+=group[op].carry;fused_total[op].failures+=group[op].failures;
        }
    }
    uint64_t outputs=0,failures=input_failures+flag_failures;
    for(int op=0;op<FUSED_OPS;op++){
        emit_fused("total",-1,-1,-1,op,&fused_total[op]);outputs+=fused_total[op].count;failures+=fused_total[op].failures;
    }
    printf("{\"kind\":\"summary\",\"outputs\":%" PRIu64 ",\"inputChecks\":%" PRIu64
        ",\"inputFailures\":%" PRIu64 ",\"flagFailures\":%" PRIu64 ",\"failures\":%" PRIu64 "}\n",
        outputs,input_checks,input_failures,flag_failures,failures);
    for(int i=0;i<4;i++){mpz_clear(v[i]);arf_clear(x+i);arf_clear(terms+i);arf_clear(reverse+i);}
    for(int i=0;i<10;i++)mpz_clear(ref[i]);
    for(int i=0;i<2;i++){arf_clear(dx+i);arf_clear(dy+i);}
    arf_clear(init);arf_clear(e);arf_clear(f);
    mpz_clears(ab,initial,ac,bd,aa,bb,one,minus_one,magnitude,quotient,remainder,half,expected,decoded,scratch,NULL);
    flint_cleanup_master();
    return failures?1:0;
}
