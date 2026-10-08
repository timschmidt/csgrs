/* Audit-only, bounded finite ball-enclosure controls. Public documented dot
   strides and initial/output aliasing only; no input/output array overlap.
   GMP rational endpoint extrema are the oracle, not an Arb sibling routine.
   All general nonempty cases have a nonzero midpoint product. Exceptional
   all-zero-midpoint/radius-only and huge-exponent branches are not exercised. */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <gmp.h>
#include "arb.h"

#define CAP 10
typedef struct { mpq_t m, r; } ball;
typedef struct { uint64_t calls, failed, exact, trace; } stats;
static const int widths[] = {1,2,3,11,12,13,24,25,26,331,332,333};
static const int lengths[] = {0,1,2,5};
static const char *names[] = {"dot","simple","precise","ui","si","uiui","siui","fmpz"};
static uint64_t inputs, input_failures, fixtures, total_calls, total_failures;
static mpz_t ztmp;
static mpq_t qm, qr, lo, hi, tmp, prod[4], xl, xh, yl, yh;

static void scale(mpq_t q, slong e)
{
    if (e >= 0) mpq_mul_2exp(q,q,(mp_bitcnt_t)e);
    else mpq_div_2exp(q,q,(mp_bitcnt_t)(-e));
}

static int decode_mid(mpq_t q, const arf_t a)
{
    if (arf_is_zero(a)) { mpq_set_ui(q,0,1); return 1; }
    if (!arf_is_finite(a) || COEFF_IS_MPZ(ARF_EXP(a))) return 0;
    nn_srcptr p; slong n;
    ARF_GET_MPN_READONLY(p,n,a);
    mpz_import(mpq_numref(q),(size_t)n,-1,sizeof(ulong),0,0,p);
    mpz_set_ui(mpq_denref(q),1);
    if (ARF_SGNBIT(a)) mpq_neg(q,q);
    scale(q,ARF_EXP(a)-n*FLINT_BITS);
    return 1;
}

static int decode_radius(mpq_t q, const mag_t r)
{
    if (!mag_is_finite(r) || COEFF_IS_MPZ(MAG_EXP(r))) return 0;
    mpq_set_ui(q,MAG_MAN(r),1);
    if (MAG_MAN(r)) scale(q,MAG_EXP(r)-MAG_BITS);
    return 1;
}

static void input_check(const arb_t a, const ball *b)
{
    inputs++;
    input_failures += !decode_mid(qm,arb_midref(a)) || !decode_radius(qr,arb_radref(a))
        || mpq_cmp(qm,b->m) || mpq_cmp(qr,b->r);
}

static uint64_t mix(uint64_t h, uint64_t v)
{
    for (int i=0;i<8;i++) { h ^= v&255; h *= UINT64_C(1099511628211); v >>= 8; }
    return h;
}

static void check(stats *s, const arb_t out, const mpq_t lower, const mpq_t upper)
{
    int ok=decode_mid(qm,arb_midref(out)) && decode_radius(qr,arb_radref(out));
    if (ok) {
        mpq_sub(lo,qm,qr); mpq_add(hi,qm,qr);
        ok=mpq_cmp(lo,lower)<=0 && mpq_cmp(hi,upper)>=0;
    }
    s->calls++; s->failed+=!ok; s->exact+=mpq_sgn(qr)==0;
    /* Trace is supplemental native/Memcheck identity evidence, not an oracle. */
    s->trace=mix(s->trace,mpz_fdiv_ui(mpq_numref(qm),4294967291UL));
    s->trace=mix(s->trace,mpz_fdiv_ui(mpq_denref(qm),4294967279UL));
    s->trace=mix(s->trace,mpz_fdiv_ui(mpq_numref(qr),4294967291UL));
    s->trace=mix(s->trace,mpz_fdiv_ui(mpq_denref(qr),4294967279UL));
    if (!ok) fprintf(stderr,"enclosure mismatch at group call %" PRIu64 "\n",s->calls);
}

static void endpoints(mpq_t lower, mpq_t upper, const ball *b)
{ mpq_sub(lower,b->m,b->r); mpq_add(upper,b->m,b->r); }

static void oracle(mpq_t lower, mpq_t upper, const ball *a, int ai, int as,
    const ball *b, int bi, int bs, int n, const ball *initial, int subtract)
{
    mpq_set_ui(lower,0,1); mpq_set_ui(upper,0,1);
    for (int i=0;i<n;i++) {
        endpoints(xl,xh,a+ai+i*as); endpoints(yl,yh,b+bi+i*bs);
        mpq_mul(prod[0],xl,yl); mpq_mul(prod[1],xl,yh);
        mpq_mul(prod[2],xh,yl); mpq_mul(prod[3],xh,yh);
        int mn=0,mx=0;
        for (int j=1;j<4;j++) {
            if (mpq_cmp(prod[j],prod[mn])<0) mn=j;
            if (mpq_cmp(prod[j],prod[mx])>0) mx=j;
        }
        if (subtract) { mpq_sub(lower,lower,prod[mx]); mpq_sub(upper,upper,prod[mn]); }
        else { mpq_add(lower,lower,prod[mn]); mpq_add(upper,upper,prod[mx]); }
    }
    if (initial) {
        endpoints(xl,xh,initial); mpq_add(lower,lower,xl); mpq_add(upper,upper,xh);
    }
}

static void make_ball(arb_t a, ball *b, int w, int family, int i, int side)
{
    const int gaps[]={0,1,63,64,65,127,128,129,257,511};
    int k=family==4 ? 0 : i;
    mpz_set_ui(ztmp,0);
    if (family==0 || family==4) {
        mpz_setbit(ztmp,(mp_bitcnt_t)w*64); mpz_sub_ui(ztmp,ztmp,1);
    } else {
        for (int j=0;j<w;j++) {
            uint64_t v=UINT64_C(0x9e3779b97f4a7c15)*(uint64_t)(j+1+17*side+31*k);
            v ^= UINT64_C(0xd1b54a32d192ed03)*(uint64_t)(family+1);
            mpz_mul_2exp(ztmp,ztmp,64); mpz_add_ui(ztmp,ztmp,(ulong)v);
        }
        mpz_setbit(ztmp,(mp_bitcnt_t)w*64-1); mpz_setbit(ztmp,0);
    }
    if ((family==4 ? (side==0 && (i&1)) : ((i+side+family)&1))) mpz_neg(ztmp,ztmp);
    if (family==5 && side==0 && i>0 && i%3==1) mpz_set_ui(ztmp,0);
    slong e=-w*64-(family==4 ? 0 : gaps[(i+side)%10]);
    arf_set_mpz(arb_midref(a),ztmp); arf_mul_2exp_si(arb_midref(a),arb_midref(a),e);
    mpq_set_z(b->m,ztmp); scale(b->m,e);
    ulong r=(family==2 || family==3 || (family==5 && (i+side)%2)) ? (ulong)(1+(i+side)%3) : 0;
    slong re=family==3 ? (i%3-2) : -2*w*64-(i%3);
    mag_set_ui_2exp_si(arb_radref(a),r,re);
    mpq_set_ui(b->r,r,1); scale(b->r,re);
    input_check(a,b);
}

static void emit(int type,int w,int family,int n,int op,const stats *s)
{
    printf("{\"kind\":\"group\",\"type\":%d,\"width\":%d,\"family\":%d,\"length\":%d,"
        "\"op\":\"%s\",\"calls\":%" PRIu64 ",\"failed\":%" PRIu64
        ",\"exact\":%" PRIu64 ",\"trace\":\"%016" PRIx64 "\"}\n",
        type,w,family,n,names[op],s->calls,s->failed,s->exact,s->trace);
    total_calls+=s->calls; total_failures+=s->failed;
}

int main(void)
{
    if (FLINT_BITS!=64 || sizeof(ulong)!=8 || GMP_NUMB_BITS!=64) return 2;
    mpz_init(ztmp); mpq_inits(qm,qr,lo,hi,tmp,xl,xh,yl,yh,NULL);
    for (int i=0;i<4;i++) mpq_init(prod[i]);
    arb_struct x[CAP],y[CAP]; ball a[CAP],b[CAP],initial;
    fmpz integers[CAP]; ulong ui[CAP],wide[2*CAP]; slong si[CAP];
    arb_t init,out; arb_init(init); arb_init(out);
    mpq_inits(initial.m,initial.r,NULL);
    mpq_set_ui(initial.m,3,8); mpq_set_ui(initial.r,1,1024);
    arf_set_si(arb_midref(init),3); arf_mul_2exp_si(arb_midref(init),arb_midref(init),-3);
    mag_set_ui_2exp_si(arb_radref(init),1,-10); input_check(init,&initial);
    for (int i=0;i<CAP;i++) {
        arb_init(x+i); arb_init(y+i); fmpz_init(integers+i);
        mpq_inits(a[i].m,a[i].r,b[i].m,b[i].r,NULL);
    }
    mpq_t lower,upper; mpq_inits(lower,upper,NULL);
    for (int type=0;type<6;type++)
    for (size_t wi=0;wi<sizeof(widths)/sizeof(widths[0]);wi++) {
        if (type && wi!=0 && wi!=2 && wi!=8) continue;
        int w=widths[wi];
        for (int family=0;family<6;family++) {
            for (int i=0;i<CAP;i++) {
                make_ball(x+i,a+i,w,family,i,0);
                make_ball(y+i,b+i,w,family,i,1);
                if (type) {
                    const int bits[]={1,60,64,65,127,128,129,191,192,193,257};
                    if (type<=2) {
                        ui[i]=i==3 ? 0 : (UWORD(1)<<((family*11+i)%61))+(ulong)(i+1);
                        si[i]=(slong)ui[i]; if ((family+i)&1) si[i]=-si[i];
                        if (type==1) mpz_set_ui(ztmp,ui[i]); else mpz_set_si(ztmp,si[i]);
                    } else if (type<=4) {
                        wide[2*i]=(i%3==1) ? 0 : UINT64_C(0xd1b54a32d192ed03)+(ulong)i;
                        wide[2*i+1]=(i%3==0) ? 0 : UINT64_C(0x1e3779b97f4a7c15)+(ulong)family;
                        if (type==4 && ((i+family)&1)) {
                            wide[2*i+1]=-wide[2*i+1]-(wide[2*i]!=0); wide[2*i]=-wide[2*i];
                        }
                        mpz_import(ztmp,2,-1,sizeof(ulong),0,0,wide+2*i);
                        if (type==4 && (wide[2*i+1]>>63)) {
                            mpz_set_ui(mpq_numref(tmp),0); mpz_setbit(mpq_numref(tmp),128);
                            mpz_sub(ztmp,ztmp,mpq_numref(tmp));
                        }
                    } else {
                        mpz_set_ui(ztmp,0); mpz_setbit(ztmp,(mp_bitcnt_t)bits[(family+i)%11]);
                        mpz_sub_ui(ztmp,ztmp,(i%2)?1:0);
                        if ((family+i)&1) mpz_neg(ztmp,ztmp);
                    }
                    fmpz_set_mpz(integers+i,ztmp);
                    mpq_set_z(b[i].m,ztmp); mpq_set_ui(b[i].r,0,1);
                    arf_set_mpz(arb_midref(y+i),ztmp); mag_zero(arb_radref(y+i)); input_check(y+i,b+i);
                }
            }
            for (size_t li=0;li<sizeof(lengths)/sizeof(lengths[0]);li++) {
                int n=lengths[li]; fixtures++;
                stats s[8]={{0}};
                for (int j=0;j<8;j++) s[j].trace=UINT64_C(14695981039346656037);
                slong precisions[]={2,63,64,65,127,128,129,w*64,w*128+1};
                for (int layout=0;layout<3;layout++) {
                    int xs=layout==0?1:layout==1?-1:2, ys=layout==2?-2:1;
                    int xi=n>0 && xs<0 ? (n-1)*(-xs):0, yi=n>0 && ys<0 ? (n-1)*(-ys):0;
                    /* Safety/semantic guard: do not enter a radius-only midpoint sum. */
                    int nonzero=n==0;
                    for (int i=0;i<n;i++) nonzero|=mpq_sgn(a[xi+i*xs].m)!=0 && mpq_sgn(b[yi+i*ys].m)!=0;
                    if (!nonzero) return 3;
                    for (int subtract=0;subtract<2;subtract++)
                    for (int im=0;im<3;im++) {
                        oracle(lower,upper,a,xi,xs,b,yi,ys,n,im?&initial:NULL,subtract);
                        for (size_t p=0;p<sizeof(precisions)/sizeof(precisions[0]);p++)
                        for (int op=type?type+2:0;op<(type?type+3:3);op++) {
                            arb_set(out,im==2 ? init : x);
                            arb_srcptr initial_ptr=im==0?NULL:im==1?init:out;
                            slong prec=precisions[p];
                            if (op==0) arb_dot(out,initial_ptr,subtract,x+xi,xs,y+yi,ys,n,prec);
                            if (op==1) arb_dot_simple(out,initial_ptr,subtract,x+xi,xs,y+yi,ys,n,prec);
                            if (op==2) arb_dot_precise(out,initial_ptr,subtract,x+xi,xs,y+yi,ys,n,prec);
                            if (op==3) arb_dot_ui(out,initial_ptr,subtract,x+xi,xs,ui+yi,ys,n,prec);
                            if (op==4) arb_dot_si(out,initial_ptr,subtract,x+xi,xs,si+yi,ys,n,prec);
                            if (op==5) arb_dot_uiui(out,initial_ptr,subtract,x+xi,xs,wide+2*yi,ys,n,prec);
                            if (op==6) arb_dot_siui(out,initial_ptr,subtract,x+xi,xs,wide+2*yi,ys,n,prec);
                            if (op==7) arb_dot_fmpz(out,initial_ptr,subtract,x+xi,xs,integers+yi,ys,n,prec);
                            check(s+op,out,lower,upper);
                        }
                    }
                }
                for (int op=type?type+2:0;op<(type?type+3:3);op++) emit(type,w,family,n,op,s+op);
                for (int i=0;i<CAP;i++) {
                    input_check(x+i,a+i); input_check(y+i,b+i);
                    if (type) {
                        fmpz_get_mpz(ztmp,integers+i); inputs++;
                        input_failures+=mpz_cmp(ztmp,mpq_numref(b[i].m))!=0;
                    }
                }
                input_check(init,&initial);
            }
        }
    }
    for (int i=0;i<CAP;i++) {
        arb_clear(x+i); arb_clear(y+i); fmpz_clear(integers+i);
        mpq_clears(a[i].m,a[i].r,b[i].m,b[i].r,NULL);
    }
    arb_clear(init); arb_clear(out); mpq_clears(initial.m,initial.r,lower,upper,NULL);
    mpz_clear(ztmp); mpq_clears(qm,qr,lo,hi,tmp,xl,xh,yl,yh,NULL);
    for (int i=0;i<4;i++) mpq_clear(prod[i]);
    flint_cleanup_master();
    printf("{\"kind\":\"summary\",\"fixtures\":%" PRIu64 ",\"calls\":%" PRIu64
        ",\"endpointComparisons\":%" PRIu64 ",\"failures\":%" PRIu64
        ",\"inputChecks\":%" PRIu64 ",\"inputFailures\":%" PRIu64 "}\n",
        fixtures,total_calls,2*total_calls,total_failures,inputs,input_failures);
    return total_failures || input_failures ? 1 : 0;
}
