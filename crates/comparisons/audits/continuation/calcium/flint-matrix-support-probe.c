/* Valid, initialized numerical inputs only. Matrix oracles use integer dot
   coefficients or the closed form for (lambda I + N)^e, never the tested
   matrix multiplication, power or polynomial-evaluation kernels. They share
   the donor scalar backend and are not an independent scalar qualification. */
#include <stdio.h>
#include "ca_mat.h"
#include "ca_poly.h"

static unsigned rows, failures;
static void emit(const char *family, int kind, int r, int inner, int c,
                 int alias, int method, int parameter, truth_t equal)
{
    printf("%s,%d,%d,%d,%d,%d,%d,%d,%s\n", family, kind, r, inner, c,
           alias, method, parameter,
           equal == T_TRUE ? "True" : equal == T_FALSE ? "False" : "Unknown");
    rows++; failures += equal != T_TRUE;
}

static slong integer_entry(int i, int j, int right)
{
    return right ? (i+2)*(j+1) - (i==j) : (i+1)*(j+2) + (i==j);
}

static void fill(ca_mat_t a, const ca_t scale, int right, ca_ctx_t ctx)
{
    for (int i=0; i<a->r; i++) for (int j=0; j<a->c; j++)
        ca_mul_si(ca_mat_entry(a,i,j), scale, integer_entry(i,j,right), ctx);
}

static void multiply_case(int kind, int r, int inner, int c, int alias,
                          int method, ca_ctx_t ctx)
{
    ca_mat_t a,b,out,expected;
    ca_mat_init(a,r,inner,ctx); ca_mat_init(b,inner,c,ctx);
    ca_mat_init(out,r,c,ctx); ca_mat_init(expected,r,c,ctx);
    ca_t base,left,right,product;
    ca_init(base,ctx); ca_init(left,ctx); ca_init(right,ctx); ca_init(product,ctx);
    ca_set_ui(base,kind ? 2 : 1,ctx);
    if (kind == 2) ca_log(base,base,ctx);
    else if (kind) ca_sqrt(base,base,ctx);
    ca_div_ui(left,base,3,ctx);
    if (kind < 3) ca_div_ui(right,base,5,ctx);
    else {
        fmpz_t d; fmpz_init(d); fmpz_one(d);
        fmpz_mul_2exp(d,d,999+kind-3); fmpz_add_ui(d,d,1);
        ca_set_fmpz(right,d,ctx); ca_div(right,base,right,ctx); fmpz_clear(d);
    }
    if (alias == 3) ca_set(right,left,ctx);
    fill(a,left,0,ctx); fill(b,right,alias == 3 ? 0 : 1,ctx);
    ca_mul(product,left,right,ctx);
    for (int i=0; i<r; i++) for (int j=0; j<c; j++) {
        slong sum=0;
        for (int k=0; k<inner; k++)
            sum += integer_entry(i,k,0)*integer_entry(k,j,alias == 3 ? 0 : 1);
        ca_mul_si(ca_mat_entry(expected,i,j),product,sum,ctx);
    }
    ca_mat_ones(out,ctx);
    ca_mat_struct *target = alias == 1 || alias == 3 ? a : alias == 2 ? b : out;
    ca_mat_struct *rhs = alias == 3 ? a : b;
    if (method) ca_mat_mul_classical(target,a,rhs,ctx);
    else ca_mat_mul(target,a,rhs,ctx);
    emit("mul",kind,r,inner,c,alias,method,0,ca_mat_check_equal(target,expected,ctx));
    ca_mat_clear(a,ctx); ca_mat_clear(b,ctx); ca_mat_clear(out,ctx); ca_mat_clear(expected,ctx);
    ca_clear(base,ctx); ca_clear(left,ctx); ca_clear(right,ctx); ca_clear(product,ctx);
}

static ulong binomial(ulong n, ulong k)
{
    ulong v=1;
    for (ulong i=1; i<=k; i++) v=v*(n-i+1)/i;
    return v;
}

/* N has ones on the first superdiagonal: (lambda I + N)^e at distance
   d above the diagonal is binomial(e,d)*lambda^(e-d), with zero for d>e. */
static void jordan_term(ca_t out, const ca_t lambda, int e, int d, ca_ctx_t ctx)
{
    if (d>e) ca_zero(out,ctx);
    else {
        if (e==d) ca_one(out,ctx); else ca_pow_ui(out,lambda,e-d,ctx);
        ca_mul_ui(out,out,binomial(e,d),ctx);
    }
}

static void jordan_case(int kind, int n, int alias, int parameter, int polynomial, ca_ctx_t ctx)
{
    ca_mat_t a,out,expected;
    ca_mat_init(a,n,n,ctx); ca_mat_init(out,n,n,ctx); ca_mat_init(expected,n,n,ctx);
    ca_t lambda,t; ca_init(lambda,ctx); ca_init(t,ctx); ca_set_ui(lambda,2,ctx);
    if (kind==0) ca_zero(lambda,ctx);
    if (kind==1) ca_div_ui(lambda,lambda,3,ctx);
    if (kind==2) ca_sqrt(lambda,lambda,ctx);
    if (kind==3) ca_log(lambda,lambda,ctx);
    for (int i=0; i<n; i++) {
        ca_set(ca_mat_entry(a,i,i),lambda,ctx);
        if (i+1<n) ca_one(ca_mat_entry(a,i,i+1),ctx);
        for (int j=i; j<n; j++) {
            if (!polynomial) jordan_term(ca_mat_entry(expected,i,j),lambda,parameter,j-i,ctx);
            else for (int k=j-i; k<parameter; k++) {
                jordan_term(t,lambda,k,j-i,ctx);
                ca_mul_si(t,t,(k%2 ? -1 : 1)*(k+1),ctx);
                ca_add(ca_mat_entry(expected,i,j),ca_mat_entry(expected,i,j),t,ctx);
            }
        }
    }
    ca_mat_ones(out,ctx);
    ca_mat_struct *target=alias ? a : out;
    if (!polynomial) ca_mat_pow_ui_binexp(target,a,parameter,ctx);
    else {
        ca_poly_t p; ca_poly_init(p,ctx);
        for (int k=0; k<parameter; k++) {
            ca_set_si(t,(k%2 ? -1 : 1)*(k+1),ctx); ca_poly_set_coeff_ca(p,k,t,ctx);
        }
        ca_mat_ca_poly_evaluate(target,p,a,ctx); ca_poly_clear(p,ctx);
    }
    emit(polynomial ? "poly" : "pow",kind,n,n,n,alias,0,parameter,ca_mat_check_equal(target,expected,ctx));
    ca_mat_clear(a,ctx); ca_mat_clear(out,ctx); ca_mat_clear(expected,ctx);
    ca_clear(lambda,ctx); ca_clear(t,ctx);
}

int main(void)
{
    ca_ctx_t ctx; ca_ctx_init(ctx);
    const int dimensions[]={0,1,2,3,4,6,8};
    const int shapes[][3]={{0,4,3},{3,0,4},{3,4,0},{2,3,4},{3,4,5},{5,6,3}};
    const int parameters[]={0,1,2,3,4,7,8,15,16,31};
    puts("family,kind,r,inner,c,alias,method,parameter,equality");
    for (int kind=0; kind<6; kind++) {
        for (unsigned d=0; d<sizeof(dimensions)/sizeof(*dimensions); d++)
            for (int alias=0; alias<4; alias++) for (int method=0; method<2; method++)
                multiply_case(kind,dimensions[d],dimensions[d],dimensions[d],alias,method,ctx);
        for (unsigned s=0; s<sizeof(shapes)/sizeof(*shapes); s++) for (int method=0; method<2; method++)
            multiply_case(kind,shapes[s][0],shapes[s][1],shapes[s][2],0,method,ctx);
    }
    for (int kind=0; kind<4; kind++) for (unsigned d=0; d<sizeof(dimensions)/sizeof(*dimensions); d++)
        for (int alias=0; alias<2; alias++) for (unsigned p=0; p<sizeof(parameters)/sizeof(*parameters); p++)
            for (int polynomial=0; polynomial<2; polynomial++)
                jordan_case(kind,dimensions[d],alias,parameters[p],polynomial,ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"matrix-support\",\"rows\":%u,\"failures\":%u}\n",rows,failures);
    return failures ? 1 : 0;
}
