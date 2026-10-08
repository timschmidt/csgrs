/* Numerical output controls only: valid dimensions, initialized owners and
   whole-input aliases. No invalid pointers or failed-rank state are exercised. */
#include <stdio.h>
#include "ca_mat.h"

static const char *truth(truth_t x)
{
    return x == T_TRUE ? "True" : x == T_FALSE ? "False" : "Unknown";
}

static void base(ca_t b, int kind, ca_ctx_t ctx)
{
    ca_set_ui(b, kind ? 2 : 1, ctx);
    if (kind == 1) ca_sqrt(b, b, ctx);
    if (kind == 2) ca_log(b, b, ctx);
}

static void triangular(ca_mat_t A, const ca_t b, ca_ctx_t ctx)
{
    ca_mat_zero(A, ctx);
    for (slong i = 0; i < A->r; i++) {
        ca_mul_ui(ca_mat_entry(A,i,i), b, i+2, ctx);
        if (i+1 < A->r) ca_set(ca_mat_entry(A,i,i+1), b, ctx);
    }
}

/* For the integer upper-bidiagonal B with B_ii=i+2 and B_i,i+1=1,
   inverse_ij=(-1)^(j-i)/product(i+2..j+2), and det(B)=product(2..n+1).
   This oracle does not call any determinant, adjugate or inverse matrix kernel. */
static void expected(ca_mat_t X, ca_t det, const ca_t b, int adj, ca_ctx_t ctx)
{
    slong n = X->r;
    ulong d = 1;
    for (slong i = 0; i < n; i++) d *= (ulong)(i+2);
    ca_pow_ui(det, b, n, ctx); ca_mul_ui(det, det, d, ctx);
    ca_t scale;
    ca_init(scale, ctx);
    if (adj && n > 0) ca_pow_ui(scale, b, n-1, ctx);
    else ca_inv(scale, b, ctx);
    ca_mat_zero(X, ctx);
    for (slong i = 0; i < n; i++) {
        ulong product = 1;
        for (slong j = i; j < n; j++) {
            product *= (ulong)(j+2);
            ca_set_si(ca_mat_entry(X,i,j), (j-i)%2 ? -1 : 1, ctx);
            ca_div_ui(ca_mat_entry(X,i,j), ca_mat_entry(X,i,j), product, ctx);
            if (adj) ca_mul_ui(ca_mat_entry(X,i,j), ca_mat_entry(X,i,j), d, ctx);
            ca_mul(ca_mat_entry(X,i,j), ca_mat_entry(X,i,j), scale, ctx);
        }
    }
    ca_clear(scale, ctx);
}

int main(void)
{
    ca_ctx_t ctx; ca_ctx_init(ctx);
    unsigned rows = 0, failures = 0;
    puts("family,kind,r,c,alias,method,success,rank_or_det,equality");
    for (int r=0;r<=6;r++) for (int c=0;c<=6;c++) for (int seed=0;seed<3;seed++)
    for (int alias=0;alias<2;alias++) for (int method=0;method<3;method++) {
        ca_mat_t A,R; ca_mat_init(A,r,c,ctx); ca_mat_init(R,r,c,ctx);
        if (seed) for (int i=0;i<r;i++) for (int j=0;j<c;j++) {
            ca_set_ui(ca_mat_entry(R,i,j), seed, ctx);
            if (seed==2) ca_log(ca_mat_entry(R,i,j),ca_mat_entry(R,i,j),ctx);
        }
        if (alias) ca_mat_zero(R,ctx);
        slong rank=-1;
        ca_mat_struct *input = alias ? R : A;
        int success = method==0 ? ca_mat_rref(&rank,R,input,ctx)
          : method==1 ? ca_mat_rref_lu(&rank,R,input,ctx)
          : ca_mat_rref_fflu(&rank,R,input,ctx);
        truth_t equal=ca_mat_check_is_zero(R,ctx);
        printf("rref,%d,%d,%d,%d,%d,%d,%ld,%s\n",seed,r,c,alias,method,success,(long)rank,truth(equal));
        rows++; failures += success!=1 || rank!=0 || equal!=T_TRUE;
        ca_mat_clear(A,ctx); ca_mat_clear(R,ctx);
    }
    for (int kind=0;kind<3;kind++) for (int n=0;n<=7;n++) for (int alias=0;alias<2;alias++) {
        ca_mat_t A,X,E; ca_mat_init(A,n,n,ctx); ca_mat_init(X,n,n,ctx); ca_mat_init(E,n,n,ctx);
        ca_t b,d,e; ca_init(b,ctx);ca_init(d,ctx);ca_init(e,ctx);base(b,kind,ctx);
        for (int method=0;method<3;method++) {
            triangular(A,b,ctx); ca_mat_ones(X,ctx); expected(E,e,b,1,ctx);
            ca_mat_struct *out=alias ? A : X;
            if (method==0) ca_mat_adjugate(out,d,A,ctx);
            else if (method==1) ca_mat_adjugate_cofactor(out,d,A,ctx);
            else ca_mat_adjugate_charpoly(out,d,A,ctx);
            truth_t de=ca_check_equal(d,e,ctx), equal=ca_mat_check_equal(out,E,ctx);
            printf("adjugate,%d,%d,%d,%d,%d,1,%s,%s\n",kind,n,n,alias,method,truth(de),truth(equal));
            rows++; failures += de!=T_TRUE || equal!=T_TRUE;
        }
        triangular(A,b,ctx); ca_mat_ones(X,ctx); expected(E,e,b,0,ctx);
        ca_mat_struct *out=alias ? A : X;
        truth_t success=ca_mat_inv(out,A,ctx), equal=ca_mat_check_equal(out,E,ctx);
        printf("inverse,%d,%d,%d,%d,0,%s,na,%s\n",kind,n,n,alias,truth(success),truth(equal));
        rows++; failures += success!=T_TRUE || equal!=T_TRUE;
        ca_mat_clear(A,ctx);ca_mat_clear(X,ctx);ca_mat_clear(E,ctx);
        ca_clear(b,ctx);ca_clear(d,ctx);ca_clear(e,ctx);
    }
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"matrix-output\",\"rows\":%u,\"failures\":%u}\n",rows,failures);
    return failures ? 1 : 0;
}
