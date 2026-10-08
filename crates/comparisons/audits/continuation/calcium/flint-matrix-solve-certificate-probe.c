/* Construct RHS from scalar/integer coefficients, not ca_mat_mul. All owners
   are initialized and shapes are valid. Failed solve outputs are not inspected. */
#include <stdio.h>
#include "ca_mat.h"

static unsigned rows, incorrect, unresolved;
static const char *truth(truth_t x)
{ return x==T_TRUE ? "True" : x==T_FALSE ? "False" : "Unknown"; }

static void base(ca_t b, int kind, ca_ctx_t ctx)
{
    ca_set_ui(b,2,ctx);
    if (kind==0) ca_div_ui(b,b,3,ctx);
    if (kind==1) ca_sqrt(b,b,ctx);
    if (kind==2) ca_log(b,b,ctx);
}

/* Dense triangular integer pattern. X_ij=(i+1)(j+2)/7. For nonunit A,
   B=(integer A*integer X)*base/7; unit mode separates the diagonal term. */
static void recipe(ca_mat_t A, ca_mat_t B, ca_mat_t X, int kind, int upper,
                   int unit, int singular, ca_ctx_t ctx)
{
    ca_t b,t; ca_init(b,ctx); ca_init(t,ctx); base(b,kind,ctx);
    int n=A->r, cols=B->c;
    for (int i=0; i<n; i++) {
        for (int k=0; k<n; k++) {
            if (singular && i==n-1) ca_zero(ca_mat_entry(A,i,k),ctx);
            else if (i==k && unit) ca_one(ca_mat_entry(A,i,k),ctx);
            else if (upper ? k>=i : k<=i)
                ca_mul_si(ca_mat_entry(A,i,k),b,i==k ? i+2 : i+k+1,ctx);
            else ca_zero(ca_mat_entry(A,i,k),ctx);
        }
        for (int j=0; j<cols; j++) {
            ca_set_si(ca_mat_entry(X,i,j),(i+1)*(j+2),ctx);
            ca_div_ui(ca_mat_entry(X,i,j),ca_mat_entry(X,i,j),7,ctx);
            slong sum=0;
            for (int k=0; k<n; k++) if ((upper ? k>=i : k<=i) && !(unit && i==k))
                sum += (i==k ? i+2 : i+k+1)*(k+1)*(j+2);
            ca_mul_si(t,b,sum,ctx); ca_div_ui(t,t,7,ctx);
            if (unit) ca_add(t,t,ca_mat_entry(X,i,j),ctx);
            if (singular && i==n-1) ca_zero(t,ctx);
            ca_set(ca_mat_entry(B,i,j),t,ctx);
        }
    }
    ca_clear(b,ctx); ca_clear(t,ctx);
}

static void one_case(int family,int kind,int n,int cols,int upper,int unit,
                     int singular,int alias,int method,ca_ctx_t ctx)
{
    ca_mat_t A,B,X,Y;
    ca_mat_init(A,n,n,ctx); ca_mat_init(B,n,cols,ctx);
    ca_mat_init(X,n,cols,ctx); ca_mat_init(Y,n,cols,ctx);
    recipe(A,B,X,kind,upper,unit,singular,ctx); ca_mat_ones(Y,ctx);
    ca_mat_struct *out=alias ? B : Y;
    truth_t status=T_TRUE,expected=singular ? T_FALSE : T_TRUE,equal=T_UNKNOWN;
    if (family==0) {
        /* Unit-diagonal mode explicitly ignores the stored diagonal. */
        if (unit) for(int i=0;i<n;i++) ca_set_ui(ca_mat_entry(A,i,i),42,ctx);
        if (upper) {
            if(method==0) ca_mat_solve_triu(out,A,B,unit,ctx);
            if(method==1) ca_mat_solve_triu_classical(out,A,B,unit,ctx);
            if(method==2) ca_mat_solve_triu_recursive(out,A,B,unit,ctx);
        } else {
            if(method==0) ca_mat_solve_tril(out,A,B,unit,ctx);
            if(method==1) ca_mat_solve_tril_classical(out,A,B,unit,ctx);
            if(method==2) ca_mat_solve_tril_recursive(out,A,B,unit,ctx);
        }
    } else {
        if(method==0) status=ca_mat_nonsingular_solve(out,A,B,ctx);
        if(method==1) status=ca_mat_nonsingular_solve_lu(out,A,B,ctx);
        if(method==2) status=ca_mat_nonsingular_solve_fflu(out,A,B,ctx);
        if(method==3) status=ca_mat_nonsingular_solve_adjugate(out,A,B,ctx);
    }
    if (status==T_TRUE && !singular) equal=ca_mat_check_equal(out,X,ctx);
    printf("%s,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s\n",family ? "nonsingular" : "triangular",
      kind,n,cols,upper,unit,singular,alias,method,truth(status),status==T_TRUE&&!singular ? truth(equal) : "na");
    rows++;
    if(status==T_UNKNOWN || (status==T_TRUE&&!singular&&equal==T_UNKNOWN)) unresolved++;
    else if(status!=expected || (status==T_TRUE&&!singular&&equal!=T_TRUE)) incorrect++;
    ca_mat_clear(A,ctx);ca_mat_clear(B,ctx);ca_mat_clear(X,ctx);ca_mat_clear(Y,ctx);
}

int main(void)
{
    ca_ctx_t ctx; ca_ctx_init(ctx);
    const int dims[]={0,1,3,4,9,10,11}, columns[]={0,1,3,9,10,11};
    const int sizes[]={0,1,2,3,4,5,6,10}, rhs[]={0,1,3,10};
    puts("family,kind,n,columns,upper,unit,singular,alias,method,status,equality");
    for(int kind=0;kind<3;kind++)for(unsigned i=0;i<sizeof(dims)/sizeof(*dims);i++)
    for(unsigned j=0;j<sizeof(columns)/sizeof(*columns);j++)for(int upper=0;upper<2;upper++)
    for(int unit=0;unit<2;unit++)for(int alias=0;alias<2;alias++)for(int method=0;method<3;method++)
        one_case(0,kind,dims[i],columns[j],upper,unit,0,alias,method,ctx);
    for(int kind=0;kind<3;kind++)for(unsigned i=0;i<sizeof(sizes)/sizeof(*sizes);i++)
    for(unsigned j=0;j<sizeof(rhs)/sizeof(*rhs);j++)for(int singular=0;singular<2;singular++) {
        if(singular && sizes[i]==0) continue;
        for(int alias=0;alias<2;alias++)for(int method=0;method<4;method++)
            one_case(1,kind,sizes[i],rhs[j],0,0,singular,alias,method,ctx);
    }
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"matrix-solve-certificate\",\"rows\":%u,\"incorrect\":%u,\"unresolved\":%u}\n",rows,incorrect,unresolved);
    return incorrect||unresolved ? 1 : 0;
}
