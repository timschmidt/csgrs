/* Known spectra, with exact coefficient oracle independent of charpoly,
   determinant and matrix-polynomial evaluation. Scalar backend remains shared. */
#include <stdio.h>
#include "ca_mat.h"
#include "ca_poly.h"

static unsigned rows,incorrect,unresolved;
static const char *truth(truth_t x)
{ return x==T_TRUE ? "True" : x==T_FALSE ? "False" : "Unknown"; }
static void base(ca_t b,int kind,ca_ctx_t ctx)
{
    ca_set_ui(b,2,ctx);
    if(kind==0) ca_div_ui(b,b,3,ctx);
    if(kind==1) ca_sqrt(b,b,ctx);
    if(kind==2) ca_log(b,b,ctx);
}

/* S=I+uv^T, u=all ones, sum(v)=0 => S^-1=I-uv^T. With
   D=diag(1,...,n), (S D S^-1)_ij = D_ij+v_j(j-i)-v_j sum_k v_k(k+1).
   Shapes include triangular blocks and dense similarity; family3 reverses the
   basis ordering of family2. No matrix inverse/multiply is used to build A. */
static void recipe(ca_mat_t A,int kind,int family,ca_ctx_t ctx)
{
    int n=A->r; slong v[12]={0},sum=0,weighted=0;
    for(int i=0;i+1<n;i++){v[i]=i%2 ? -1 : 1;sum+=v[i];}
    if(n) v[n-1]=-sum;
    for(int i=0;i<n;i++) weighted+=v[i]*(i+1);
    ca_t b;ca_init(b,ctx);base(b,kind,ctx);
    for(int i=0;i<n;i++)for(int j=0;j<n;j++) {
        int r=family==3 ? n-i-1 : i,c=family==3 ? n-j-1 : j;
        slong coefficient=r==c ? r+1 : 0;
        if(family==1 && r==c+1) coefficient=1;
        if(family>=2) coefficient+=v[c]*(c-r-weighted);
        ca_mul_si(ca_mat_entry(A,i,j),b,coefficient,ctx);
    }
    ca_clear(b,ctx);
}

static void expected(ca_poly_t p,int kind,int n,ca_ctx_t ctx)
{
    slong coefficients[12]={1};
    for(int root=1;root<=n;root++) {
        coefficients[root]=coefficients[root-1];
        for(int k=root-1;k>=1;k--) coefficients[k]=coefficients[k-1]-root*coefficients[k];
        coefficients[0]*=-root;
    }
    ca_t b,t;ca_init(b,ctx);ca_init(t,ctx);base(b,kind,ctx);
    for(int k=0;k<=n;k++) {
        ca_pow_ui(t,b,n-k,ctx);ca_mul_si(t,t,coefficients[k],ctx);
        ca_poly_set_coeff_ca(p,k,t,ctx);
    }
    ca_clear(b,ctx);ca_clear(t,ctx);
}

static void one_case(int kind,int n,int family,int seed,int method,ca_ctx_t ctx)
{
    ca_mat_t A;ca_mat_init(A,n,n,ctx);recipe(A,kind,family,ctx);
    ca_poly_t p,e;ca_poly_init(p,ctx);ca_poly_init(e,ctx);expected(e,kind,n,ctx);
    if(seed) {
        ca_t t;ca_init(t,ctx);ca_set_ui(t,2,ctx);ca_log(t,t,ctx);
        for(int k=0;k<17;k++) ca_poly_set_coeff_ca(p,k,t,ctx);
        ca_clear(t,ctx);
    }
    int success=1;
    if(method==0) ca_mat_charpoly(p,A,ctx);
    else success=ca_mat_charpoly_danilevsky(p,A,ctx);
    truth_t equal=success ? ca_poly_check_equal(p,e,ctx) : T_UNKNOWN;
    printf("charpoly,%d,%d,%d,%d,%d,%d,%s\n",kind,n,family,seed,method,success,success ? truth(equal) : "na");
    rows++;
    if(!success || equal==T_UNKNOWN) unresolved++;
    else if(equal!=T_TRUE || p->length!=n+1) incorrect++;
    ca_mat_clear(A,ctx);ca_poly_clear(p,ctx);ca_poly_clear(e,ctx);
}

int main(void)
{
    ca_ctx_t ctx;ca_ctx_init(ctx);
    const int dims[]={0,1,2,3,4,5,6,8,10};
    puts("family,kind,n,shape,seed,method,success,equality");
    for(int kind=0;kind<3;kind++)for(unsigned d=0;d<sizeof(dims)/sizeof(*dims);d++)
    for(int family=0;family<4;family++)for(int seed=0;seed<2;seed++)for(int method=0;method<2;method++)
        one_case(kind,dims[d],family,seed,method,ctx);
    ca_ctx_clear(ctx);flint_cleanup();
    printf("{\"suite\":\"charpoly-certificate\",\"rows\":%u,\"incorrect\":%u,\"unresolved\":%u}\n",rows,incorrect,unresolved);
    return incorrect||unresolved ? 1 : 0;
}
