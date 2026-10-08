#include "spectral-corpus.h"

static truth_t match_blocks(const ca_vec_t lambda,slong count,const slong *indices,
                           const slong *sizes,int kind,int pattern,ca_ctx_t ctx)
{
    if(count!=corpus_count[pattern])return T_FALSE;
    int used[3]={0};ca_t b,e;ca_init(b,ctx);ca_init(e,ctx);corpus_base(b,kind,ctx);
    truth_t result=T_TRUE;
    for(int i=0;i<count;i++) {
        if(indices[i]<0||indices[i]>=lambda->length||sizes[i]<=0){result=T_FALSE;break;}
        int found=0,unknown=0;
        for(int j=0;j<count;j++)if(!used[j]&&sizes[i]==corpus_sizes[pattern][j]) {
            ca_add_ui(e,b,corpus_labels[pattern][j],ctx);
            truth_t eq=ca_check_equal(ca_vec_entry(lambda,indices[i]),e,ctx);
            if(eq==T_TRUE){used[j]=1;found=1;break;}unknown|=eq==T_UNKNOWN;
        }
        if(!found){result=unknown?T_UNKNOWN:T_FALSE;break;}
    }
    ca_clear(b,ctx);ca_clear(e,ctx);return result;
}
static truth_t match_form(const ca_mat_t J,int kind,int pattern,ca_ctx_t ctx)
{
    int n=J->r;ca_vec_t lambda;ca_vec_init(lambda,n,ctx);
    slong indices[SPECTRAL_MAX]={0},sizes[SPECTRAL_MAX]={0},count=0;
    truth_t result=T_TRUE;
    for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(j!=i&&j!=i+1) {
        truth_t zero=ca_check_is_zero(ca_mat_entry(J,i,j),ctx);
        if(zero!=T_TRUE){result=zero==T_FALSE?T_FALSE:T_UNKNOWN;goto cleanup;}
    }
    for(int i=0;i<n;) {
        int start=i;indices[count]=count;ca_set(ca_vec_entry(lambda,count),ca_mat_entry(J,i,i),ctx);
        do {
            truth_t eq=ca_check_equal(ca_mat_entry(J,i,i),ca_mat_entry(J,start,start),ctx);
            if(eq!=T_TRUE){result=eq;goto cleanup;}
            i++;
            if(i==n)break;
            truth_t zero=ca_check_is_zero(ca_mat_entry(J,i-1,i),ctx);
            if(zero==T_TRUE)break;
            truth_t one=ca_check_is_one(ca_mat_entry(J,i-1,i),ctx);
            if(one!=T_TRUE){result=one;goto cleanup;}
        } while(i<n);
        sizes[count++]=i-start;
    }
    result=match_blocks(lambda,count,indices,sizes,kind,pattern,ctx);
cleanup:
    ca_vec_clear(lambda,ctx);return result;
}
/* Independent exact rational determinant; no CA inverse or determinant oracle. */
static truth_t invertible(const ca_mat_t P,ca_ctx_t ctx)
{
    fmpq_mat_t q;fmpq_mat_init(q,P->r,P->c);fmpq_t d;fmpq_init(d);truth_t result=T_UNKNOWN;
    for(int i=0;i<P->r;i++)for(int j=0;j<P->c;j++)
        if(!ca_get_fmpq(fmpq_mat_entry(q,i,j),ca_mat_entry(P,i,j),ctx))goto cleanup;
    fmpq_mat_det(d,q);result=fmpq_is_zero(d)?T_FALSE:T_TRUE;
cleanup:
    fmpq_clear(d);fmpq_mat_clear(q);return result;
}
static truth_t chain(const ca_mat_t A,const ca_mat_t P,const ca_mat_t J,ca_ctx_t ctx)
{
    ca_t left,right,t;ca_init(left,ctx);ca_init(right,ctx);ca_init(t,ctx);truth_t result=T_TRUE;
    for(int i=0;i<A->r;i++)for(int j=0;j<A->c;j++) {
        ca_zero(left,ctx);ca_zero(right,ctx);
        for(int k=0;k<A->r;k++) {
            ca_mul(t,ca_mat_entry(A,i,k),ca_mat_entry(P,k,j),ctx);ca_add(left,left,t,ctx);
            ca_mul(t,ca_mat_entry(P,i,k),ca_mat_entry(J,k,j),ctx);ca_add(right,right,t,ctx);
        }
        truth_t eq=ca_check_equal(left,right,ctx);
        if(eq==T_FALSE){result=T_FALSE;goto cleanup;}if(eq==T_UNKNOWN)result=T_UNKNOWN;
    }
cleanup:
    ca_clear(left,ctx);ca_clear(right,ctx);ca_clear(t,ctx);return result;
}
int main(void)
{
    unsigned rows=0,incorrect=0,unresolved=0;ca_ctx_t ctx;ca_ctx_init(ctx);
    puts("kind,pattern,shape,method,success,blocks,chain,invertible");
    for(int kind=0;kind<3;kind++)for(int pattern=0;pattern<SPECTRAL_PATTERNS;pattern++)
    for(int shape=0;shape<3;shape++)for(int method=0;method<6;method++) {
        int n=corpus_dimension(pattern),success=0;
        ca_mat_t A,input,J,P;ca_mat_init(A,n,n,ctx);ca_mat_init(input,n,n,ctx);ca_mat_init(J,n,n,ctx);ca_mat_init(P,n,n,ctx);
        corpus_matrix(A,kind,pattern,shape,ctx);ca_mat_set(input,A,ctx);ca_mat_ones(J,ctx);ca_mat_ones(P,ctx);
        truth_t blocks=T_UNKNOWN,equation=T_UNKNOWN,inv=T_UNKNOWN;int have_p=method!=0&&method!=4;
        if(method==0) {
            ca_vec_t lambda;ca_vec_init(lambda,0,ctx);slong count=0,indices[SPECTRAL_MAX]={0},sizes[SPECTRAL_MAX]={0};
            success=ca_mat_jordan_blocks(lambda,&count,indices,sizes,input,ctx);
            if(success)blocks=match_blocks(lambda,count,indices,sizes,kind,pattern,ctx);
            ca_vec_clear(lambda,ctx);
        } else if(method==5) {
            ca_vec_t lambda;ca_vec_init(lambda,3,ctx);ca_t b;ca_init(b,ctx);corpus_base(b,kind,ctx);
            slong indices[3]={0},sizes[3]={0};
            for(int i=0;i<3;i++)ca_add_ui(ca_vec_entry(lambda,i),b,i,ctx);
            /* Supply exactly the distinct eigenvalues actually used. */
            int count=corpus_count[pattern],num_lambda=0;
            for(int i=0;i<count;i++){indices[i]=corpus_labels[pattern][i];sizes[i]=corpus_sizes[pattern][i];if(indices[i]+1>num_lambda)num_lambda=indices[i]+1;}
            ca_vec_set_length(lambda,num_lambda,ctx);
            success=ca_mat_jordan_transformation(P,lambda,count,indices,sizes,input,ctx);
            corpus_matrix(J,kind,pattern,0,ctx);
            if(success)blocks=match_form(J,kind,pattern,ctx);
            ca_clear(b,ctx);ca_vec_clear(lambda,ctx);
        } else {
            ca_mat_struct *j=method==2?input:J,*p=method==3?input:P;
            success=ca_mat_jordan_form(j,method==4?NULL:p,input,ctx);
            if(success) {
                blocks=match_form(j,kind,pattern,ctx);
                if(method==2)ca_mat_set(J,input,ctx);
                if(method==3)ca_mat_set(P,input,ctx);
            }
        }
        if(success&&have_p){equation=chain(A,P,J,ctx);inv=invertible(P,ctx);}
        printf("%d,%d,%d,%d,%d,%s,%s,%s\n",kind,pattern,shape,method,success,
            success?corpus_truth(blocks):"na",success&&have_p?corpus_truth(equation):"na",success&&have_p?corpus_truth(inv):"na");
        rows++;
        if(!success||blocks==T_UNKNOWN||(have_p&&(equation==T_UNKNOWN||inv==T_UNKNOWN)))unresolved++;
        else if(blocks==T_FALSE||(have_p&&(equation==T_FALSE||inv==T_FALSE)))incorrect++;
        ca_mat_clear(A,ctx);ca_mat_clear(input,ctx);ca_mat_clear(J,ctx);ca_mat_clear(P,ctx);
    }
    ca_ctx_clear(ctx);flint_cleanup();
    printf("{\"suite\":\"spectral\",\"rows\":%u,\"incorrect\":%u,\"unresolved\":%u}\n",rows,incorrect,unresolved);
    return incorrect||unresolved?1:0;
}
