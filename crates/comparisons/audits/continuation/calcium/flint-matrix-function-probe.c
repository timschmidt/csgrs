#include "spectral-corpus.h"

/* Direct finite Taylor coefficients for each known Jordan block. The shared
   scalar backend evaluates exp/log at its eigenvalue, but no matrix function,
   Jordan decomposition or matrix polynomial is used as the coefficient oracle. */
static int expected(ca_mat_t out,int kind,int pattern,int shape,int logarithm,ca_ctx_t ctx)
{
    ca_mat_t j;ca_mat_init(j,out->r,out->c,ctx);
    ca_t b,lambda,t,power;ca_init(b,ctx);ca_init(lambda,ctx);ca_init(t,ctx);ca_init(power,ctx);
    corpus_base(b,kind,ctx);int offset=0,exists=1;
    for(int block=0;block<corpus_count[pattern];block++) {
        int size=corpus_sizes[pattern][block];ca_add_ui(lambda,b,corpus_labels[pattern][block],ctx);
        if(logarithm&&ca_check_is_zero(lambda,ctx)==T_TRUE){exists=0;break;}
        if(logarithm)ca_log(t,lambda,ctx);else ca_exp(t,lambda,ctx);
        for(int order=0;order<size;order++) {
            if(order) {
                if(logarithm) {
                    ca_pow_ui(power,lambda,order,ctx);ca_inv(t,power,ctx);ca_div_ui(t,t,order,ctx);
                    if(order%2==0)ca_neg(t,t,ctx);
                } else ca_div_ui(t,t,order,ctx);
            }
            for(int row=0;row+order<size;row++)ca_set(ca_mat_entry(j,offset+row,offset+row+order),t,ctx);
        }
        offset+=size;
    }
    if(exists)corpus_transform(out,j,shape,ctx);
    ca_mat_clear(j,ctx);ca_clear(b,ctx);ca_clear(lambda,ctx);ca_clear(t,ctx);ca_clear(power,ctx);return exists;
}
int main(void)
{
    unsigned rows=0,incorrect=0,unresolved=0;ca_ctx_t ctx;ca_ctx_init(ctx);
    puts("kind,pattern,shape,alias,function,exists,status,equality");
    for(int kind=0;kind<6;kind++)for(int pattern=0;pattern<SPECTRAL_PATTERNS;pattern++)
    for(int shape=0;shape<3;shape++)for(int alias=0;alias<2;alias++)for(int logarithm=0;logarithm<2;logarithm++) {
        int n=corpus_dimension(pattern);ca_mat_t A,E,Y;
        ca_mat_init(A,n,n,ctx);ca_mat_init(E,n,n,ctx);ca_mat_init(Y,n,n,ctx);
        corpus_matrix(A,kind,pattern,shape,ctx);ca_mat_ones(Y,ctx);
        int exists=expected(E,kind,pattern,shape,logarithm,ctx);ca_mat_struct *out=alias?A:Y;
        truth_t status=logarithm?ca_mat_log(out,A,ctx):(ca_mat_exp(out,A,ctx)?T_TRUE:T_UNKNOWN);
        truth_t eq=status==T_TRUE&&exists?ca_mat_check_equal(out,E,ctx):T_UNKNOWN;
        printf("%d,%d,%d,%d,%s,%d,%s,%s\n",kind,pattern,shape,alias,logarithm?"log":"exp",exists,
            corpus_truth(status),status==T_TRUE&&exists?corpus_truth(eq):"na");
        rows++;
        if(status==T_UNKNOWN||(status==T_TRUE&&exists&&eq==T_UNKNOWN))unresolved++;
        else if(status!=(exists?T_TRUE:T_FALSE)||(exists&&eq!=T_TRUE))incorrect++;
        ca_mat_clear(A,ctx);ca_mat_clear(E,ctx);ca_mat_clear(Y,ctx);
    }
    ca_ctx_clear(ctx);flint_cleanup();
    printf("{\"suite\":\"matrix-function\",\"rows\":%u,\"incorrect\":%u,\"unresolved\":%u}\n",rows,incorrect,unresolved);
    return incorrect||unresolved?1:0;
}
