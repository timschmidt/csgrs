/* Independent finite recipes: A=bI+Q, with Q an integer matrix similar to
   known Jordan blocks. No ca_mat multiplication/inverse/Jordan constructor. */
#include "ca_mat.h"
#include "ca_poly.h"
#include "fmpq_mat.h"
#include <stdio.h>

enum { SPECTRAL_PATTERNS=9, SPECTRAL_MAX=6 };
static const int corpus_count[SPECTRAL_PATTERNS]={0,1,1,2,2,2,3,3,3};
static const int corpus_sizes[SPECTRAL_PATTERNS][3]={
    {0},{1},{4},{3,1},{1,3},{2,2},{2,1,2},{1,1,1},{3,2,1}};
static const int corpus_labels[SPECTRAL_PATTERNS][3]={
    {0},{0},{0},{0,0},{0,0},{0,0},{0,1,0},{0,1,2},{0,0,1}};
static int corpus_dimension(int pattern)
{
    int n=0;for(int i=0;i<corpus_count[pattern];i++)n+=corpus_sizes[pattern][i];return n;
}
static void corpus_base(ca_t b,int kind,ca_ctx_t ctx)
{
    if(kind==0){ca_set_ui(b,2,ctx);ca_div_ui(b,b,3,ctx);}
    if(kind==1){ca_set_ui(b,2,ctx);ca_sqrt(b,b,ctx);}
    if(kind==2){ca_set_ui(b,2,ctx);ca_log(b,b,ctx);}
    if(kind==3)ca_zero(b,ctx);
    if(kind==4)ca_one(b,ctx);
    if(kind==5)ca_set_si(b,-1,ctx);
}
static void corpus_spec(int pattern,int *label,int *link)
{
    int offset=0;
    for(int k=0;k<corpus_count[pattern];k++)for(int j=0;j<corpus_sizes[pattern][k];j++) {
        label[offset]=corpus_labels[pattern][k];link[offset]=j+1<corpus_sizes[pattern][k];offset++;
    }
}
/* Shape 1 uses S=I+uv^T, u=ones, v^T u=0. Shape 2 reverses the basis
   of shape 1. Explicit entry formula avoids tested matrix kernels. */
static void corpus_transform(ca_mat_t out,const ca_mat_t in,int shape,ca_ctx_t ctx)
{
    const int n=in->r;slong v[SPECTRAL_MAX]={0},sum=0;
    for(int i=0;i+1<n;i++){v[i]=i%2?-1:1;sum+=v[i];}if(n)v[n-1]=-sum;
    ca_t left,right,weighted,t;ca_init(left,ctx);ca_init(right,ctx);ca_init(weighted,ctx);ca_init(t,ctx);
    ca_zero(weighted,ctx);
    for(int i=0;i<n;i++)for(int j=0;j<n;j++) {
        ca_mul_si(t,ca_mat_entry(in,i,j),v[i],ctx);ca_add(weighted,weighted,t,ctx);
    }
    for(int i=0;i<n;i++)for(int j=0;j<n;j++) {
        int r=shape==2?n-1-i:i,c=shape==2?n-1-j:j;
        ca_set(ca_mat_entry(out,i,j),ca_mat_entry(in,r,c),ctx);
        if(shape) {
            ca_zero(left,ctx);ca_zero(right,ctx);
            for(int k=0;k<n;k++) {
                ca_mul_si(t,ca_mat_entry(in,k,c),v[k],ctx);ca_add(left,left,t,ctx);
                ca_add(right,right,ca_mat_entry(in,r,k),ctx);
            }
            ca_add(ca_mat_entry(out,i,j),ca_mat_entry(out,i,j),left,ctx);
            ca_mul_si(t,right,v[c],ctx);ca_sub(ca_mat_entry(out,i,j),ca_mat_entry(out,i,j),t,ctx);
            ca_mul_si(t,weighted,v[c],ctx);ca_sub(ca_mat_entry(out,i,j),ca_mat_entry(out,i,j),t,ctx);
        }
    }
    ca_clear(left,ctx);ca_clear(right,ctx);ca_clear(weighted,ctx);ca_clear(t,ctx);
}
static void corpus_matrix(ca_mat_t out,int kind,int pattern,int shape,ca_ctx_t ctx)
{
    int labels[SPECTRAL_MAX]={0},links[SPECTRAL_MAX]={0};corpus_spec(pattern,labels,links);
    ca_t b;ca_init(b,ctx);corpus_base(b,kind,ctx);
    ca_mat_t j;ca_mat_init(j,out->r,out->c,ctx);
    for(int i=0;i<out->r;i++) {
        ca_add_ui(ca_mat_entry(j,i,i),b,labels[i],ctx);
        if(links[i])ca_one(ca_mat_entry(j,i,i+1),ctx);
    }
    corpus_transform(out,j,shape,ctx);ca_mat_clear(j,ctx);ca_clear(b,ctx);
}
static const char *corpus_truth(truth_t t)
{return t==T_TRUE?"True":t==T_FALSE?"False":"Unknown";}
