/* K M Briggs 2003 Apr 01 */
/* sign fixed in xr_subi 2005 Feb 02 */
/* mpz_clear fixed line 460 2005 Apr 27 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "xr.h"
#include "pi_c_kmb.c"

const char* xr_version="xrc-1.2 compiled" " "__DATE__" "__TIME__;

int _b=1;                                      /* granularity */
const int caching=1;                           /* 0=>no caching */
const double _q=3.321928094887362347870319429; /* log(10)/log(2) */
FILE* dotf;                                    /* used in dotdump */
int showcache=1;                               /* used in dotdump */

void _pih(mpz_t r, const int n);
/* why is this not found in stdio.h?... */
/*int snprintf(char*,size_t,const char*,const int64_t);*/
int snprintf(char *str, size_t size, const char *format, ...);
/* op codes for main evalution loop */
enum _op {_rat,_abs,_neg,_sqrt,_recip,_iadd,_isub,_imul,_subi,_divi,_add,_sub,_mul,_sqr,_div,_root,_exp1,_pi};

/* op codes names for dump functions */
char* _opnames[]={"","||","-","sqrt","1/","+","-","*","-","/","+","-","*","sqr","/","root","exp1","pi"};

struct _node {  /* internal representation of an exact real */
  void* x;      /* left  operand */
  void* y;      /* right operand */
  enum _op f;   /* operation */
  mpz_t cache;  /* cache */
  int maxn;     /* cache high water mark */
};

void _eval(mpz_t,struct _node*,const int);

#ifdef TIMING
  struct timeval tv0[1],tv1[1];
#endif

/* internal utility functions */

void _abort(const char* s, int line) {
  fprintf(stderr,"%s aborting - %s at line %d of xr.c\n",xr_version,s,line);
  exit(1);
}

void xr_timing(int k) {
  #ifdef TIMING
  double t0,t1,dt;
  if (k==0) { /* first call */
    gettimeofday(tv0,NULL);
    fprintf(stderr,"xr_timing: t = 0\n");
    return;
  }
  gettimeofday(tv1,NULL);
  t0=tv0->tv_usec/1000.0+1000*tv0->tv_sec;
  t1=tv1->tv_usec/1000.0+1000*tv1->tv_sec;
  dt=t1-t0;
  fprintf(stderr,"xr_timing: delta t = %.3f millisecond%s\n",dt,(dt!=1?"s":""));
  *tv0=*tv1;
  #endif
  return;
}

int64_t _max(const int64_t x, const int64_t y) {
  return x>y?x:y;
}

int64_t _gcd(int64_t x, int64_t y) {
  int64_t t;
  while (y) { t=y; y=x%y; x=t; }
  return x;
}

int64_t _log2(const mpz_t x) { /* floor(log2(x))[x>0], else 0 */
  if (mpz_cmp_si(x,0)<=0) return 0;
  return mpz_popcount(x)-1;
}

int64_t _log2int64_t(const int64_t x) { /* floor(log2(x))[x>0], else 0 */
  int64_t t=x,k=0;
  if (x<=0) return 0;
  while (1) {
    t>>=1;
    if (t==0) return k;
    k++;
  }
}

/* internal exact real functions... */

void _eval(mpz_t,struct _node*,const int);

size_t mpz_print(const mpz_t x) {
  return mpz_out_str(stdout,10,x);
}

void mpz_fprint(FILE* f, const mpz_t x) {
  mpz_out_str(f,10,x);
}

void _deepdump(const struct _node* a, const char* msg) {
  printf("%s",msg);
  if (caching && a->maxn>=0) {
    printf("[");
    if (showcache) { printf("cache="); mpz_print((a->cache)); printf(","); }
    printf("maxn=%d] ",a->maxn);
  }
  switch (a->f) {
    case _rat:
      printf("q(");
      mpz_print(*(mpz_t*)(a->x));
      if (a->y) { printf("/"); mpz_print(*(mpz_t*)(a->y)); }
      printf(")");
      break;
    case _imul:
      printf("(%ld",*(int64_t*)a->x);
      _deepdump(a->y," i*");
      printf(")");
      break;
     case _divi:
      printf("(");
      _deepdump(a->x," /i");
      printf("%ld)",*(int64_t*)a->y);
      break;
    default:
      printf("(");
      _deepdump(a->x," ");
      if (a->f==_add) printf(" +");
      if (a->f==_sub) printf(" -");
      if (a->f==_mul) printf("*");
      _deepdump(a->y," ");
      printf(")");
  }
  return;
}

void _dotdump(const struct _node* a, int64_t a0, const char* msg) {
  int64_t aa,ax,ay;
  aa=a0-(int64_t)a;
  if (a) ax=a0-(int64_t)a->x;
  if (a) ay=a0-(int64_t)a->y;
  if (a) switch (a->f) { /* FIXME missing ops */
    case _rat:
      fprintf(dotf,"%Ld [shape=ellipse,color=green,label=\"",aa);
      mpz_fprint(dotf,*(mpz_t*)(a->x));
      if (a->y) { fprintf(dotf,"/"); mpz_fprint(dotf,*(mpz_t*)(a->y)); }
      fprintf(dotf,"\"]\n");
      break;
    case _imul:
      fprintf(dotf,"%Ld [shape=ellipse,label=\"%Ld*\\n[%d]",
        aa,*(int64_t*)a->x,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ay);
      _dotdump(a->y,a0,"");
      break;
    case _iadd:
      fprintf(dotf,"%Ld [shape=ellipse,label=\"%Ld-\\n[%d]",
        aa,*(int64_t*)a->x,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ay);
      _dotdump(a->y,a0,"");
      break;
    case _sqr:
      fprintf(dotf,"%Ld [label=\"square\\n[%Ld]",aa,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _sqrt:
      fprintf(dotf,"%Ld [label=\"sqrt\\n[%Ld]",aa,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _recip:
      fprintf(dotf,"%d [label=\"1/\\n[%Ld]",aa,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _exp1:
      fprintf(dotf,"%Ld [label=\"exp1\\n[%Ld]",aa,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%Ld -> %Ld\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _isub:
      fprintf(dotf,"%d [shape=diamond,label=\"%ld-\\n[%d]",
        aa,*(int64_t*)a->x,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%d -> %d [style=dotted]\n",aa,ay);
      _dotdump(a->y,a0,"");
      break;
    case _divi:
      fprintf(dotf,"%d [shape=diamond,label=\"/%ld\\n[%d]",
        aa,*(int64_t*)a->y,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%d -> %d\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _root:
      fprintf(dotf,"%d [shape=diamond,label=\"root%ld\\n[%d]",
        aa,*(int64_t*)a->y,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%d -> %d [style=dotted]\n",aa,ax);
      _dotdump(a->x,a0,"");
      break;
    case _sub:
      fprintf(dotf,"%d [shape=diamond,label=\"-\\n[%d]",aa,a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%d -> %d\n",aa,ax);
      fprintf(dotf,"%d -> %d [style=dotted]\n",aa,ay);
      _dotdump(a->x,a0," ");
      _dotdump(a->y,a0," ");
      break;
    default: /* other binary ops */
      fprintf(dotf,"%d [label=\"%s\\n[%d]",aa,_opnames[(int)a->f],a->maxn);
      if (showcache && a->maxn>-1) {
        fprintf(dotf,"\\n"); mpz_fprint(dotf,a->cache); }
      fprintf(dotf,"\"]\n");
      fprintf(dotf,"%d -> %d\n",a0-(int64_t)a,ax);
      fprintf(dotf,"%d -> %d\n",a0-(int64_t)a,ay);
      _dotdump(a->x,a0," ");
      _dotdump(a->y,a0," ");
  }
  return;
}

void xr_dotdump(const char* dotfn, const struct _node* a, const int sc) {
  int a0;
  char* dfn;
  showcache=sc;
  a0=(int64_t)a;
  dfn=malloc(strlen(dotfn)+5);
  strcpy(dfn,dotfn);
  dfn=strcat(dfn,".dot");
  dotf=fopen(dfn,"w");
  fprintf(stderr,"dotdump: writing to %s...",dfn);
  fprintf(dotf,"digraph %s {\n",dotfn);
  fprintf(dotf,"label=\"%s\"\n",dotfn);
  fprintf(dotf,"comment=\"automatically generated by %s\"\n",xr_version);
  fprintf(dotf,"size=\"8,10\"\n");
  fprintf(dotf,"ratio=1\n");
  fprintf(dotf,"node [width=1.2,height=1.2,color=yellow,shape=box,style=filled,fontname=Courier,fontsize=24]\n");
  fprintf(dotf,"edge [color=blue1,linewidth=2,arrowsize=2,fontsize=24]\n");
  fprintf(dotf,"graph [fontcolor=green, fontsize=24]\n");
  fprintf(dotf,"center=true\n");
  fprintf(dotf,"mclimit=10\n");
  fprintf(dotf,"nslimit=10\n");
  fprintf(dotf,"concentrate=true\n");
  _dotdump(a,a0,"");
  fprintf(dotf,"}\n");
  fclose(dotf);
  fprintf(stderr," written - now run \"dot -Tps2 %s | gv -\"\n",dfn);
  free(dfn);
  return;
}

int _msd(struct _node* z, int mx) {
  int c,i=0;
  mpz_t a;
  mpz_init(a);
  do {
    _eval(a,z,i);
    c=mpz_cmpabs_ui(a,1);
    if ((i<mx && c>0) || (mx<=0 && c>0) || (mx>0 && i>=mx)) {
      /* if (i>mx) i=mx;  in case stride>1 has taken us too far */
      mpz_clear(a);
      /* z->msd=i; */
      return i;
    }
    i+=1; /* FIXME what about a bigger stride here? */
  } while (i);
  _abort("_msd non-termination",__LINE__);
  return 0; /* never get here */
}

void mpz_ndiv_q_2exp(mpz_t rop, const mpz_t op, const uint64_t p) {
 /* rop <- op/2^p, rounded to nearest (rounding away when middle) */
  if (p!=0) {
    int rnd_bit=mpz_tstbit(op,p-1);
    int sgn=mpz_sgn(op);
    mpz_tdiv_q_2exp(rop,op,p);
    if (rnd_bit) {
      if (sgn>0) mpz_add_ui(rop,rop,1);
      else       mpz_sub_ui(rop,rop,1);
    }
  }
  else mpz_set(rop,op);
  return;
}

/* rop <- op/2^p, rounded to nearest (rounding away when middle) */
/* inline version */
#define NDIV(rop,op,p)\
if (p!=0) {\
  int rnd_bit=mpz_tstbit(op,p-1);\
  int sgn=mpz_sgn(op);\
  mpz_tdiv_q_2exp(rop,op,p);\
  if (rnd_bit) {\
    if (sgn>0) mpz_add_ui(rop,rop,1);\
    else       mpz_sub_ui(rop,rop,1);\
   }\
}\
else mpz_set(rop,op);

void mpz_ndiv_q_ui(mpz_t rop, const mpz_t op, const uint64_t p) {
  /* rop <- op/p, rounded to nearest */
  /* op>0 only! Not checked! */
  if (p==1) { mpz_set(rop,op); return; }
  mpz_add_ui(rop,op,p/2);
  mpz_tdiv_q_ui(rop,rop,p);
  return;
}

void _exphelper(mpz_t y, xr_t x, int64_t n) {
  int i;
  uint64_t m,e,p;
  mpz_t q,u,t,s;
  m=e=0;
  mpz_init(q); /* FIXME can we use fewer mpz_t temporaries ? */
  mpz_init(u);
  mpz_init(s);
  mpz_init(t);
  mpz_set_si(s,1);
  mpz_mul_2exp(s,s,_b*(n+2));
  mpz_mul_si(s,s,3);
  while (1) {
    mpz_fac_ui(t,m);
    if (mpz_cmp(t,s)>0) break;
    m++;
  }
  while (1) {
    mpz_set_si(s,1);
    mpz_mul_2exp(s,s,_b*e);
    if (mpz_cmp_si(s,2*m)>0) break;
    e++;
  }
  p=n+e+2;
  _eval(s,x,p);
  mpz_set_si(u,1);
  mpz_mul_2exp(u,u,_b*p);
  mpz_set(t,u);
  for (i=1; i<=m; i++) {
    mpz_mul(q,s,t);
    mpz_ndiv_q_2exp(t,q,_b*p); /* FIXME is this ok for t=ndiv(q,v*i) ?? */
    mpz_tdiv_q_ui(t,t,i);
    mpz_add(u,u,t);
    if (mpz_cmp_si(t,0)==0) break;
  }
  mpz_ndiv_q_2exp(y,u,_b*(e+2));
  mpz_clear(q);
  mpz_clear(u);
  mpz_clear(s);
  mpz_clear(t);
  return;
}

int64_t xr_log2_bound(xr_t x) {
  /* return k such that |x/2^k|<1 */
  int64_t k=0,i=0;
  xr_t y;
  mpz_t w,t,q;
  mpz_init(w);
  mpz_init(t);
  mpz_init_set_si(q,1);
  y=xr_set(x);
  while (1) {
    _eval(t,y,i);
    mpz_sub_ui(w,q,1);
    if (mpz_cmpabs(t,w)<0) { /* => |x/2^k|<1 */
      mpz_clear(w);
      mpz_clear(t);
      mpz_clear(q);
      xr_free(y);
      return k;
    }
    mpz_tdiv_q_2exp(w,q,1);
    mpz_add_ui(w,w,1);
    if (mpz_cmpabs(t,w)>0) { /* => |x/2^k|>1/2 */
      k++;
      if (k==LONG_MAX) _abort("out of range in xr_log2_bound",__LINE__);
      y=xr_divi(y,2);
    }
    mpz_mul_2exp(q,q,_b);
    i++;
  }
  return 0; /* never get here */
}

void _eval(mpz_t w, struct _node* z, const int n) {
  int64_t p,q,m,r,s;
  mpz_t a;
  if (0) { /* FIXME */
    fprintf(stderr,"n=%d ",n);
    _abort("n<0 in _eval",__LINE__);
  }
  s=z->maxn-n;
  if (caching && s>=0) {
    if (s==0) { mpz_set(w,z->cache); return; }
    mpz_tdiv_q_2exp(w,z->cache,_b*s);
    return;
  }
  switch (z->f) {
    case _rat: /* z is rational */
      mpz_mul_2exp(w,*(mpz_t*)(z->x),_b*n);
      if (z->y) mpz_tdiv_q(w,w,*(mpz_t*)(z->y)); /* z is not integer */
      return;
    case _abs:
      _eval(w,z->x,n);
      mpz_abs(w,w);
      break;
    case _neg:
      _eval(w,z->x,n);
      mpz_neg(w,w);
      break;
    case _sqrt:
      _eval(w,z->x,2*n);
      if (mpz_cmp_si(w,0)<0) _abort("negative argument to sqrt in _eval",__LINE__);
      mpz_sqrt(w,w);
      break;
    case _recip:
      m=_msd(z->x,0);
      if (m<=-n) 
        mpz_set_si(w,0);
      else {
        p=n+2*m+1;
        if (_b==1) p++;
        _eval(w,z->x,p);
        mpz_init_set_ui(a,1);
        mpz_mul_2exp(a,a,_b*(n+p));
        if (mpz_cmp_si(w,1)>0) mpz_cdiv_q(w,a,w);
        else mpz_fdiv_q(w,a,w);
        mpz_clear(a); /* KMB 2005 Apr 27 - was outside else */
      }
      break;
    case _iadd:
      p=(_b==1?2:1);
      mpz_set_si(w,*(int64_t*)z->x);
      mpz_mul_2exp(w,w,_b*(n+p));
      mpz_init(a);
      _eval(a,z->y,n+p);
      mpz_add(w,w,a);
      NDIV(w,w,_b*p)
      mpz_clear(a);
      break;
    /*case _isub:
      p=(_b==1?2:1);
      mpz_set_si(w,*(int64_t*)z->x);
      mpz_mul_2exp(w,w,_b*(n+p));
      mpz_init(a);
      _eval(a,z->y,n+p);
      mpz_sub(w,w,a);
      NDIV(w,w,_b*p)
      mpz_clear(a);
      break;*/
    case _isub:
      p=(_b==1?2:1);
      mpz_set_si(w,*(int64_t*)z->x);
      mpz_mul_2exp(w,w,_b*(n+p));
      mpz_init(a);
      _eval(a,z->y,n+p);
      NDIV(a,a,_b*p) /* try shifting first */
      NDIV(w,w,_b*p)
      mpz_sub(w,w,a);
      mpz_clear(a);
      break;
    case _imul:
      m=*(int64_t*)z->x;
      switch (m) {
        case 0:
          mpz_set_ui(w,0);
          break;
        /*case 1:
          mpz_set(w,z->y);
          break;
        case -1:
          mpz_neg(w,z->y);
          break; */
        default:
          p=(1+_log2int64_t(m))/_b+2;
          _eval(w,z->y,n+p);
          mpz_mul_si(w,w,m);
          NDIV(w,w,_b*p)
      }
      break;
    case _subi:
      p=(_b==1?2:1);
      mpz_set_si(w,*(int64_t*)z->y);
      mpz_mul_2exp(w,w,_b*(n+p));
      mpz_init(a);
      _eval(a,z->x,n+p);
      mpz_sub(w,a,w); /* sign fixed 2005 Feb 02 */
      NDIV(w,w,_b*p)
      mpz_clear(a);
      break;
    case _divi:
      m=*(int64_t*)z->y;
      if (m>0) {
        _eval(w,z->x,n);
        mpz_add_ui(w,w,m/2);
        mpz_tdiv_q_ui(w,w,m);
      } else {
        _abort("divi for m<=0 not implemented",__LINE__);
      }
      break;
    case _add:
      p=(_b==1?2:1);
      mpz_init(a);
      _eval(a,z->x,n+p);
      _eval(w,z->y,n+p);
      mpz_add(a,a,w);
      /* NDIV(w,w,_b*p) */
      mpz_ndiv_q_2exp(w,a,_b*p);
      mpz_clear(a);
      break;
    case _sub:
      p=(_b==1?2:1);
      mpz_init(a);
      _eval(a,z->x,n+p);
      _eval(w,z->y,n+p);
      mpz_sub(w,a,w);
      NDIV(w,w,_b*p)
      mpz_clear(a);
      break;
    case _mul:
      if (_b==1) { /* FIXME or use `else' in b=1 case too? */
        r=(n+2)/2;
        s=n+2-r;
        mpz_init(a);
        _eval(a,z->x,r);
        mpz_abs(a,a);
        p=_log2(a);
        _eval(w,z->y,s);
        mpz_abs(w,w);
        q=_log2(w);
        if (p==0 && q==0) {
          mpz_set_si(w,0);
        } else {
          _eval(a,z->x,q+r+1);
          _eval(w,z->y,p+s+1);
          mpz_mul(w,a,w);
          NDIV(w,w,p+q+4)
        }
        mpz_clear(a);
      } else {
	if (_b==1) { p=4; q=3; } else { p=3; q=2; }
        m=(n+q)/2;
        r=_max(n+p-_msd(z->y,n+p-m),m);
        s=_max(n+p-_msd(z->x,n+p-m),m);
        mpz_init(a);
        _eval(a,z->x,r);
        _eval(w,z->y,s);
        mpz_mul(w,a,w);
        mpz_add_ui(w,w,1);
        NDIV(w,w,(_b*(r+s-n)))
        mpz_clear(a);
      }
      break;
    case _sqr:
      if (_b==1) { p=4; q=3; } else { p=3; q=2; }
      m=(n+q)/2;
      s=_max(n+p-_msd(z->x,n+p-m),m);
      _eval(w,z->x,s);
      mpz_mul(w,w,w);
      mpz_add_ui(w,w,1);
      NDIV(w,w,(_b*(2*s-n)))
      break;
    case _root:
      m=*(int64_t*)z->y;
      _eval(w,z->x,m*n);
      mpz_root(w,w,m);
      break;
    case _exp1: /* exp for |x|<1, not checked */
      _exphelper(w,z->x,n);
      break;
    case _pi:
      _pih(w,n);
      break;
    default:
      _abort("_eval: function not implemented\n",__LINE__);
  }
  if (caching) { z->maxn=n; mpz_set(z->cache,w); }
  return;
}

/* xr user functions... */

int xr_get_b(void) {
  return _b;
}

void xr_set_b(const int b) {
  _b=b;
  return;
}

xr_t xr_init(int64_t a, int64_t b) {
  int64_t g;
  struct _node* z=malloc(sizeof(struct _node));
  mpz_t* x=malloc(sizeof(mpz_t));
  mpz_t* y=malloc(sizeof(mpz_t));
  if (b==0)  _abort("denominator=0 in xr_init",__LINE__);
  if (b<0) { a=-a; b=-b; }
  g=_gcd(a,b);
  a/=g; b/=g;
  mpz_init_set_si(*x,a);
  mpz_init_set_si(*y,b);
  z->x=x;
  z->y=y;
  if (b==1) z->y=NULL; /* indicates denominator=1 */
  z->f=_rat;
  mpz_init(z->cache);
  z->maxn=-1;
  return z;
}

xr_t xr_set(const xr_t a) {
  struct _node* z=malloc(sizeof(struct _node));
  z->x=a->x;
  z->y=a->y;
  z->f=a->f;
  mpz_init(z->cache);
  z->maxn=-1;
  return z;
}

void xr_free(const xr_t x) {
  /* FIXME recursive clear ?
  if (!x->x) free(x->x);
  if (!x->y) free(x->y);
  */
  if (!x) {
    if (!x->cache) mpz_clear(x->cache);
    free(x);
  }
  return;
}

/* macro to generate an xr user function with no arguments */
#define ONARY_OP(X)\
xr_t xr_##X(void) {\
  struct _node* z=malloc(sizeof(struct _node));\
  z->x=NULL;\
  z->y=NULL;\
  z->f=_##X;\
  mpz_init(z->cache);\
  z->maxn=-1;\
  return z;\
}

/* macro to generate an xr user function with one xr_t argument */
#define UNARY_OP(X)\
xr_t xr_##X(const xr_t x) {\
  struct _node* z=malloc(sizeof(struct _node));\
  z->x=x;\
  z->y=NULL;\
  z->f=_##X;\
  mpz_init(z->cache);\
  z->maxn=-1;\
  return z;\
}

/* macro to generate an xr user function with one int64_t and one xr_t argument */
#define BUNARY_OP(X)\
xr_t xr_##X(const int64_t x, const xr_t y) {\
  int64_t* t=malloc(sizeof(int64_t));\
  struct _node* z=malloc(sizeof(struct _node));\
  *t=x;\
  z->x=t;\
  z->y=y;\
  z->f=_##X;\
  mpz_init(z->cache);\
  z->maxn=-1;\
  return z;\
}

/* macro to generate an xr user function with one xr_t and one int64_t argument */
#define UBNARY_OP(X)\
xr_t xr_##X(const xr_t x, const int64_t y) {\
  int64_t* t=malloc(sizeof(int64_t));\
  struct _node* z=malloc(sizeof(struct _node));\
  *t=y;\
  z->x=x;\
  z->y=t;\
  z->f=_##X;\
  mpz_init(z->cache);\
  z->maxn=-1;\
  return z;\
}

/* macro to generate an xr user function with two xr_t arguments */
#define BINARY_OP(X)\
xr_t xr_##X(const xr_t x, const xr_t y) {\
  struct _node* z=malloc(sizeof(struct _node));\
  z->x=x;\
  z->y=y;\
  z->f=_##X;\
  mpz_init(z->cache);\
  z->maxn=-1;\
  return z;\
}

ONARY_OP(pi)

UNARY_OP(abs)
UNARY_OP(neg)
UNARY_OP(sqr)
UNARY_OP(sqrt)
UNARY_OP(recip)
UNARY_OP(exp1)

BUNARY_OP(iadd)
BUNARY_OP(isub)
BUNARY_OP(imul)

UBNARY_OP(subi)
UBNARY_OP(divi)
UBNARY_OP(root)

BINARY_OP(add)
BINARY_OP(sub)
BINARY_OP(mul)

/* evaluation-initiating functions */

void xr_eval(mpz_t z, const xr_t x, const int n) {
  _eval(z,x,n);
  return;
}

int xr_cmp(xr_t a, xr_t b) {
  int i=0;
  mpz_t ai,bi;
  mpz_init(ai);
  mpz_init(bi);
  do {
    if (0) fprintf(stderr,"%d eval: i=%d\n",__LINE__,i);
    _eval(ai,a,i);
    if (0) fprintf(stderr,"%d eval: i=%d\n",__LINE__,i);
    _eval(bi,b,i);
    mpz_sub_ui(bi,bi,1);
    if (mpz_cmp(ai,bi)<0) {
      mpz_clear(ai);
      mpz_clear(bi);
      return -1;
    }
    mpz_add_ui(bi,bi,2);
    if (mpz_cmp(ai,bi)>0) {
      mpz_clear(ai);
      mpz_clear(bi);
      return  1;
    }
  } while (++i);
  _abort("xr_cmp non-termination",__LINE__);
  return 0; /* never get here */
}

/* output */

char* int64_t_to_str(const int64_t x) {
  int n,size=20;
  char *p;
  if ((p=malloc(size))==NULL) return NULL;
  while (1) {
     /* Try to print in the allocated space. */
     n=snprintf(p,size,"%lld",x);
     /* If that worked, return the string. */
     if (n>0 && n<size) return p;
     /* Else try again with more space. */
     if (n>1)     /* glibc 2.1 */
        size=n+1; /* precisely what is needed */
     else         /* glibc 2.0 */
        size*=2;  /* twice the old size */
     if ((p=realloc(p,size))==NULL) return NULL;
  }
}

double xr_get_d(const xr_t x, int n) {
  int k;
  double y;
  mpz_t a;
  if (n<1)  n=1;
  if (n>16) n=16;
  mpz_init(a);
  k=_msd(x,0)+(int)ceil(_q*n/_b); /* ensures |x(k)| has r.e. <10^{-n} */
  if (k*_b>1023) _abort("exponent out of range in xr_get_d",__LINE__);
  mpz_init(a);
  _eval(a,x,k);
  y=mpz_get_d(a);
  mpz_clear(a);
  return ldexp(y,-k*_b);
}

char* xr_get_str(const xr_t x, int n) {
  int j,k;
  char *s,*ex;
  char* sgn=" 0.";
  mpz_t a,b;
  if (n<1)  n=1;
  k=_msd(x,0)+(int)ceil(_q*n/_b); /* ensures |x(k)| has r.e. <10^{-n} */
  mpz_init(a);
  mpz_init(b);
  _eval(b,x,k);
  j=n+(int)(k*_b/_q);
  mpz_ui_pow_ui(a,10,j);
  mpz_mul(b,a,b);
  mpz_tdiv_q_2exp(a,b,_b*k);
  if (mpz_cmp_si(a,0)<0) { sgn="-0."; mpz_neg(a,a); }
  s=mpz_get_str(NULL,10,a);
  if (!(ex=int64_t_to_str(strlen(s)-j))) _abort("malloc failure in int64_t_to_str",__LINE__);
  if (!(s=realloc(s,5+n+strlen(ex)))) _abort("realloc failure in xr_get_str",__LINE__); 
  memmove(s+3,s,n);
  strncpy(s,sgn,3);
  strncpy(s+n+3,"e",1);
  strncpy(s+n+4,ex,strlen(ex)+1);
  free(ex);
  mpz_clear(a);
  mpz_clear(b);
  return s;
}

int xr_print(const xr_t x, const int n) {
  char* s=xr_get_str(x,n);
  int k;
  k=printf("%s",xr_get_str(x,n));
  free(s);
  return k;
}

int xr_print_nl(const xr_t x, const int n) {
  char* s=xr_get_str(x,n);
  int k;
  k=printf("%s\n",xr_get_str(x,n));
  free(s);
  return k;
}

/* non-primitive arithmetic */

xr_t xr_near_int(const xr_t x) { /* floor or ceiling */
  int64_t i;
  mpz_t xi,up,dn,t,y;
  xr_t r;
  i=_msd(x,0);
  mpz_init(xi);
  mpz_init(up);
  mpz_init(dn);
  mpz_init(t);
  mpz_init(y);
  do {
    _eval(xi,x,i);
    mpz_sub_ui(xi,xi,1);
    mpz_tdiv_q_2exp( y,xi,_b*i);
    mpz_add_ui(xi,xi,2);
    mpz_tdiv_q_2exp(up,xi,_b*i);
    mpz_sub(up,up,y);
    if (mpz_cmp_si(up,1)<0) {
      r=xr_init(mpz_get_ui(y),1);
      mpz_clear(xi);
      mpz_clear(up);
      mpz_clear(dn);
      mpz_clear(t);
      mpz_clear(y);
      return r;
    }
  } while (++i);
  _abort("failure in xr_near_int",__LINE__); /* never get here */
}

xr_t xr_div(const xr_t x, const xr_t y) {
  return xr_mul(x,xr_recip(y));
}

xr_t xr_powi(const xr_t xx, int64_t n) {
  int64_t m,N=n;
  xr_t x,y,z;
  y=xr_init(1,1);
  if (n==0) return y; /* gives 0^0=1 */
  if (n==1) return xx;
  if (n==2) return xr_sqr(xx);
  z=xr_init(1,1);
  if (n>0) x=xr_set(xx);
  else { n=-n; x=xr_recip(xx); }
  z=xr_set(x);
  while (1) {
    m=N; 
    N/=2;
    if (m!=2*N) { y=xr_mul(z,y); if (N==0) return y; }
    z=xr_sqr(z);
  }
  return y; /* never get here */
}

xr_t xr_exp(const xr_t x) {
  int64_t i,m;
  xr_t y;
  m=xr_log2_bound(x);
  if (m==LONG_MAX) _abort("scaling failure in exp",__LINE__);
  y=xr_exp1(xr_divi(x,1L<<m));
  for (i=0; i<m; i++) y=xr_sqr(y);
  return y;
}

/* pi */

char* _pihelper(int64_t nhex) {
  int ic;
  char *pihex;
  double pid;
  if (nhex>(1L<<24)) _abort("nhex too big in pihelper",__LINE__);
  if (nhex<1L) nhex=1;
  pihex=(char*)calloc(nhex,1);
  for (ic=0; ic<nhex; ic+=8) { /* get 8 hexits at a time */
    pid=4*series(1,ic)-2*series(4,ic)-series(5,ic)-series(6,ic);
    pid=pid-floor(pid)+1;
    ihex(pid,8,pihex+ic);
  }
  return pihex;
}

void _pih(mpz_t r, const int n) {
  int i,k;
  int64_t m;
  char* pihex;
  m=(int)(4.0*n/_b);
  m/=8; m+=1; m*=8;
  pihex=_pihelper(m);
  mpz_set_ui(r,3);
  for (i=0; i<m; i++) {
    k=pihex[i]-'0';
    if (k>9) k-=39;
    mpz_mul_2exp(r,r,4);
    mpz_add_ui(r,r,k);
  }
  mpz_tdiv_q_2exp(r,r,4*m-_b*n);
  free(pihex);
  return;
}
