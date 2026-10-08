/* K M Briggs 2005 Feb 02 
 make prime_test
 mcopy -o -v prime_test.c xr.c a:
*/

#include "xr.h"

/*
unsigned int min_k(const unsigned int p, const mpfr_t x, const mpfr_t y) {
  // return smallest k s.t. p^k*x>y
  int k=-1;
  mpfr_t t;
  mpfr_init(t);
  mpfr_set(t,x,GMP_RNDN);
  while (k++,mpfr_cmp(t,y)<=0) mpfr_mul_ui(t,t,p,GMP_RNDN);
  mpfr_clear(t);
  return k;
}
*/

int prime_test() {
  int k=-1,pi=15485863,epsn=1,epsd=2; /* 1024^epsd really */
  xr_t p,t,x,y;
  xr_set_b(64);
  xr_timing(0);
  p=xr_init(pi,1);
  printf("p=%s\n",xr_get_str(p,12));
  /* x = p^(epsn/2^epsd)-1 */
  x=xr_init(pi,1);
  printf("x=%s\n",xr_get_str(x,8));
  for (k=0; k<epsd; k++) x=xr_root(x,1024);
  printf("x=%s\n",xr_get_str(x,8));
  x=xr_powi(x,epsn); 
  printf("x=%s\n",xr_get_str(x,8));
  x=xr_subi(x,1);
  printf("x=%s\n",xr_get_str(x,8));
  /* y = p^(1+epsn/2^epsd)-1 */
  y=xr_init(pi,1);
  for (k=0; k<epsd; k++) y=xr_root(y,1024);
  y=xr_powi(y,epsn);
  y=xr_imul(pi,y);
  y=xr_subi(y,1);
  printf("y=%s\n",xr_get_str(y,12));
  t=xr_set(x);
  printf("A\n");
  k=-1;
  while (k++,xr_cmp(t,y)<=0) {
    t=xr_imul(pi,t);
    printf("k=%4d t=%s\n",k,xr_get_str(t,12));
  }
  printf("k=%4d\n",k);
  xr_timing(1);
  xr_free(x);
  xr_free(y);
  xr_free(t);
  xr_free(p);
  return k;
}

int main(int argc, char* argv[]) {
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  prime_test(60);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
