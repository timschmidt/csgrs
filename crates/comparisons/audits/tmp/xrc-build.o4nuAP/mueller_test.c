/* K M Briggs 2003 Mar 26 */

#include "xr.h"

void mueller_test(int nit) {
  int i;
  xr_t x;
  printf("Norbert Mueller logistic test: x<-3.75*x*(1-x)...\n");
  xr_timing(0);
  x=xr_init(1,2);
  for (i=1; i<=nit; i++) {
    x=xr_divi(xr_imul(375,xr_mul(x,xr_isub(1,x))),100);
    if (i%100==0) { 
      printf("i=%4d x=%g\n",i,xr_get_d(x,6));
      xr_timing(1);
    }
  }
  xr_free(x);
  return;
}

int main(int argc, char* argv[]) {
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  mueller_test(1000);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
