/* K M Briggs 2003 Mar 26 */

#include "xr.h"

void logistic_test(int nit) {
  int i;
  xr_t x;
  printf("logistic test: x<-4*x*(1-x)...\n");
  xr_timing(0);
  x=xr_init(671875,1000000);
  for (i=1; i<=nit; i++) {
    printf("i=%4d x=%s\n",i,xr_get_str(x,12));
    x=xr_imul(4,xr_mul(x,xr_isub(1,x)));
  }
  printf("i=%4d x=%s\n",nit,xr_get_str(x,12));
  xr_free(x);
  xr_timing(1);
  return;
}

int main(int argc, char* argv[]) {
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  logistic_test(60);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
