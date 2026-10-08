/* K M Briggs 2003 Mar 31 */

#include "xr++.h"

void logistic_test(int nit) {
  int i;
  xr x;
  printf("logistic++ test: x<-4*x*(1-x)...\n");
  xr_timing(0);
  x=xr(671875,1000000);
  for (i=1; i<=nit; i++) {
    x=4*x*(1-x);
    cout<<i<<" "<<x<<endl;
  }
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
