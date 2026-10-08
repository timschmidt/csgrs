/* K M Briggs 2003 Mar 26 */

#include "xr.h"

int main(int argc, char* argv[]) {
  long i;
  xr_t x;
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("Harmonic sum test...\n");
  xr_timing(0);
  x=xr_init(0,1);
  for (i=1; i<=10000; i++) {
    x=xr_add(x,xr_init(1,i));
    if (i%1000==0) { 
      printf("i=%ld sum=",i); 
      xr_print(x,6); 
      printf("\n"); 
      xr_timing(1);
    }
  }
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
