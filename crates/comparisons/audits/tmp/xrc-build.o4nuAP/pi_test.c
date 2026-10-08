/* K M Briggs 2003 Mar 28 */

#include "xr.h"

int main(int argc, char* argv[]) {
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("pi test...\n");
  xr_timing(0);
  xr_print(xr_pi(),100);
  printf("\n");
  xr_timing(1);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
