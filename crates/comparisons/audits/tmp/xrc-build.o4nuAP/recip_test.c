/* K M Briggs 2003 Mar 26 */

#include "xr.h"

int main(int argc, char* argv[]) {
  xr_t x,y;
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("recip test...\n");
  x=xr_init(7,1);
  y=xr_recip(x);
  xr_print(y,100);
  printf("\nfinished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
