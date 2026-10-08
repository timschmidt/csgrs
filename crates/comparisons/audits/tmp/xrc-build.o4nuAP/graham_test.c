/* K M Briggs 2003 Mar 26 */

#include "xr.h"

int main(int argc, char* argv[]) {
  xr_t x,y;
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("Ron Graham's sqrt test...\n");
  x=
  xr_add(xr_sqrt(xr_init(1000001,1)),
  xr_add(xr_sqrt(xr_init(1000025,1)),
  xr_add(xr_sqrt(xr_init(1000031,1)),
  xr_add(xr_sqrt(xr_init(1000084,1)),
  xr_add(xr_sqrt(xr_init(1000087,1)),
  xr_add(xr_sqrt(xr_init(1000134,1)),
  xr_add(xr_sqrt(xr_init(1000158,1)),
  xr_add(xr_sqrt(xr_init(1000182,1)),
         xr_sqrt(xr_init(1000198,1))))))))));
  y=
  xr_add(xr_sqrt(xr_init(1000002,1)),
  xr_add(xr_sqrt(xr_init(1000018,1)),
  xr_add(xr_sqrt(xr_init(1000042,1)),
  xr_add(xr_sqrt(xr_init(1000066,1)),
  xr_add(xr_sqrt(xr_init(1000113,1)),
  xr_add(xr_sqrt(xr_init(1000116,1)),
  xr_add(xr_sqrt(xr_init(1000169,1)),
  xr_add(xr_sqrt(xr_init(1000175,1)),
         xr_sqrt(xr_init(1000199,1))))))))));
  printf("should be -1: %d\n",xr_cmp(x,y));
  xr_dotdump("graham_test",x,1);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
