/* K M Briggs 2003 Mar 27 */

const int digits=50;

#include "xr.h"

size_t mpz_print(mpz_t);

int main(int argc, char* argv[]) {
  long i;
  xr_t x,z;
  mpz_t y;
  mpz_init(y);
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("exp test...\n");
  xr_timing(0);
  for (i=1; i<=10; i++) {
    x=xr_exp1(xr_init(1,i));
    printf("exp(1/%ld)=",i); xr_print(x,digits); printf("\n"); 
  }
  xr_timing(1);
  for (i=1; i<=10; i++) {
    x=xr_exp1(xr_init(-1,i));
    printf("exp(-1/%ld)=",i); xr_print(x,digits); printf("\n"); 
  }
  xr_timing(1);
  for (i=1; i<=10; i++) {
    x=xr_exp(xr_init(i,1));
    printf("exp(%ld)=",i); xr_print(x,digits);  printf("\n");  
  }
  xr_timing(1);
  for (i=1; i<=10; i++) {
    x=xr_exp(xr_init(-i,1));
    printf("exp(-%ld)=",i); xr_print(x,digits);  printf("\n"); 
  }
  printf("x=exp(pi*sqrt(163)) test...\n");
  xr_timing(1);
  x=xr_exp(xr_mul(xr_pi(),xr_sqrt(xr_init(163,1))));
  z=xr_near_int(x); 
  printf("cmp(x,nearint(x))=%d\n",xr_cmp(x,z));
  xr_timing(1);
  xr_dotdump("exp_test",x,1);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  printf("exp(0)=");
  x=xr_exp(xr_init(0,1)); xr_print(x,digits);  printf("\n");  
  return 0;
}
