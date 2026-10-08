/* gcc -O example.c xr.o -lm -lgmp -o example */

#include "xr.h"  

int main() { /* test whether exp(pi*sqrt(163)) is integral */ 
  xr_t x;
  x=xr_exp(xr_mul(xr_pi(),xr_sqrt(xr_init(163,1))));
  printf("%d\n",xr_cmp(x,xr_near_int(x)));
  return 0;
}

