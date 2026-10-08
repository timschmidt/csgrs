#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xr.h"

int main(int argc, char **argv)
{
  long i, depth;
  xr_t x;
  mpz_t z;
  if (argc != 3) return 64;
  depth=strtol(argv[1],NULL,10);
  x=xr_init(1,3);
  if (strcmp(argv[2],"neg") == 0) {
    for (i=0; i<depth; ++i) x=xr_neg(x);
  } else if (strcmp(argv[2],"iadd") == 0) {
    for (i=0; i<depth; ++i) x=xr_iadd(1,x);
  } else {
    return 64;
  }
  mpz_init(z);
  xr_eval(z,x,20);
  printf("bits=%lu sign=%d\n",(unsigned long)mpz_sizeinbase(z,2),mpz_sgn(z));
  mpz_clear(z);
  return 0;
}
