#include <stdio.h>
#include <string.h>
#include "xr.h"

int main(int argc, char **argv)
{
  xr_t x;
  mpz_t z;
  if (argc != 2) return 64;
  if (strcmp(argv[1], "free-null") == 0) {
    xr_free(NULL);
    return 0;
  }
  x=xr_mul(xr_init(2,3),xr_init(5,7));
  if (strcmp(argv[1], "eval") == 0) {
    mpz_init(z);
    xr_eval(z,x,40);
    mpz_clear(z);
  } else if (strcmp(argv[1], "get-d") == 0) {
    printf("%.17g\n",xr_get_d(x,40));
  } else if (strcmp(argv[1], "print") == 0) {
    xr_print_nl(x,40);
  } else {
    return 64;
  }
  xr_free(x);
  return 0;
}
