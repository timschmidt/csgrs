#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "xr.h"

static uint64_t now_ns(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC,&ts);
  return (uint64_t)ts.tv_sec*UINT64_C(1000000000)+(uint64_t)ts.tv_nsec;
}

int main(int argc,char **argv)
{
  int i,b,iterations,precision,repeats;
  uint64_t start,first,repeated;
  xr_t x;
  mpz_t z;
  if (argc!=5) return 64;
  b=atoi(argv[1]);
  iterations=atoi(argv[2]);
  precision=atoi(argv[3]);
  repeats=atoi(argv[4]);
  xr_set_b(b);
  x=xr_init(43,64);
  for (i=0;i<iterations;i++) x=xr_imul(4,xr_mul(x,xr_isub(1,x)));
  mpz_init(z);
  start=now_ns();
  xr_eval(z,x,precision);
  first=now_ns()-start;
  start=now_ns();
  for (i=0;i<repeats;i++) xr_eval(z,x,precision);
  repeated=now_ns()-start;
  printf("first_ns=%llu repeat_avg_ns=%.3f low_bit=%d\n",
    (unsigned long long)first,(double)repeated/repeats,mpz_tstbit(z,0));
  mpz_clear(z);
  return 0;
}
