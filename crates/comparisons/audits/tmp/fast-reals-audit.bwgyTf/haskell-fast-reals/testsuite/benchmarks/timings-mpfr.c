/* timings-mpfr - program to measure the efficiency of GNU MPFR

Copyright 2001, 2010, 2011 Free Software Foundation, Inc.
Copyright 2001 Norbert Mueller (Univ. Trier, Germany).
Contributed by the Arenaire and Caramel projects, INRIA.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 3 of the License, or (at your
option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program; see the file COPYING.LESSER.  If not, see
http://www.gnu.org/licenses/ or write to the Free Software Foundation, Inc.,
51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA. */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "gmp.h"
#include "mpfr.h"

#if defined (USG) || defined (__SVR4) || defined (_UNICOS) || defined(HPUX)
#include <time.h>

int
cputime ()
{
  if (CLOCKS_PER_SEC < 100000)
    return clock () * 1000 / CLOCKS_PER_SEC;
  return clock () / (CLOCKS_PER_SEC / 1000);
}
#else
#include <sys/types.h>
#include <sys/resource.h>

static int
cputime ()
{
  struct rusage rus;

  getrusage (0, &rus);
  return rus.ru_utime.tv_sec * 1000 + rus.ru_utime.tv_usec / 1000;
}
#endif

int
main (int argc, char *argv[])
{
  int n, prec, st, st2, N, i;
  mpfr_t x, y, z;
  
  if (argc != 2 && argc != 3)
    {
      fprintf(stderr, "Usage: timing digits \n");
      exit(1);
    }

  printf ("Using MPFR-%s with GMP-%s\n", mpfr_version, gmp_version);
  n = atoi(argv[1]);
  prec = (int) ( n * log(10.0) / log(2.0) + 1.0 );
  printf("[precision is %u bits]\n", prec);
 
  mpfr_init2(x, prec); mpfr_init2(y, prec); mpfr_init2(z, prec); 

  mpfr_set_d(x, 3.0, GMP_RNDN); mpfr_sqrt(x, x, GMP_RNDN); mpfr_sub_ui (x, x, 1, GMP_RNDN);
  mpfr_set_d(y, 5.0, GMP_RNDN); mpfr_sqrt(y, y, GMP_RNDN);

  mpfr_log (z, x, GMP_RNDN);

  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_mul(z, x, y, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("x*y        took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);

  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_mul(z, x, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("x*x        took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);

  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_div(z, x, y, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("x/y        took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_sqrt(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("sqrt(x)    took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_exp(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("exp(x)     took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_log(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("log(x)     took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_sin(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000); 	  
  printf("sin(x)     took %f ms (%d eval in %d ms)\n", 
	 (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_cos(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000);
  printf("cos(x)     took %f ms (%d eval in %d ms)\n",
         (double)(st2-st)/(N-1),N-1,st2-st);

  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_acos(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000);
  printf("arccos(x)  took %f ms (%d eval in %d ms)\n",
         (double)(st2-st)/(N-1),N-1,st2-st);
  
  N=1;  st = cputime();
  do {
    for (i=0;i<N;i++) mpfr_atan(z, x, GMP_RNDN);
    N=2*N;
    st2=cputime();
  } while (st2-st<1000);
  printf("arctan(x)  took %f ms (%d eval in %d ms)\n",
         (double)(st2-st)/(N-1),N-1,st2-st);

  mpfr_clear(x); mpfr_clear(y); mpfr_clear(z);
  return 0;
}
