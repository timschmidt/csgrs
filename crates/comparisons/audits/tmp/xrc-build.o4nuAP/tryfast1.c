/* K M Briggs 2003 Apr 20 
   gcc tryfast1.c -lgmp -lm && a.out
   mcopy -o tryfast1.c a:
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <gmp.h>

const int b=1;

void mpz_ndiv_q_2exp(mpz_t rop, const mpz_t op, const unsigned long p);

unsigned long xcng(void);
double uni(void);

unsigned long xcng(void) { /* uniform integer [0,ULONG_MAX] */
  static unsigned long x=123456789,y=521288629,z,w;
  z=x+y; x=y; w=(z<<8); y=w+17*(z>>24);
  if (z<x) y+=4352; if (y<w) y+=17; return y;
}

double uni(void) { /* uniform double [0,1) */
  return 2.3283064365386962891e-10*xcng();
}

int uni_int(int n) {
  return (int)floor(n*2.3283064365386962891e-10*xcng());
}

#define MP_LIMB_T_SWAP(x, y)                    \
  do {                                          \
    mp_limb_t __mp_limb_t_swap__tmp = (x);      \
    (x) = (y);                                  \
    (y) = __mp_limb_t_swap__tmp;                \
  } while (0)
#define MP_SIZE_T_SWAP(x, y)                    \
  do {                                          \
    mp_size_t __mp_size_t_swap__tmp = (x);      \
    (x) = (y);                                  \
    (y) = __mp_size_t_swap__tmp;                \
  } while (0)

#define MP_PTR_SWAP(x, y)               \
  do {                                  \
    mp_ptr __mp_ptr_swap__tmp = (x);    \
    (x) = (y);                          \
    (y) = __mp_ptr_swap__tmp;           \
  } while (0)
#define MP_SRCPTR_SWAP(x, y)                    \
  do {                                          \
    mp_srcptr __mp_srcptr_swap__tmp = (x);      \
    (x) = (y);                                  \
    (y) = __mp_srcptr_swap__tmp;                \
  } while (0)
#define MPZ_PTR_SWAP(x, y)              \
  do {                                  \
    mpz_ptr __mpz_ptr_swap__tmp = (x);  \
    (x) = (y);                          \
    (y) = __mpz_ptr_swap__tmp;          \
  } while (0)
#define MPZ_SRCPTR_SWAP(x, y)                   \
  do {                                          \
    mpz_srcptr __mpz_srcptr_swap__tmp = (x);    \
    (x) = (y);                                  \
    (y) = __mpz_srcptr_swap__tmp;               \
  } while (0)
#define MPN_NORMALIZE(DST, NLIMBS) \
  do {									\
    while (NLIMBS > 0)							\
      {									\
	if ((DST)[(NLIMBS) - 1] != 0)					\
	  break;							\
	NLIMBS--;							\
      }									\
  } while (0)

#define VARIATION
void add_shift(mpz_ptr w, mpz_srcptr u, mpz_srcptr v, int p) {
  int rs,ls,rnd_bit,sgn;
  mp_srcptr up, vp;
  mp_ptr wp;
  mp_size_t usize, vsize, wsize;
  mp_size_t abs_usize;
  mp_size_t abs_vsize;
  mp_limb_t cy_limb;
  ls=(b*p)/mp_bits_per_limb; // limb shift
  rs=b*p-mp_bits_per_limb*ls;
  printf("add_shift: limb shift=%d remaining shift=%d\n",ls,rs);
  usize = u->_mp_size;
  vsize = VARIATION v->_mp_size;
  abs_usize = abs (usize);
  abs_vsize = abs (vsize);
  if (abs_usize < abs_vsize) { /* Swap U and V. */
    MPZ_SRCPTR_SWAP (u, v);
    MP_SIZE_T_SWAP (usize, vsize);
    MP_SIZE_T_SWAP (abs_usize, abs_vsize);
  }
  /* True: ABS_USIZE >= ABS_VSIZE.  */
  /* If not space for w (and possible carry), increase space.  */
  wsize = abs_usize + 1;
  wsize++; /* KMB: allow for possible carry in rounding step */
  if (w->_mp_alloc < wsize) _mpz_realloc (w, wsize);
  /* These must be after realloc (u or v may be the same as w).  */
  up = u->_mp_d;
  vp = v->_mp_d;
  wp = w->_mp_d;
  if ((usize ^ vsize) < 0) {
      /* U and V have different sign.  Need to compare them to determine
	 which operand to subtract from which.  */
      /* This test is right since ABS_USIZE >= ABS_VSIZE.  */
      if (abs_usize != abs_vsize) {
	  mpn_sub (wp, up, abs_usize, vp, abs_vsize);
	  wsize = abs_usize;
	  MPN_NORMALIZE (wp, wsize);
	  if (usize < 0) wsize = -wsize;
      } else if (mpn_cmp (up, vp, abs_usize) < 0) {
	  mpn_sub_n (wp, vp, up, abs_usize);
	  wsize = abs_usize;
	  MPN_NORMALIZE (wp, wsize);
	  if (usize >= 0) wsize = -wsize;
      } else {
	  mpn_sub_n (wp, up, vp, abs_usize);
	  wsize = abs_usize;
	  MPN_NORMALIZE (wp, wsize);
	  if (usize < 0) wsize = -wsize;
      }
  } else { /* U and V have same sign.  Add them.  */
    printf("add_shift: usize=%d\n",abs_usize);
    printf("add_shift: vsize=%d\n",abs_vsize);
    abs_usize-=ls;
    abs_vsize-=ls;
    if (abs_vsize>0) {
      cy_limb=mpn_add(wp,up+ls,abs_usize,vp+ls,abs_vsize);
      wp[abs_usize]=cy_limb;
      wsize=abs_usize+cy_limb;
      printf("add_shift: wsize=%d\n",wsize);
    } else  if (abs_usize>0) /* v will be shifted off => no add needed */
      mpn_add_1(wp,up+ls,abs_usize,0);
    else {
      w->_mp_size=0;
      return;
    }
    if (rs>0) {
      rnd_bit=(wp[0]&(1<<(rs-1)))>>(rs-1);
      printf("add_shift: rnd_bit=%d\n",rnd_bit);
      sgn=usize;
      if (usize<0) wsize=-wsize;
      mpn_rshift(wp,wp,wsize,rs);
      if (rnd_bit) {
        if (sgn>0) cy_limb=mpn_add_1(wp,wp,wsize,1);
        else       cy_limb=mpn_sub_1(wp,wp,wsize,1);
        wp[wsize]=cy_limb;
        wsize+=cy_limb;
      }
    }
  }
  w->_mp_size = wsize;
}

void test(int n) {
  int i,k,sz,p;
  mpz_t x,y,z,w;
  mpz_init2(x,110*mp_bits_per_limb);
  mpz_init2(y,110*mp_bits_per_limb);
  mpz_init(z);
  mpz_init(w);
  for (i=0; i<n; i++) {
     printf("i=%d\n",i);
     sz=4+uni_int(100); mpn_random(x->_mp_d,sz); x->_mp_size=sz;
     sz=sz-3; mpn_random(y->_mp_d,sz); y->_mp_size=sz;
     //printf("x="); mpz_out_str(stdout,10,x); printf("\n");
     //printf("y="); mpz_out_str(stdout,10,y); printf("\n");
     p=uni_int(100);
     mpz_add(z,x,y); mpz_ndiv_q_2exp(z,z,p);
     add_shift(w,x,y,p);
     k=mpz_cmp(z,w);
     if (k) {
       printf("k=%d\n",k);
       printf("p=%d\n",p);
       printf("z size=%d\n",z->_mp_size);
       printf("w size=%d\n",w->_mp_size);
       printf("z="); mpz_out_str(stdout,10,z); printf("\n");
       printf("w="); mpz_out_str(stdout,10,w); printf("\n");
       mpz_sub(z,z,w);
       printf("z-w=",p); mpz_out_str(stdout,10,w); printf("\n");
       exit(1);
     }
  }
  printf("all tests ok\n");
  exit(0);
}


int main() {
  int p=3;
  mpz_t x,y,z;
  test(500);
  mpz_init_set_ui(x,123456); mpz_pow_ui(x,x,21);
  mpz_init_set_ui(y,99991); mpz_pow_ui(y,y,21);
  //mpz_set_si(x,34); mpz_pow_ui(x,x,1);
  //mpz_set_si(y,9); mpz_pow_ui(y,y,1);
  mpz_init(z);
  printf("bits per limb=%d\n",mp_bits_per_limb);
  printf("high level:\n");
  mpz_add(z,x,y);
  printf("z="); mpz_out_str(stdout,10,z); printf("\n");
  mpz_ndiv_q_2exp(z,z,b*p);
  printf("ndiv(z,%d^%d)=",1<<b,p); mpz_out_str(stdout,10,z); printf("\n");
  printf("low level:\n");
  mpz_set(z,x); // make sure enough room
  add_shift(z,x,y,p);
  printf("      z/%d^%d=",1<<b,p); mpz_out_str(stdout,10,z); printf("\n");
  return 0;
}

void mpz_ndiv_q_2exp(mpz_t rop, const mpz_t op, const unsigned long p) {
 /* rop <- op/2^p, rounded to nearest (rounding away when middle) */
  if (p!=0) {
    int rnd_bit=mpz_tstbit(op,p-1);
    int sgn=mpz_sgn(op);
    mpz_tdiv_q_2exp(rop,op,p);
    if (rnd_bit) {
      if (sgn>0) mpz_add_ui(rop,rop,1);
      else       mpz_sub_ui(rop,rop,1);
    }
  }
  else mpz_set(rop,op);
  return;
}


