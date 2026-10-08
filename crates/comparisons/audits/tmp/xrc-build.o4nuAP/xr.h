/*
  xrc-1.1
  K M Briggs 2003 Mar 26
  Keith.Briggs@bt.com
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <gmp.h>

#ifdef TIMING
  #include <sys/timeb.h>
  #include <sys/time.h>
#endif

extern const char* xr_version;
struct _node;
typedef struct _node* xr_t;

/* xr internal  functions */

int _msd(struct _node* z, int mx);
void _eval(mpz_t w, struct _node* z, const int n);

/* xr user utility functions */

void xr_timing(int);
void xr_dotdump(const char* dotfn, const struct _node* a, const int sc);
double xr_get_d(const xr_t x, int n);
char* xr_get_str(const xr_t x, int n);
int xr_print(const xr_t x, const int n);
int xr_print_nl(const xr_t x, const int n);
int xr_get_b(void);
void xr_set_b(int);

/* xr user arithmetic functions */

xr_t xr_init(long a, long b);
xr_t xr_set(const xr_t a);
void xr_free(const xr_t x);
void xr_eval(mpz_t z, const xr_t x, const int n); /* FIXME should we expose this? */

/* onary */
xr_t xr_pi(void);

/* unary */
xr_t xr_abs(const xr_t x);
xr_t xr_neg(const xr_t x);
xr_t xr_recip(const xr_t x);
xr_t xr_sqrt(const xr_t x);
xr_t xr_sqr(const xr_t x);
xr_t xr_exp(const xr_t x);
xr_t xr_exp1(const xr_t x);

/* bunary */
xr_t xr_iadd(const long x, const xr_t y);
xr_t xr_isub(const long x, const xr_t y);
xr_t xr_imul(const long x, const xr_t y);

/* ubnary */
xr_t xr_subi(const xr_t x, const long y);
xr_t xr_divi(const xr_t x, const long y);
xr_t xr_root(const xr_t x, const long n);

/* binary  */
xr_t xr_add(const xr_t x, const xr_t y);
xr_t xr_sub(const xr_t x, const xr_t y);
xr_t xr_mul(const xr_t x, const xr_t y);

/* non-primitive  */
int xr_cmp(xr_t a, xr_t b);
long xr_log2_bound(xr_t x);
xr_t xr_near_int(const xr_t x);
xr_t xr_powi(const xr_t x, long n);
xr_t xr_div(const xr_t x, const xr_t y);
