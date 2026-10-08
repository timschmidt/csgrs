#include <gmp.h>
#include <iostream>
#include <iomanip>

extern "C" {
  #include "xr.h"
}

using namespace std;

class xr {
  xr_t v;
public:
  xr(xr_t x) { v=x; }
  ~xr() { xr_free(v); }
  xr(long a=0, long b=1){
    v=xr_init(a,b);
  }
  friend xr abs(xr x);
  friend xr neg(xr x);
  friend xr sqrt(xr);
  friend xr exp(xr);
  friend xr operator+(xr   x, xr   y);
  friend xr operator+(xr   x, long y);
  friend xr operator+(long x, xr   y);
  friend xr operator-(xr   x, xr   y);
  friend xr operator-(xr   x, long y);
  friend xr operator-(long x, xr   y);
  friend xr operator-(xr   x);
  friend xr operator*(xr   x, xr   y);
  friend xr operator*(xr   x, long y);
  friend xr operator*(long x, xr   y);
  friend xr operator/(xr   x, xr   y);
  friend xr operator/(xr   x, long y);
  friend xr operator/(long x, xr   y);
  friend xr near_int(xr x);
  friend int cmp(xr x, xr y);
  friend int operator<(xr x, xr y);
  friend int operator>(xr x, xr y);
  void dec(int d) { xr_print((*this).v,d); }
  friend ostream& operator<<(ostream& s, xr x);
};

void set_b(int b) { xr_set_b(b); }
int  get_b(void ) { return xr_get_b(); }

xr operator+(xr   x, xr   y) { return xr_add(x.v,y.v); }
xr operator+(xr   x, long y) { return xr_iadd(y,x.v); }
xr operator+(long x, xr   y) { return xr_iadd(x,y.v); }

xr operator-(xr   x, xr   y) { return xr_sub(x.v,y.v); }
xr operator-(xr   x, long y) { return xr_subi(x.v,y); }
xr operator-(long x, xr   y) { return xr_isub(x,y.v); }
xr operator-(xr   x) { return xr_neg(x.v); }

xr operator*(xr   x, xr   y) { return xr_mul(x.v,y.v); }
xr operator*(long x, xr   y) { return xr_imul(x,y.v); }
xr operator*(xr   x, long y) { return xr_imul(y,x.v); }

xr operator/(xr   x, xr   y) { return xr_div(x.v,y.v); }
xr operator/(xr   x, long y) { return xr_divi(x.v,y); }
xr operator/(long x, xr   y) { return xr_recip(xr_divi(y.v,x)); }

xr near_int(xr x) { return xr_near_int(x.v); }
int cmp(xr x, xr y) { return xr_cmp(x.v,y.v); }
int operator<(xr x, xr y) { return xr_cmp(x.v,y.v)<0; }
int operator>(xr x, xr y) { return xr_cmp(x.v,y.v)>0; }

ostream& operator<<(ostream& s, xr x) { 
  char* y=xr_get_str(x.v,10);
  s<<y;
  free(y);
  return s;
}

xr abs(xr x) { return xr_abs(x.v); }
xr neg(xr x) { return xr_neg(x.v); }
xr exp(xr x) { return xr_exp(x.v); }
xr sqrt(xr x) { return xr_sqrt(x.v); }
xr pi(void) { return xr_pi(); }
