// g++ -O example.cc xr.o -lm -lgmp -o example

#include "xr++.h"  

int main() { // test whether exp(pi*sqrt(163)) is integral
  xr x;
  x=exp(pi()*sqrt(xr(163)));
  cout<<(x>near_int(x))<<endl;
  return 0;
}

