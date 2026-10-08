// g++ poly_test.cc xr.o -lgmp
// mcopy -o poly_test.cc a:

#include "xr++.h"

int pnpoly(int npol, xr xp[], xr yp[], xr x, xr y);

int main() {
  xr xp[3],yp[3];
  xp[0]=xr(-1L); xp[1]=xr( 1L); xp[2]=xr(0L); 
  yp[0]=xr( 0L); yp[1]=xr( 0L); yp[2]=xr(1L); 
  cout<<pnpoly(3,xp,yp,xr(0L),xr(1,2))<<endl;;
  return 0;
}

int pnpoly(int npol, xr xp[], xr yp[], xr x, xr y) {
  int i,j,c=0;
  for (i=0, j=npol-1; i<npol; j=i++) {
    if ((((yp[i]<y) && (y<yp[j])) || ((yp[j]<y) && (y<yp[i]))) &&
        (x<((xp[j]-xp[i])*(y-yp[i])/(yp[j]-yp[i])+xp[i]))) { 
	    c=!c; }
  }
  return c;
}

/*
int pnpoly(int npol, double *xp, double *yp, double x, double y);

int main() {
  double x,y,xp[3],yp[3];
  xp[0]=-1; xp[1]= 1; xp[2]=0; 
  yp[0]= 0; yp[1]= 0; yp[2]=1; 
  printf("%d\n",pnpoly(3,xp,yp,0,0.5));
  return 0;
}

int pnpoly(int npol, double *xp, double *yp, double x, double y) {
  int i,j,c=0;
  for (i=0, j=npol-1; i<npol; j=i++) {
    if ((((yp[i]<y) && (y<yp[j])) || ((yp[j]<y) && (y<yp[i]))) &&
        (x<(xp[j]-xp[i])*(y-yp[i])/(yp[j]-yp[i])+xp[i]))
      c=!c;
  }
  return c;
}
*/
