// gcc poly_test.c xr.o -lgmp -lm
// mcopy -o poly_test.c a:

#include "xr.h"

int pnpoly(int npol, xr_t xp[], xr_t yp[], xr_t x, xr_t y);

int main() {
  xr_t xp[3],yp[3];
  xr_set_b(3);
  xp[0]=xr_init(-1L,1); xp[1]=xr_init( 1L,1); xp[2]=xr_init(0L,1); 
  yp[0]=xr_init( 0L,1); yp[1]=xr_init( 0L,1); yp[2]=xr_init(1L,1); 
  printf("%d\n",pnpoly(3,xp,yp,xr_init(0,1),xr_init(1,2)));
  return 0;
}

int pnpoly(int npol, xr_t xp[], xr_t yp[], xr_t x, xr_t y) {
  int i,j,c=0;
  for (i=0, j=npol-1; i<npol; j=i++) {
    if (((xr_cmp(yp[i],y)<1 && xr_cmp(y,yp[j])<1) 
      || (xr_cmp(yp[j],y)<1 && xr_cmp(y,yp[i])<1)) &&
        xr_cmp(x,xr_add(xr_div(xr_mul(xr_sub(xp[j],xp[i]),xr_sub(y,yp[i])),xr_sub(yp[j],yp[i])),xp[i]))<1
	)
	    { c=!c; }
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
