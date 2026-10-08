/* K M Briggs 2003 Mar 28 */

#include "xr.h"

int main(int argc, char* argv[]) {
 xr_t a,b,c1,c2,c,d11,d2,d121;
 xr_t x1,x2,x3,x4,x5,xdiv;
  if (argc>1) xr_set_b(atoi(argv[1]));
  printf("starting %s using %s, b=%d, gmp version=%s\n",
	  argv[0],xr_version,xr_get_b(),gmp_version);
  printf("my_test...\n");
  xr_set_b(2);
  xr_timing(0);
 
  a=xr_init(617,1);
  b=xr_init(33096,1);
  xdiv=xr_div(a,b);
  printf("\n");
  printf("xdiv= \n");xr_print(xdiv,20);


  c1=xr_init(33375,100);
  c2=xr_init(55,10);
  d11=xr_init(11,1);
  d121=xr_init(121,1);
  d2=xr_init(2,1);
  printf("\n");
  printf("a= \n");xr_print(a,20);
  printf("\n");
  printf("b= \n");xr_print(b,20);
  printf("\n");
  printf("c1= \n");xr_print(c1,20);
  printf("\n");
  printf("c2= \n");xr_print(c2,20);
  printf("\n");
  printf("d11= \n");xr_print(d11,20);
  printf("\n");
  printf("d2= \n");xr_print(d2,20);
  printf("\n");
  printf("d121= \n");xr_print(d121,20);
  printf("\n");

/*    C = Cq1*B**6 + A**2*(d11*A**2*B**2-B**6-d121*B**4-d2) + Cq2*B**8+A/(d2*B)
 */
  x1=xr_mul(c1,xr_powi(b,6));
  printf("x1= \n");xr_print(x1,20);
  printf("\n");
  x2=xr_sub(xr_mul(d11,xr_mul(xr_powi(a,2),xr_powi(b,2))),xr_powi(b,6));
  printf("x2= \n");xr_print(x2,20);
  printf("\n");
  x3=xr_sub(x2,xr_mul(d121,xr_powi(b,4)));
  printf("x3= \n");xr_print(x3,20);
  printf("\n");
  x4=xr_sub(x3,d2);
  printf("x4= \n");xr_print(x4,20);
  printf("\n");
  xdiv=xr_div(xr_init(1,1),xr_init(2,1));
  /*  xdiv=xr_div(a,xr_mul(d2,b)); */
  printf("xdiv= \n");xr_print(xdiv,20);
  printf("\n");
  x5=xr_add(xr_mul(c2,xr_powi(b,8)),xdiv);
  printf("x5= \n");xr_print(x5,20);
  printf("\n");
  c=xr_add(xr_add(x1,xr_mul(xr_powi(a,2),x4)),x5);  
  printf("c= \n");xr_print(c,20);
  printf("\n");
  xr_timing(1);
  printf("finished %s, b=%d\n",argv[0],xr_get_b());
  return 0;
}
