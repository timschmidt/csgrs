// gcc xr_rc.c xr.o -lgmp -lm && ./a.out

/*
From: Richard G. Clegg [richard@richardclegg.org]
Sent: 20 July 2006 19:41
To: Briggs,KM,Keith,CXR2 R
Subject: xr.c

Hi Keith,
	Hope all is well with you?  I've been doing some coding with your xr.c 
library which has been a mostly painless experience though, 
unfortunately, I can find no good solution to memory deallocation 
(without adding reference counting to your code).

Some quick comments:

1) there's no prototype for xr_print_nl in xr.h.
2) the header xr.h insists that gmp.h is in /usr/local/lib.  I think it 
would be better just to include this since the Makefile can be edited to 
include /usr/local/lib and it seems better if the end user can edit the 
Makefile not the source
3) there appears to be a bug in the special code you put in the imul 
routine when multiplying by 1 (the code also fails when multiplying by 
-1).  I notice you have special routines for these numbers but I don't 
have a deep enough understanding of your code to suggest a fix.

Minimal code exhibiting problem:
*/

#include <stdio.h>
#include "xr.h"

int main()
{
   xr_t x,y;
   x= xr_init(1,1);
   printf("x= ");
   xr_print(x,3);
   printf("\n");
   y= xr_imul(1,x);
   printf("y= ");
   xr_print(y,3);
   printf("\n");
   return 0;
}
/*
I hope your work is going well -- I've come up with some interesting new 
queuing theory result based upon my work with Markov Chains.

-- 
Richard G. Clegg,
Networks & NonLinear Dynamics Group,
Dept. of Maths, Uni. of York.
http://www.richardclegg.org/
*/
