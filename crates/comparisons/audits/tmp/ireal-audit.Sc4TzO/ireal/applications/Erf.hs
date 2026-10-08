module Erf where

import Data.Number.IReal.IRealOperations
import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.IReal
import Data.Number.IReal

{-

To illustrate how to extend the library with e.g. special functions we provide here the example of
the error function erf and the complementary error function erfc. 

We follow Brent and Zimmermann's Modern Computer Arithmetic, chapter 4.

We use the MacLaurin series which follows directly from the fact 
that the derivative of erf is \x -> 2/sqrt pi * exp(-x^2). This series 
actually converges for all x, but convergence is slow for large |x| and
no range reduction is known. For large |x| an asymptotic expansion for erfc 
is preferrable, but we have not implemented this, but accept only arguments with
|x| < 100. Note also that for x > 100, erfc x < 1e-4345.
-} 

erf, erfc :: IReal -> IReal
erf x 
 | midI (abs (appr x 0)) <= 100 = 2/sqrt pi * x * g (sq x)
 | otherwise            = error "erf(x) only implemented for |x| < 100; for bigger argument it differs from +-1 with at most 1e-4345" 
 where g = powerSeries (\a n -> negate (a *(2*n-1) `div`(n * (2*n+1)))) 2 id


erfc x = 1 - erf x