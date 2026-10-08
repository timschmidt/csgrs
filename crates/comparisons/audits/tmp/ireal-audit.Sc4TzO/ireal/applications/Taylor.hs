module Taylor where 

import Data.Number.IReal

facs :: (Fractional a, Num a) => [a]
facs = map fromInteger (scanl (*) 1 [1..])

generalTerms h = zipWith (/) (iterate (*h) 1) facs

taylorCoeffs f a = zipWith (/) (derivs f a) facs ++ repeat 0

-- Taylor polynomial of degree n of f at a, evaluated at x
taylorPoly f a n x = sum (zipWith (*) (take (n+1) (derivs f a)) (generalTerms (x-a)))

taylorRem f a n x = deriv n1 f (a -+- x) * (x-a)^n1 / facs !! n1
   where n1 = n+1

-- Taylor expansion of f at a of order n with Lagrange's remainder, evaluated at x.
taylor f a n x = taylorPoly f a n x + taylorRem f a n x