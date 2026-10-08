module Integrals where

import Data.Number.IReal
import Taylor

integral k d f i = try eps i 
  where
   eps = 10 ^^ (-d)
   evens (x:_:xs) = x : evens xs
   try eps i
     | w < eps =  approx + ((-w) -+- w)
     | otherwise = try eps2 (lower i -+- m) + try eps2 (m -+- upper i)
         where  eps2 = scale eps (-1)
                m = mid i
                approx = 2 * sum (take (k `div` 2 + 1) (evens (zipWith (*) ts rs)))
                w =  upper (abs (prec d $ (ts2!!k - ts!!k))) * rs!!k * 2
                ts = taylorCoeffs f m
                ts2 = taylorCoeffs f i
                rs = zipWith (/) (iterate (*rad i) (rad i)) [1..]

{-
arcLength k eps f = integral k eps g  
    where g x = sqrt (1 + deriv 1 f x ^ 2)
-}