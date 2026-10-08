{-# LANGUAGE PolyKinds #-}
{-# LANGUAGE DataKinds #-}
module IntegralsRounded where

import Data.Number.IReal.Rounded
import Taylor

integral k d f i = try eps i 
  where
   eps = 10 ^^ (-d)
   evens (x:_:xs) = x : evens xs
   k1 = k `div` 2 + 1
   k2 = 2 * k1 - 1
   try eps i
     | w < eps =  1 -- approx + ((-w) -+- w)
     | otherwise = try eps2 (lower i -+- m) + try eps2 (m -+- upper i)
          where eps2 = eps / 2
                m = mid i
                approx = 2 * sum (take k1 (evens (zipWith (*) ts rs)))
                w =  2 * upper (abs (ts2!!k2 - ts!!k2)) * (rad i) ^ (k2+1)/fromIntegral (k2+1)
                ts = taylorCoeffs f m
                ts2 = taylorCoeffs f i
                rs = zipWith (/) (iterate (*rad i) (rad i)) (map fromIntegral [1..])


-- main = (integral 20 10 (\x ->sin (x + exp x)) (0 -+- 8) :: Rounded 80) ? 10

{-
time ./IntegralsRounded 
0.3474001727

real	0m8.769s
user	0m8.687s
sys	0m0.075s

Why so slow? 

-}
