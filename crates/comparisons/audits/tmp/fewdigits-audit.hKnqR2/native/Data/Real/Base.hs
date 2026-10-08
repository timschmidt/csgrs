module Data.Real.Base (Base, approxBase, powers, sumBase) where

import Data.Real.Gauge
import Ratio
import Data.Bits
import Test.QuickCheck
import GHC.Real

type Base = Rational

approxBase :: Base -> Gauge -> Base
approxBase x e | 0 < err  = (round $ x*(fromIntegral ((1::Integer) `shiftL` (err)))) % 
                              (1 `shiftL` (err))
               | otherwise = fromInteger $
  (round $ x/(fromIntegral ((1::Integer) `shiftL` (-err)))) `shiftL` (-err)
 where
  (n,d) = (numerator e, denominator e)
  err = (bitLength (d-1))-(bitLength n)

prop_approxBase x e = e /= 0 ==> abs (x - y) <= (abs e)
 where
  y = approxBase x (abs e)

powers :: Base -> [Base]
powers x = zipWith (:%) (iterate (*numerator x) 1) 
                        (iterate (*denominator x) 1)

bitLength x' | x == 0 = -1
             | otherwise = bitL 0 $ until (\y -> x < (1 `shiftL` y)) (2*) 1
 where
  x = abs x'
  bitL low high | good low = low
                | good high = high
                | upper mid = bitL low mid
                | lower mid = bitL mid high
   where
    mid = (high+low) `div` 2
    lower y = (1 `shiftL` y) <= x
    upper y = x < (1 `shiftL` (y+1))
    good y =  lower y && upper y

prop_bitLength :: Integer -> Bool
prop_bitLength x' = (1 `shiftL` l) <= x && x < (1 `shiftL` (l+1))
 where
  l = bitLength x
  x = (abs x')+1

prop_bitLength2 :: Integer -> Bool
prop_bitLength2 x' = (1 `shiftL` (l-1)) < x && x <= (1 `shiftL` l)
 where
  l = (bitLength (x-1))+1
  x = (abs x')+1

sumBase :: [Base] -> Base
sumBase [] = 0
sumBase l = (sum [(lcd `div` d)*n |(n,d)<-l'])%lcd
 where
  l' = map (\x -> (numerator x,denominator x)) l
  lcd = lcm' l'
  lcm' [] = undefined
  lcm' [(_,a)] = a
  lcm' x = lcm (lcm' a) (lcm' b)
   where
    (a,b) = splitAt ((length x) `div`2) x


