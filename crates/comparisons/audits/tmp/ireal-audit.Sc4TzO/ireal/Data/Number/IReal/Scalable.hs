module Data.Number.IReal.Scalable where

import Data.Bits
import Data.Ratio

import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.Auxiliary

-- | Scaling. @scale x n@ computes @x * 2^n@ using bit shifts.
class Scalable a where
  scale :: a -> Int -> a

-- | Correctly rounded result for negative n.
-- Rounds upwards when decimal part of unrounded result is .5
instance Scalable Integer where 
  scale x n
    |n >= 0 = shift x n
    |otherwise = shift (x + bit (-n-1)) n

instance (Integral a, Bits a) => Scalable (Ratio a) where 
  scale x n
    |n >= 0 = shift num n % den
    |otherwise = num % shift den (-n)
    where num = numerator x
          den = denominator x

instance Scalable IntegerInterval where
  scale i@(I (m,r)) n
        | n >= 0 = I (scale m n, scale r n)
        | otherwise = shift (lowerI i) n `upto` (-shift (-upperI i) n)

instance Scalable Double where
  scale = flip scaleFloat

class VarPrec a where
   prec :: Int -> a -> a
   precB :: Int -> a -> a
   prec d  = precB (dec2bits d)

instance VarPrec a => VarPrec [a] where
   prec d xs = map (prec d) xs
   precB b xs = map (precB b) xs

instance VarPrec Double where
   prec _  = id
   precB _ = id