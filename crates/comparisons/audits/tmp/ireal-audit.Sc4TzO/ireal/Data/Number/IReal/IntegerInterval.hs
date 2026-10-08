module Data.Number.IReal.IntegerInterval where

import Data.Number.IReal.Powers
import Data.Bits

data IntegerInterval = I (Integer, Integer) deriving Show  -- (lower,upper)

-- make sure interval is not too thin
upto :: Integer -> Integer -> IntegerInterval
l `upto` u = I (l, max u (l+2))

midI, radI, lowerI, upperI :: IntegerInterval -> Integer
midI (I (l,u)) = shift (l+u) (-1)
radI (I (l,u)) = shift (u-l+1) (-1)
lowerI (I (l,_)) = l
upperI (I (_,u)) = u

isThin :: IntegerInterval -> Bool
isThin (I (l,u)) = u == l+2

ivalCase :: IntegerInterval -> a -> a -> a -> a
ivalCase (I (l,u)) pos neg zer
  | l >= 0 = pos
  | u <= 0 = neg
  | otherwise = zer

instance Num IntegerInterval where
  I (l1,u1) + I (l2,u2) = I (l1+l2,u1+u2)
  i1@(I (l1,u1)) * i2@(I (l2,u2)) =
     ivalCase i1
       (f (l1*l2,u1*u2) (u1*l2,l1*u2) (u1*l2,u1*u2))
       (f (l1*u2,u1*l2) (u1*u2,l1*l2) (l1*u2,l1*l2))
       (f (l1*u2,u1*u2) (u1*l2,l1*u2) (min (l1*u2) (u1*l2),max (l1*l2) (u1*u2)))
     where f x y z = ivalCase i2 (I x) (I y) (I z)
  abs i@(I (l,u)) = ivalCase i i (-i) (0 `upto` max (-l) u)
  negate (I (l,u)) = I (-u,-l)
  signum i = ivalCase i 1 (-1) (error "signum (for IntegerInterval): argument includes 0")
  fromInteger n = I (n-1,n+1) 

instance Powers IntegerInterval where
  pow i@(I (l,u)) n
   |even n = ivalCase i (I (l^n,u^n)) (I (u^n,l^n)) (0 `upto` (max (-l) u) ^ n)
   |otherwise =I (l^n,u^n)
