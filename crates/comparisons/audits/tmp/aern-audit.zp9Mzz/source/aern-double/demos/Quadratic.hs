
module Main where

import Numeric.AERN.DoubleBasis.RealApprox

import Control.Monad.ST (runST)

type R = RealApprox

main = do
  putStrLn $ "quadratic 1 1 1 = " ++ show (quadratic 1 1 1)
  putStrLn $ "quadratic 1 2 1 = " ++ show (quadratic 1 2 1)
  putStrLn $ "quadratic 1 5 6 = " ++ show (quadratic 1 5 6)

-- | 
-- Returns list such that the union of its members contains all
-- real roots of each real polynomial u*x^2+v*x+w obtained by
-- choosing coefficients u in a, v in b and w in c. 
quadratic :: R -> R -> R -> [R]
quadratic a b c 
  | certainlyZero discriminant =
    [doubleRoot]
  | certainlyNonnegative discriminant =
    [leftRoot,rightRoot]
  | certainlyNegative discriminant =
    []
  | otherwise =
    [bottom]
  where
  discriminant = b^2-4*a*c
  doubleRoot = -b/(2*a)
  leftRoot = doubleRoot-sqrtDiscriminant/(2*a)  -- (-b-sqrt(b^2-4*a*c))/(2*a)
  rightRoot = doubleRoot+sqrtDiscriminant/(2*a) -- (-b+sqrt(b^2-4*a*c))/(2*a)
  sqrtDiscriminant = sqrt discriminant


certainlyZero x =
  case x ==? 0 of
    Just True -> True
    _ -> False 

certainlyNonnegative x =
  case x >=? 0 of
    Just True -> True
    _ -> False 

certainlyNegative x =
  case x <? 0 of
    Just True -> True
    _ -> False 
