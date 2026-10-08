module Data.Number.IReal.Auxiliary where

import GHC.Num

infix 1 `atDecimals` 

-- Auxiliary functions and constants --------------------------------------------

-- | Base 2 logarithm of argument, rounded downwards.
lg2 :: Integer -> Int
lg2 = fromIntegral . GHC.Num.integerLogBase 2

-- | Converts precisions from decimal to binary.
dec2bits ::  Int -> Int
dec2bits d = ceiling (fromIntegral d * logBase 2 10 :: Double) 

-- | Operator allowing function expecting binary precision to be applied to decimal ditto.
atDecimals :: (Int -> a) -> Int -> a
atDecimals f = f . dec2bits

