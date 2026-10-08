{-# LANGUAGE MagicHash #-}

{- |The IntegerLog module provides a fast integer logarithm in base 2. Current implementation relies in the integer-gmp package.
-}
module Data.CDAR.IntegerLog (integerLog2) where

import GHC.Integer.Logarithms -- only reason for needing integer-gmp package
import GHC.Exts

{-# INLINE integerLog2 #-}
{- |The 'integerLog2' function gives the floor of the logarithm of a positive integer. For non-positive arguments the returned result is meaningless. Possible overflow of returned value is ignored.

It could be argued that NegInf should be returned for 0 if the return type had been 'Extended.Extended' 'Int'.
-}
integerLog2 :: Integer -> Int
integerLog2 n = I# (integerLog2# n)

{- Should be able to rely on some provided integer logarithms from ghc 7.2
   onwards. A Haskell2010 version previously used follows. 

integerLog2 :: Integer -> Int
integerLog2 i =
    if i < 2 then 0
    else integerLog2' i 1 2

integerLog2' :: Integer -> Int -> Int -> Int
integerLog2' i l h =
    if i < bit h then
       integerLog2'' i l h
    else
       integerLog2' i h (2*h)

integerLog2'' :: Integer -> Int -> Int -> Int
integerLog2'' i l h =
    if (h-l) <= 1 then
       l
    else
       if i < bit m then
          integerLog2'' i l m
       else
          integerLog2'' i m h
    where m = (l+h) `div` 2
-}
