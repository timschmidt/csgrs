-- |
-- Module:      Math.NumberTheory.Roots.Squares.Internal
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Internal functions dealing with square roots. End-users should not import this module.

{-# LANGUAGE CPP              #-}
{-# LANGUAGE MagicHash        #-}
{- HLINT ignore "Use fewer imports" -}

module Math.NumberTheory.Roots.Squares.Internal
  ( karatsubaSqrt
  , isqrtA
  ) where

import Data.Bits (finiteBitSize, unsafeShiftL, unsafeShiftR, (.&.), (.|.))

import GHC.Exts (Int(..), Int#, isTrue#, int2Double#, sqrtDouble#, double2Int#, (<#))
#ifdef MIN_VERSION_integer_gmp
import GHC.Exts (uncheckedIShiftRA#, (*#), (-#))
import GHC.Integer.GMP.Internals (Integer(..), shiftLInteger, shiftRInteger, sizeofBigNat#)
import GHC.Integer.Logarithms (integerLog2#)
#define IS S#
#define IP Jp#
#define bigNatSize sizeofBigNat
#else
import GHC.Exts (uncheckedShiftRL#, word2Int#, minusWord#, timesWord#)
import GHC.Num.BigNat (bigNatSize#)
import GHC.Num.Integer (Integer(..), integerLog2#, integerShiftR#, integerShiftL#)
#endif

-- Find approximation to square root in 'Integer', then
-- find the integer square root by the integer variant
-- of Heron's method. Takes only a handful of steps
-- unless the input is really large.
{-# SPECIALISE isqrtA :: Integer -> Integer #-}
isqrtA :: Integral a => a -> a
isqrtA 0 = 0
isqrtA n = heron n (fromInteger . appSqrt . fromIntegral $ n)

-- Heron's method for integers. First make one step to ensure
-- the value we're working on is @>= r@, then we have
-- @k == r@ iff @k <= step k@.
{-# SPECIALISE heron :: Integer -> Integer -> Integer #-}
heron :: Integral a => a -> a -> a
heron n a = go (step a)
      where
        step k = (k + n `quot` k) `quot` 2
        go k
            | m < k     = go m
            | otherwise = k
              where
                m = step k

-- Find a fairly good approximation to the square root.
-- At most one off for small Integers, about 48 bits should be correct
-- for large Integers.
appSqrt :: Integer -> Integer
appSqrt (IS i#) = IS (double2Int# (sqrtDouble# (int2Double# i#)))
appSqrt n@(IP bn#)
    | isTrue# (bigNatSize# bn# <# thresh#) =
          floor (sqrt $ fromInteger n :: Double)
    | otherwise = case integerLog2# n of
#ifdef MIN_VERSION_integer_gmp
                    l# -> case uncheckedIShiftRA# l# 1# -# 47# of
                            h# -> case shiftRInteger n (2# *# h#) of
                                    m -> case floor (sqrt $ fromInteger m :: Double) of
                                            r -> shiftLInteger r h#
#else
                    l# -> case uncheckedShiftRL# l# 1# `minusWord#` 47## of
                            h# -> case integerShiftR# n (2## `timesWord#` h#) of
                                    m -> case floor (sqrt $ fromInteger m :: Double) of
                                            r -> integerShiftL# r h#
#endif
    where
        -- threshold for shifting vs. direct fromInteger
        -- we shift when we expect more than 256 bits
        thresh# :: Int#
        thresh# = if finiteBitSize (0 :: Word) == 64 then 5# else 9#
-- There's already a check for negative in integerSquareRoot,
-- but integerSquareRoot' is exported directly too.
appSqrt _ = error "integerSquareRoot': negative argument"


-- Integer square root with remainder, using the Karatsuba Square Root
-- algorithm from
-- Paul Zimmermann. Karatsuba Square Root. [Research Report] RR-3805, 1999,
-- pp.8. <inria-00072854>

karatsubaSqrt :: Integer -> (Integer, Integer)
karatsubaSqrt 0 = (0, 0)
karatsubaSqrt n
    | lgN < 2300 =
        let s = isqrtA n in (s, n - s * s)
    | otherwise =
        if lgN .&. 2 /= 0 then
            karatsubaStep k (karatsubaSplit k n)
        else
            -- before we split n into 4 part we must ensure that the first part
            -- is at least 2^k/4, since this doesn't happen here we scale n by
            -- multiplying it by 4
            let n' = n `unsafeShiftL` 2
                (s, r) = karatsubaStep k (karatsubaSplit k n')
                r' | s .&. 1 == 0 = r
                   | otherwise = r + double s - 1
            in  (s `unsafeShiftR` 1, r' `unsafeShiftR` 2)
  where
    k = lgN `unsafeShiftR` 2 + 1
#ifdef MIN_VERSION_integer_gmp
    lgN = I# (integerLog2# n)
#else
    lgN = I# (word2Int# (integerLog2# n))
#endif

karatsubaStep :: Int -> (Integer, Integer, Integer, Integer) -> (Integer, Integer)
karatsubaStep k (a3, a2, a1, a0)
    | r >= 0 = (s, r)
    | otherwise = (s - 1, r + double s - 1)
  where
    r = cat u a0 - q * q
    s = s' `unsafeShiftL` k + q
    (q, u) = cat r' a1 `quotRem` double s'
    (s', r') = karatsubaSqrt (cat a3 a2)
    cat x y = x `unsafeShiftL` k .|. y
    {-# INLINE cat #-}

karatsubaSplit :: Int -> Integer -> (Integer, Integer, Integer, Integer)
karatsubaSplit k n0 = (a3, a2, a1, a0)
  where
    a3 = n3
    n3 = n2 `unsafeShiftR` k
    a2 = n2 .&. m
    n2 = n1 `unsafeShiftR` k
    a1 = n1 .&. m
    n1 = n0 `unsafeShiftR` k
    a0 = n0 .&. m
    m = 1 `unsafeShiftL` k - 1

double :: Integer -> Integer
double x = x `unsafeShiftL` 1
{-# INLINE double #-}
