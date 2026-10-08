-- |
-- Module:      Math.NumberTheory.Roots.Squares
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Daniel Fischer <daniel.is.fischer@googlemail.com>
--
-- Functions dealing with fourth powers. Efficient calculation of integer fourth
-- roots and efficient testing for being a square's square.

{-# LANGUAGE CPP       #-}
{-# LANGUAGE MagicHash #-}
{- HLINT ignore "Use fewer imports" -}

module Math.NumberTheory.Roots.Fourth
    ( integerFourthRoot
    , integerFourthRoot'
    , exactFourthRoot
    , isFourthPower
    , isFourthPower'
    , isPossibleFourthPower
    ) where

import Data.Bits (finiteBitSize, (.&.))
import GHC.Exts (Int#, Ptr(..), int2Double#, double2Int#, isTrue#, sqrtDouble#, (<#))
import Numeric.Natural (Natural)

#ifdef MIN_VERSION_integer_gmp
import GHC.Exts (uncheckedIShiftRA#, (*#), (-#))
import GHC.Integer.GMP.Internals (Integer(..), shiftLInteger, shiftRInteger, sizeofBigNat#)
import GHC.Integer.Logarithms (integerLog2#)
#define IS S#
#define IP Jp#
#define bigNatSize sizeofBigNat
#else
import GHC.Exts (uncheckedShiftRL#, minusWord#, timesWord#)
import GHC.Num.BigNat (bigNatSize#)
import GHC.Num.Integer (Integer(..), integerLog2#, integerShiftR#, integerShiftL#)
#endif

import Math.NumberTheory.Utils.BitMask (indexBitSet)

-- | Calculate the integer fourth root of a nonnegative number,
--   that is, the largest integer @r@ with @r^4 <= n@.
--   Throws an error on negaitve input.
{-# SPECIALISE integerFourthRoot :: Int -> Int #-}
{-# SPECIALISE integerFourthRoot :: Word -> Word #-}
{-# SPECIALISE integerFourthRoot :: Integer -> Integer #-}
{-# SPECIALISE integerFourthRoot :: Natural -> Natural #-}
integerFourthRoot :: Integral a => a -> a
integerFourthRoot n
    | n < 0     = error "integerFourthRoot: negative argument"
    | otherwise = integerFourthRoot' n

-- | Calculate the integer fourth root of a nonnegative number,
--   that is, the largest integer @r@ with @r^4 <= n@.
--   The condition is /not/ checked.
{-# RULES
"integerFourthRoot'/Int"     integerFourthRoot' = biSqrtInt
"integerFourthRoot'/Word"    integerFourthRoot' = biSqrtWord
"integerFourthRoot'/Integer" integerFourthRoot' = biSqrtIgr
  #-}
{-# INLINE [1] integerFourthRoot' #-}
integerFourthRoot' :: Integral a => a -> a
integerFourthRoot' 0 = 0
integerFourthRoot' n = newton4 n (approxBiSqrt n)

-- | Returns @Nothing@ if @n@ is not a fourth power,
--   @Just r@ if @n == r^4@ and @r >= 0@.
{-# SPECIALISE exactFourthRoot :: Int -> Maybe Int #-}
{-# SPECIALISE exactFourthRoot :: Word -> Maybe Word #-}
{-# SPECIALISE exactFourthRoot :: Integer -> Maybe Integer #-}
{-# SPECIALISE exactFourthRoot :: Natural -> Maybe Natural #-}
exactFourthRoot :: Integral a => a -> Maybe a
exactFourthRoot 0 = Just 0
exactFourthRoot n
    | n < 0     = Nothing
    | isPossibleFourthPower n && r2*r2 == n = Just r
    | otherwise = Nothing
      where
        r = integerFourthRoot' n
        r2 = r*r

-- | Test whether an integer is a fourth power.
--   First nonnegativity is checked, then the unchecked
--   test is called.
{-# SPECIALISE isFourthPower :: Int -> Bool #-}
{-# SPECIALISE isFourthPower :: Word -> Bool #-}
{-# SPECIALISE isFourthPower :: Integer -> Bool #-}
{-# SPECIALISE isFourthPower :: Natural -> Bool #-}
isFourthPower :: Integral a => a -> Bool
isFourthPower 0 = True
isFourthPower n = n > 0 && isFourthPower' n

-- | Test whether a nonnegative number is a fourth power.
--   The condition is /not/ checked. If a number passes the
--   'isPossibleFourthPower' test, its integer fourth root
--   is calculated.
{-# SPECIALISE isFourthPower' :: Int -> Bool #-}
{-# SPECIALISE isFourthPower' :: Word -> Bool #-}
{-# SPECIALISE isFourthPower' :: Integer -> Bool #-}
{-# SPECIALISE isFourthPower' :: Natural -> Bool #-}
isFourthPower' :: Integral a => a -> Bool
isFourthPower' n = isPossibleFourthPower n && r2*r2 == n
  where
    r = integerFourthRoot' n
    r2 = r*r

-- | Test whether a nonnegative number is a possible fourth power.
--   The condition is /not/ checked.
--   This eliminates about 99.958% of numbers.
{-# SPECIALISE isPossibleFourthPower :: Int -> Bool #-}
{-# SPECIALISE isPossibleFourthPower :: Word -> Bool #-}
{-# SPECIALISE isPossibleFourthPower :: Integer -> Bool #-}
{-# SPECIALISE isPossibleFourthPower :: Natural -> Bool #-}
isPossibleFourthPower :: Integral a => a -> Bool
isPossibleFourthPower n'
  =  indexBitSet mask256 (fromInteger (n .&. 255))
  && indexBitSet mask425 (fromInteger (n `rem` 425))
  && indexBitSet mask377 (fromInteger (n `rem` 377))
  where
    n = toInteger n'

{-# SPECIALISE newton4 :: Integer -> Integer -> Integer #-}
newton4 :: Integral a => a -> a -> a
newton4 n a = go (step a)
      where
        step k = (3*k + n `quot` (k*k*k)) `quot` 4
        go k
            | m < k     = go m
            | otherwise = k
              where
                m = step k

{-# SPECIALISE approxBiSqrt :: Integer -> Integer #-}
approxBiSqrt :: Integral a => a -> a
approxBiSqrt = fromInteger . appBiSqrt . fromIntegral

-- Find a fairly good approximation to the fourth root.
-- About 48 bits should be correct for large Integers.
appBiSqrt :: Integer -> Integer
appBiSqrt (IS i#) = IS (double2Int# (sqrtDouble# (sqrtDouble# (int2Double# i#))))
appBiSqrt n@(IP bn#)
    | isTrue# (bigNatSize# bn# <# thresh#) =
          floor (sqrt . sqrt $ fromInteger n :: Double)
    | otherwise = case integerLog2# n of
#ifdef MIN_VERSION_integer_gmp
                    l# -> case uncheckedIShiftRA# l# 2# -# 47# of
                            h# -> case shiftRInteger n (4# *# h#) of
                                    m -> case floor (sqrt $ sqrt $ fromInteger m :: Double) of
                                            r -> shiftLInteger r h#
#else
                    l# -> case uncheckedShiftRL# l# 2# `minusWord#` 47## of
                            h# -> case integerShiftR# n (4## `timesWord#` h#) of
                                    m -> case floor (sqrt $ sqrt $ fromInteger m :: Double) of
                                            r -> integerShiftL# r h#
#endif
    where
        -- threshold for shifting vs. direct fromInteger
        -- we shift when we expect more than 256 bits
        thresh# :: Int#
        thresh# = if finiteBitSize (0 :: Word) == 64 then 5# else 9#

-- There's already a check for negative in integerFourthRoot,
-- but integerFourthRoot' is exported directly too.
appBiSqrt _ = error "integerFourthRoot': negative argument"

-----------------------------------------------------------------------------
-- Generated by 'Math.NumberTheory.Utils.BitMask.vectorToAddrLiteral'

mask256 :: Ptr Word
mask256 = Ptr "\ETX\NUL\ETX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL\STX\NUL"#

mask425 :: Ptr Word
mask425 = Ptr "\ETX\NUL!\NUL\NUL\NUL\f\NUL\NUL\NULB\NUL \EOT\NUL\NUL\NUL\SOH\NUL\NUL@\b\NUL\132\NUL\SOH\NUL \STX\NUL\NUL\b\SOH\128\DLE\NUL\NUL\NUL\EOT\NUL\NUL\NUL!\NUL\DLE\STX\128\NUL\128\NUL\NUL\NUL \NUL"#

mask377 :: Ptr Word
mask377 = Ptr "\ETX\NUL\SOH \NUL\NUL0\NUL\STXD\130@\NUL\b \NUL\NUL\b\EOT\SOH \ACK\NUL\NUL@\DLE\NUL\NUL\NUL\NUL\NUL\SOH!\NUL\NUL@\NUL\NUL\NUL\n@\NUL\b\NUL\NUL\DLE \NUL"#

-- biSqRes256 :: V.Vector Bool
-- biSqRes256 = runST $ do
--     ar <- MV.replicate 256 False
--     let note 257 = return ()
--         note i = MV.unsafeWrite ar i True >> note (i+16)
--     MV.unsafeWrite ar 0 True
--     MV.unsafeWrite ar 16 True
--     note 1
--     V.unsafeFreeze ar

-- biSqRes425 :: V.Vector Bool
-- biSqRes425 = runST $ do
--     ar <- MV.replicate 425 False
--     let note 154 = return ()
--         note i = MV.unsafeWrite ar ((i*i*i*i) `rem` 425) True >> note (i+1)
--     note 0
--     V.unsafeFreeze ar

-- biSqRes377 :: V.Vector Bool
-- biSqRes377 = runST $ do
--     ar <- MV.replicate 377 False
--     let note 144 = return ()
--         note i = MV.unsafeWrite ar ((i*i*i*i) `rem` 377) True >> note (i+1)
--     note 0
--     V.unsafeFreeze ar

-----------------------------------------------------------------------------
-- Specialisations for Int, Word, and Integer

biSqRootIntLimit :: Int
biSqRootIntLimit = if finiteBitSize (0 :: Word) == 64 then 55108 else 215

biSqrtInt :: Int -> Int
biSqrtInt 0 = 0
biSqrtInt n
    | r > biSqRootIntLimit = biSqRootIntLimit
    | n < r4    = r-1
    | otherwise = r
      where
        x :: Double
        x = fromIntegral n
        -- timed faster than x**0.25, never too small
        r = truncate (sqrt (sqrt x))
        r2 = r*r
        r4 = r2*r2

biSqRootWordLimit :: Word
biSqRootWordLimit = if finiteBitSize (0 :: Word) == 64 then 65535 else 255

biSqrtWord :: Word -> Word
biSqrtWord 0 = 0
biSqrtWord n
    | r > biSqRootWordLimit = biSqRootWordLimit
    | n < r4    = r-1
    | otherwise = r
      where
        x :: Double
        x = fromIntegral n
        r = truncate (sqrt (sqrt x))
        r2 = r*r
        r4 = r2*r2

biSqrtIgr :: Integer -> Integer
biSqrtIgr 0 = 0
biSqrtIgr n = newton4 n (approxBiSqrt n)
