-- |
-- Module:      Math.NumberTheory.Roots.Cubes
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Daniel Fischer <daniel.is.fischer@googlemail.com>
--
-- Functions dealing with cubes. Moderately efficient calculation of integer
-- cube roots and testing for cubeness.

{-# LANGUAGE BangPatterns #-}
{-# LANGUAGE CPP          #-}
{-# LANGUAGE MagicHash    #-}
{- HLINT ignore "Use fewer imports" -}

module Math.NumberTheory.Roots.Cubes
    ( integerCubeRoot
    , integerCubeRoot'
    , exactCubeRoot
    , isCube
    , isCube'
    , isPossibleCube
    ) where

import Data.Bits (finiteBitSize, (.&.))
import GHC.Exts (Int#, Ptr(..), int2Double#, double2Int#, isTrue#, (/##), (**##), (<#))
import Numeric.Natural (Natural)

#ifdef MIN_VERSION_integer_gmp
import GHC.Exts (quotInt#, (*#), (-#))
import GHC.Integer.GMP.Internals (Integer(..), shiftLInteger, shiftRInteger, sizeofBigNat#)
import GHC.Integer.Logarithms (integerLog2#)
#define IS S#
#define IP Jp#
#define bigNatSize sizeofBigNat
#else
import GHC.Exts (minusWord#, timesWord#, quotWord#)
import GHC.Num.BigNat (bigNatSize#)
import GHC.Num.Integer (Integer(..), integerLog2#, integerShiftR#, integerShiftL#)
#endif

import Math.NumberTheory.Utils.BitMask (indexBitSet)

-- | For a given \( n \)
--   calculate its integer cube root \( \lfloor \sqrt[3]{n} \rfloor \).
--   Note that this is not symmetric about 0.
--
-- >>> map integerCubeRoot [7, 8, 9]
-- [1,2,2]
-- >>> map integerCubeRoot [-7, -8, -9]
-- [-2,-2,-3]
{-# SPECIALISE integerCubeRoot :: Int -> Int #-}
{-# SPECIALISE integerCubeRoot :: Word -> Word #-}
{-# SPECIALISE integerCubeRoot :: Integer -> Integer #-}
{-# SPECIALISE integerCubeRoot :: Natural -> Natural #-}
integerCubeRoot :: Integral a => a -> a
integerCubeRoot 0 = 0
integerCubeRoot n
    | n > 0     = integerCubeRoot' n
    | otherwise =
      let m = negate n
          r = if m < 0
                then negate . fromInteger $ integerCubeRoot' (negate $ fromIntegral n)
                else negate (integerCubeRoot' m)
      in if r*r*r == n then r else r - 1

-- | Calculate the integer cube root of a nonnegative integer @n@,
--   that is, the largest integer @r@ such that @r^3 <= n@.
--   The precondition @n >= 0@ is not checked.
{-# RULES
"integerCubeRoot'/Int"     integerCubeRoot' = cubeRootInt'
"integerCubeRoot'/Word"    integerCubeRoot' = cubeRootWord
"integerCubeRoot'/Integer" integerCubeRoot' = cubeRootIgr
  #-}
{-# INLINE [1] integerCubeRoot' #-}
integerCubeRoot' :: Integral a => a -> a
integerCubeRoot' 0 = 0
integerCubeRoot' n = newton3 n (approxCuRt n)

-- | Calculate the exact integer cube root if it exists,
-- otherwise return 'Nothing'.
--
-- >>> map exactCubeRoot [-9, -8, -7, 7, 8, 9]
-- [Nothing,Just (-2),Nothing,Nothing,Just 2,Nothing]
{-# SPECIALISE exactCubeRoot :: Int -> Maybe Int #-}
{-# SPECIALISE exactCubeRoot :: Word -> Maybe Word #-}
{-# SPECIALISE exactCubeRoot :: Integer -> Maybe Integer #-}
{-# SPECIALISE exactCubeRoot :: Natural -> Maybe Natural #-}
exactCubeRoot :: Integral a => a -> Maybe a
exactCubeRoot 0 = Just 0
exactCubeRoot n
    | n < 0     =
      if m < 0
        then fmap (negate . fromInteger) $ exactCubeRoot (negate $ fromIntegral n)
        else fmap negate (exactCubeRoot m)
    | isPossibleCube n && r*r*r == n    = Just r
    | otherwise = Nothing
      where
        m = negate n
        r = integerCubeRoot' n

-- | Test whether the argument is a perfect cube.
--
-- >>> map isCube [-9, -8, -7, 7, 8, 9]
-- [False,True,False,False,True,False]
{-# SPECIALISE isCube :: Int -> Bool #-}
{-# SPECIALISE isCube :: Word -> Bool #-}
{-# SPECIALISE isCube :: Integer -> Bool #-}
{-# SPECIALISE isCube :: Natural -> Bool #-}
isCube :: Integral a => a -> Bool
isCube 0 = True
isCube n
    | n > 0     = isCube' n
    | m > 0     = isCube' m
    | otherwise = isCube' (negate (fromIntegral n) :: Integer)
      where
        m = negate n

-- | Test whether a nonnegative integer is a cube.
--   Before 'integerCubeRoot' is calculated, a few tests
--   of remainders modulo small primes weed out most non-cubes.
--   On average, assuming that the majority of inputs aren't cubes,
--   this is much faster than @let r = cubeRoot n in r*r*r == n@.
--   The condition @n >= 0@ is /not/ checked.
{-# SPECIALISE isCube' :: Int -> Bool #-}
{-# SPECIALISE isCube' :: Word -> Bool #-}
{-# SPECIALISE isCube' :: Integer -> Bool #-}
{-# SPECIALISE isCube' :: Natural -> Bool #-}
isCube' :: Integral a => a -> Bool
isCube' !n = isPossibleCube n
             && (r*r*r == n)
      where
        r    = integerCubeRoot' n

-- | Test whether a nonnegative number is possibly a cube.
--   Only about 0.08% of all numbers pass this test.
--   The precondition @n >= 0@ is /not/ checked.
{-# SPECIALISE isPossibleCube :: Int -> Bool #-}
{-# SPECIALISE isPossibleCube :: Word -> Bool #-}
{-# SPECIALISE isPossibleCube :: Integer -> Bool #-}
{-# SPECIALISE isPossibleCube :: Natural -> Bool #-}
isPossibleCube :: Integral a => a -> Bool
isPossibleCube n'
  =  indexBitSet mask512 (fromInteger (n .&. 511))
  && indexBitSet mask837 (fromInteger (n `rem` 837))
  && indexBitSet mask637 (fromInteger (n `rem` 637))
  && indexBitSet mask703 (fromInteger (n `rem` 703))
  where
    n = toInteger n'

----------------------------------------------------------------------
--                         Utility Functions                        --
----------------------------------------------------------------------

-- Special case for 'Int', a little faster.
-- For @n <= 2^64@, the truncated 'Double' is never
-- more than one off. Things might overflow for @n@
-- close to @maxBound@, so check for overflow.
cubeRootInt' :: Int -> Int
cubeRootInt' 0 = 0
cubeRootInt' n
    | n < c || c < 0    = r-1
    | 0 < d && d < n    = r+1
    | otherwise         = r
      where
        x = fromIntegral n :: Double
        r = truncate (x ** (1/3))
        c = r*r*r
        d = c+3*r*(r+1)

cubeRootWordLimit :: Word
cubeRootWordLimit = if finiteBitSize (0 :: Word) == 64 then 2642245 else 1625

cubeRootWord :: Word -> Word
cubeRootWord 0 = 0
cubeRootWord w
    | r > cubeRootWordLimit = cubeRootWordLimit
    | w < c             = r-1
    | c < w && e < w && c < e  = r+1
    | otherwise         = r
      where
        r = truncate (fromIntegral w ** (1/3) :: Double)
        c = r*r*r
        d = 3*r*(r+1)
        e = c+d

cubeRootIgr :: Integer -> Integer
cubeRootIgr 0 = 0
cubeRootIgr n = newton3 n (approxCuRt n)

{-# SPECIALISE newton3 :: Integer -> Integer -> Integer #-}
newton3 :: Integral a => a -> a -> a
newton3 n a = go (step a)
      where
        step k = (2*k + n `quot` (k*k)) `quot` 3
        go k
            | m < k     = go m
            | otherwise = k
              where
                m = step k

{-# SPECIALISE approxCuRt :: Integer -> Integer #-}
approxCuRt :: Integral a => a -> a
approxCuRt 0 = 0
approxCuRt n = fromInteger $ appCuRt (fromIntegral n)

-- | approximate cube root, about 50 bits should be correct for large numbers
appCuRt :: Integer -> Integer
appCuRt (IS i#) = case double2Int# (int2Double# i# **## (1.0## /## 3.0##)) of
                    r# -> IS r#
appCuRt n@(IP bn#)
    | isTrue# (bigNatSize# bn# <# thresh#) =
          floor (fromInteger n ** (1.0/3.0) :: Double)
    | otherwise = case integerLog2# n of
#ifdef MIN_VERSION_integer_gmp
                    l# -> case (l# `quotInt#` 3#) -# 51# of
                            h# -> case shiftRInteger n (3# *# h#) of
                                    m -> case floor (fromInteger m ** (1.0/3.0) :: Double) of
                                           r -> shiftLInteger r h#
#else
                    l# -> case (l# `quotWord#` 3##) `minusWord#` 51## of
                            h# -> case integerShiftR# n (3## `timesWord#` h#) of
                                    m -> case floor (fromInteger m ** (1.0/3.0) :: Double) of
                                            r -> integerShiftL# r h#
#endif
    where
        -- threshold for shifting vs. direct fromInteger
        -- we shift when we expect more than 256 bits
        thresh# :: Int#
        thresh# = if finiteBitSize (0 :: Word) == 64 then 5# else 9#

-- There's already handling for negative in integerCubeRoot,
-- but integerCubeRoot' is exported directly too.
appCuRt _ = error "integerCubeRoot': negative argument"

-----------------------------------------------------------------------------
-- Generated by 'Math.NumberTheory.Utils.BitMask.vectorToAddrLiteral'

mask512 :: Ptr Word
mask512 = Ptr "\171\171\170\171\170\171\170\171\171\171\170\171\170\171\170\171\170\171\170\171\170\171\170\171\171\171\170\171\170\171\170\171\170\171\170\171\170\171\170\171\171\171\170\171\170\171\170\171\170\171\170\171\170\171\170\171\171\171\170\171\170\171\170\171"#

mask837 :: Ptr Word
mask837 = Ptr "\ETX\SOH\NUL\b\b@@@\SOH\NUL\NUL\n\NUL0\DLE \NUL\NUL\NUL\EOT\b\EOT\NULP\NUL\NUL\128\ETX\NUL\STX\DLE\NUL\NUL\128@\129\NUL\NUL\NUL\EOT \NUL\160\NUL\NUL\NUL\ENQ\NUL\DLE\b0\NUL\NUL\NUL\ETX\EOT\STX\NUL(\NUL\NUL@\SOH\NUL\SOH\b\NUL\NUL@\160@\NUL\NUL\NUL\STX\DLE\NULp\NUL\NUL\128\STX\NUL\b\EOT\b\NUL\NUL\NUL\SOH\STX\ETX\NUL\DC4\NUL\NUL\160\128\128\NUL\EOT\EOT\NUL \DLE"#

mask637 :: Ptr Word
mask637 = Ptr "\ETX!\NUL\b\EOT\NUL\NUL\STX\SOH@\b\DC4\b\SOH@ \NUL\NUL\DLE\b\NULB\160@\CAN\NUL\STX\SOH\NUL\128@\NUL\DLE\STX\ENQB@\DLE\b\NUL\NUL\EOT\130\128\DLE(\DLE\STX\128@\NUL\NUL \DLE\NUL\134@\129\DLE\NUL\EOT\STX\NUL\NUL\129\NUL \EOT\n\132\NUL \DLE\NUL\NUL\b\EOT\NUL!\DLE"#

mask703 :: Ptr Word
mask703 = Ptr "\ETX\t\NUL\140` \NUL\NUL\DC1\b\DLE\SOH\128\NUL\NUL&@\DLE\NUL\128\NUL\b\b\128\NUL\NUL\SOH!\DLE\DLE\NUL\SOH\f\n\STX`\NUL\ETX\SOH\NUL\f@\160\NUL\NUL\ENQ\STX0\NUL\128\192\NUL\ACK@P0\128\NUL\b\b\132\128\NUL\NUL\SOH\DLE\DLE\NUL\SOH\NUL\b\STXd\NUL\NUL\SOH\128\b\DLE\136\NUL\NUL\EOT\ACK1\NUL\144@"#

-- -- not very discriminating, but cheap, so it's an overall gain
-- cr512 :: V.Vector Bool
-- cr512 = runST $ do
--     ar <- MV.replicate 512 True
--     let note s i
--             | i < 512   = MV.unsafeWrite ar i False >> note s (i+s)
--             | otherwise = return ()
--     note 4 2
--     note 8 4
--     note 32 16
--     note 64 32
--     note 256 128
--     MV.unsafeWrite ar 256 False
--     V.unsafeFreeze ar

-- -- Remainders modulo @3^3 * 31@
-- cubeRes837 :: V.Vector Bool
-- cubeRes837 = runST $ do
--     ar <- MV.replicate 837 False
--     let note 837 = return ()
--         note k = MV.unsafeWrite ar ((k*k*k) `rem` 837) True >> note (k+1)
--     note 0
--     V.unsafeFreeze ar

-- -- Remainders modulo @7^2 * 13@
-- cubeRes637 :: V.Vector Bool
-- cubeRes637 = runST $ do
--     ar <- MV.replicate 637 False
--     let note 637 = return ()
--         note k = MV.unsafeWrite ar ((k*k*k) `rem` 637) True >> note (k+1)
--     note 0
--     V.unsafeFreeze ar

-- -- Remainders modulo @19 * 37@
-- cubeRes703 :: V.Vector Bool
-- cubeRes703 = runST $ do
--     ar <- MV.replicate 703 False
--     let note 703 = return ()
--         note k = MV.unsafeWrite ar ((k*k*k) `rem` 703) True >> note (k+1)
--     note 0
--     V.unsafeFreeze ar
