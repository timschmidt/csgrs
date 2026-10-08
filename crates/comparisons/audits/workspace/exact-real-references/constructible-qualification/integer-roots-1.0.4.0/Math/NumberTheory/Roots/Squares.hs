-- |
-- Module:      Math.NumberTheory.Roots.Squares
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Daniel Fischer <daniel.is.fischer@googlemail.com>
--
-- Functions dealing with squares. Efficient calculation of integer square roots
-- and efficient testing for squareness.

{-# LANGUAGE BangPatterns #-}
{-# LANGUAGE MagicHash    #-}

module Math.NumberTheory.Roots.Squares
    ( -- * Square root calculation
      integerSquareRoot
    , integerSquareRoot'
    , integerSquareRootRem
    , integerSquareRootRem'
    , exactSquareRoot
      -- * Tests for squares
    , isSquare
    , isSquare'
    , isPossibleSquare
    ) where

import Data.Bits (finiteBitSize, (.&.))
import Data.Int (Int64)
import Data.Word (Word64)
import GHC.Exts (Ptr(..))
import Numeric.Natural (Natural)

import Math.NumberTheory.Roots.Squares.Internal
import Math.NumberTheory.Utils.BitMask (indexBitSet)

-- | For a non-negative input \( n \)
--   calculate its integer square root \( \lfloor \sqrt{n} \rfloor \).
--   Throw an error on negative input.
--
-- >>> integerSquareRoot 99
-- 9
-- >>> integerSquareRoot 100
-- 10
-- >>> integerSquareRoot 101
-- 10
{-# SPECIALISE integerSquareRoot :: Int -> Int #-}
{-# SPECIALISE integerSquareRoot :: Word -> Word #-}
{-# SPECIALISE integerSquareRoot :: Int64 -> Int64 #-}
{-# SPECIALISE integerSquareRoot :: Word64 -> Word64 #-}
{-# SPECIALISE integerSquareRoot :: Integer -> Integer #-}
{-# SPECIALISE integerSquareRoot :: Natural -> Natural #-}
integerSquareRoot :: Integral a => a -> a
integerSquareRoot n
  | n < 0       = error "integerSquareRoot: negative argument"
  | otherwise   = integerSquareRoot' n

-- | Calculate the integer square root of a non-negative number @n@,
--   that is, the largest integer @r@ with @r*r <= n@.
--   The precondition @n >= 0@ is not checked.
{-# RULES
"integerSquareRoot'/Int"     integerSquareRoot' = isqrtInt'
"integerSquareRoot'/Word"    integerSquareRoot' = isqrtWord
"integerSquareRoot'/Int64"   integerSquareRoot' = isqrtInt64'
"integerSquareRoot'/Word64"  integerSquareRoot' = isqrtWord64
"integerSquareRoot'/Integer" integerSquareRoot' = isqrtInteger
"integerSquareRoot'/Natural" integerSquareRoot' = fromInteger . isqrtInteger . toInteger
  #-}
{-# INLINE [1] integerSquareRoot' #-}
integerSquareRoot' :: Integral a => a -> a
integerSquareRoot' = isqrtA

-- | For a non-negative input \( n \)
--   calculate its integer square root \( r = \lfloor \sqrt{n} \rfloor \)
--   and remainder \( s = n - r^2 \).
--   Throw an error on negative input.
--
-- >>> integerSquareRootRem 99
-- (9,18)
-- >>> integerSquareRootRem 100
-- (10,0)
-- >>> integerSquareRootRem 101
-- (10,1)
{-# SPECIALISE integerSquareRootRem :: Int -> (Int, Int) #-}
{-# SPECIALISE integerSquareRootRem :: Word -> (Word, Word) #-}
{-# SPECIALISE integerSquareRootRem :: Integer -> (Integer, Integer) #-}
{-# SPECIALISE integerSquareRootRem :: Natural -> (Natural, Natural) #-}
integerSquareRootRem :: Integral a => a -> (a, a)
integerSquareRootRem n
  | n < 0       = error "integerSquareRootRem: negative argument"
  | otherwise   = integerSquareRootRem' n

-- | Calculate the integer square root of a non-negative number as well as
--   the difference of that number with the square of that root, that is if
--   @(s,r) = integerSquareRootRem' n@ then @s^2 <= n == s^2+r < (s+1)^2@.
--   The precondition @n >= 0@ is not checked.
{-# RULES
"integerSquareRootRem'/Integer" integerSquareRootRem' = karatsubaSqrt
  #-}
{-# INLINE [1] integerSquareRootRem' #-}
integerSquareRootRem' :: Integral a => a -> (a, a)
integerSquareRootRem' n = (s, n - s * s)
  where
    s = integerSquareRoot' n

-- | Calculate the exact integer square root if it exists,
-- otherwise return 'Nothing'.
--
-- >>> map exactSquareRoot [-100, 99, 100, 101]
-- [Nothing,Nothing,Just 10,Nothing]
{-# SPECIALISE exactSquareRoot :: Int -> Maybe Int #-}
{-# SPECIALISE exactSquareRoot :: Word -> Maybe Word #-}
{-# SPECIALISE exactSquareRoot :: Integer -> Maybe Integer #-}
{-# SPECIALISE exactSquareRoot :: Natural -> Maybe Natural #-}
exactSquareRoot :: Integral a => a -> Maybe a
exactSquareRoot n
  | n >= 0
  , isPossibleSquare n
  , (r, 0) <- integerSquareRootRem' n = Just r
  | otherwise                         = Nothing

-- | Test whether the argument is a perfect square.
--
-- >>> map isSquare [-100, 99, 100, 101]
-- [False,False,True,False]
{-# SPECIALISE isSquare :: Int -> Bool #-}
{-# SPECIALISE isSquare :: Word -> Bool #-}
{-# SPECIALISE isSquare :: Integer -> Bool #-}
{-# SPECIALISE isSquare :: Natural -> Bool #-}
isSquare :: Integral a => a -> Bool
isSquare n = n >= 0 && isSquare' n

-- | Test whether the input (a non-negative number) @n@ is a square.
--   The same as 'isSquare', but without the negativity test,
--   so marginally faster.
--
--   The precondition @n >= 0@ is not tested, passing negative
--   arguments may cause any kind of havoc.
{-# SPECIALISE isSquare' :: Int -> Bool #-}
{-# SPECIALISE isSquare' :: Word -> Bool #-}
{-# SPECIALISE isSquare' :: Integer -> Bool #-}
{-# SPECIALISE isSquare' :: Natural -> Bool #-}
isSquare' :: Integral a => a -> Bool
isSquare' n
    | isPossibleSquare n
    , (_, 0) <- integerSquareRootRem' n = True
    | otherwise                         = False

-- | Test whether a non-negative number may be a square.
--   Non-negativity is not checked, passing negative arguments may
--   cause any kind of havoc.
--
--   First the remainder modulo 256 is checked (that can be calculated
--   easily without division and eliminates about 82% of all numbers).
--   After that, the remainders modulo 9, 25, 7, 11 and 13 are tested
--   to eliminate altogether about 99.436% of all numbers.
{-# SPECIALISE isPossibleSquare :: Int -> Bool #-}
{-# SPECIALISE isPossibleSquare :: Word -> Bool #-}
{-# SPECIALISE isPossibleSquare :: Integer -> Bool #-}
{-# SPECIALISE isPossibleSquare :: Natural -> Bool #-}
isPossibleSquare :: Integral a => a -> Bool
isPossibleSquare n'
  =  indexBitSet mask256 (fromInteger (n .&. 255))
  && indexBitSet mask693 (fromInteger (n `rem` 693))
  && indexBitSet mask325 (fromInteger (n `rem` 325))
  where
    n = toInteger n'

-----------------------------------------------------------------------------
-- Generated by 'Math.NumberTheory.Utils.BitMask.vectorToAddrLiteral'

mask256 :: Ptr Word
mask256 = Ptr "\DC3\STX\ETX\STX\DC2\STX\STX\STX\DC3\STX\STX\STX\DC2\STX\STX\STX\DC2\STX\ETX\STX\DC2\STX\STX\STX\DC2\STX\STX\STX\DC2\STX\STX\STX"#

mask693 :: Ptr Word
mask693 = Ptr "\DC3\STXA\STX0\NUL\STX\EOTI\NUL\STX\t\CAN\NUL\NULB\164\NUL\DC1\EOT\b\STX\NUL@P\128@\NUL\STX\t\128 \SOH\DLE\NUL\SOH\130$\NUL\128\DC4(\NUL\NUL\SOH\DC2\NUL\f\STX\DC4\SOH\NUL \b\NUL\"\NUL\128\EOT`\144\NUL\b\129\NULE\DC2\DLE@\STX\EOT\NUL\129\NUL\t\b\EOT\SOH\194\128\NUL\DLE\EOT\NUL\DLE\NUL\NUL"#

mask325 :: Ptr Word
mask325 = Ptr "\DC3B\SOH&\144\NUL\n!%\140\STXH0\SOH\DC4BJ\b\ENQ\144@\STX(\132\148\DLE\n \131\EOTP\f)!\DC4@\STX\EM\160\DLE\DC2"#

-- -- Make an array indicating whether a remainder is a square remainder.
-- sqRemArray :: Int -> V.Vector Bool
-- sqRemArray md = runST $ do
--   ar <- MV.replicate md False
--   let !stop = (md `quot` 2) + 1
--       fill k
--         | k < stop  = MV.unsafeWrite ar ((k*k) `rem` md) True >> fill (k+1)
--         | otherwise = return ()
--   MV.unsafeWrite ar 0 True
--   MV.unsafeWrite ar 1 True
--   fill 2
--   V.unsafeFreeze ar

-- sr256 :: V.Vector Bool
-- sr256 = sqRemArray 256

-- sr693 :: V.Vector Bool
-- sr693 = sqRemArray 693

-- sr325 :: V.Vector Bool
-- sr325 = sqRemArray 325

-----------------------------------------------------------------------------
-- Specialisations for Int, Word, and Integer

-- For @n <= 2^64@, the result of
--
-- > truncate (sqrt $ fromIntegral n)
--
-- is never too small and never more than one too large.
-- The multiplication doesn't overflow for 32 or 64 bit Ints.
isqrtInt' :: Int -> Int
isqrtInt' n
    | n < r*r   = r-1
    | otherwise = r
      where
        !r = (truncate :: Double -> Int) . sqrt $ fromIntegral n

isqrtInt64' :: Int64 -> Int64
isqrtInt64' n
    | n < r*r   = r-1
    | otherwise = r
      where
        !r = (truncate :: Double -> Int64) . sqrt $ fromIntegral n

-- Same for Word.
isqrtWord :: Word -> Word
isqrtWord n
    | n < (r*r)
      -- Double interprets values near maxBound as 2^64, we don't have that problem for 32 bits
      || finiteBitSize (0 :: Word) == 64 && r == 4294967296
                = r-1
    | otherwise = r
      where
        !r = (fromIntegral :: Int -> Word) . (truncate :: Double -> Int) . sqrt $ fromIntegral n

isqrtWord64 :: Word64 -> Word64
isqrtWord64 n
    | n < (r*r)
      -- Double interprets values near maxBound as 2^64
      || r == 4294967296
                = r-1
    | otherwise = r
      where
        !r = (fromIntegral :: Int64 -> Word64) . (truncate :: Double -> Int64) . sqrt $ fromIntegral n

{-# INLINE isqrtInteger #-}
isqrtInteger :: Integer -> Integer
isqrtInteger = fst . karatsubaSqrt
