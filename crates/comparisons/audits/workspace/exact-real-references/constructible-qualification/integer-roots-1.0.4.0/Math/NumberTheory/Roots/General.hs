-- |
-- Module:      Math.NumberTheory.Roots.General
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Daniel Fischer <daniel.is.fischer@googlemail.com>
--
-- Calculating integer roots and determining perfect powers.
-- The algorithms are moderately efficient.
--

{-# LANGUAGE BangPatterns  #-}
{-# LANGUAGE CPP           #-}
{-# LANGUAGE MagicHash     #-}
{-# LANGUAGE ViewPatterns  #-}
{- HLINT ignore "Use list comprehension" -}
{- HLINT ignore "Use fewer imports" -}

module Math.NumberTheory.Roots.General
    ( integerRoot
    , exactRoot
    , isKthPower
    , isPerfectPower
    , highestPower
    ) where

#include "MachDeps.h"

import Prelude hiding (Foldable(..))
import Data.Bits (countTrailingZeros, shiftL, shiftR)
import Data.Foldable (Foldable(..))
import Data.List (sortBy)
import Data.Maybe (isJust)
import GHC.Exts (Int(..), Word(..), word2Int#, int2Double#, double2Int#, isTrue#, Ptr(..), indexWord16OffAddr#, (/##), (**##))
#if MIN_VERSION_base(4,16,0)
import GHC.Exts (word16ToWord#)
#endif
#ifdef WORDS_BIGENDIAN
import GHC.Exts (byteSwap16#)
#endif
import Numeric.Natural (Natural)

#ifdef MIN_VERSION_integer_gmp
import GHC.Exts (int2Word#, quotInt#, (<#), (*#), (-#), (+#))
import GHC.Integer.GMP.Internals (Integer(..), shiftLInteger, shiftRInteger)
import GHC.Integer.Logarithms (integerLog2#)
#define IS S#
#else
import GHC.Exts (plusWord#, minusWord#, timesWord#, quotWord#, ltWord#)
import GHC.Num.Integer (Integer(..), integerLog2#, integerShiftR#, integerShiftL#)
#endif

import qualified Math.NumberTheory.Roots.Squares as P2
import qualified Math.NumberTheory.Roots.Cubes as P3
import qualified Math.NumberTheory.Roots.Fourth as P4
import Math.NumberTheory.Primes.Small
import Math.NumberTheory.Utils.FromIntegral (wordToInt)

-- | For a positive power \( k \) and
-- a given \( n \)
-- return the integer \( k \)-th root \( \lfloor \sqrt[k]{n} \rfloor \).
-- Throw an error if \( k \le 0 \) or if \( n \le 0 \) and \( k \) is even.
--
-- >>> integerRoot 6 65
-- 2
-- >>> integerRoot 5 243
-- 3
-- >>> integerRoot 4 624
-- 4
-- >>> integerRoot 3 (-124)
-- -5
-- >>> integerRoot 1 5
-- 5
{-# SPECIALISE integerRoot :: Int -> Int -> Int #-}
{-# SPECIALISE integerRoot :: Int -> Word -> Word #-}
{-# SPECIALISE integerRoot :: Int -> Integer -> Integer #-}
{-# SPECIALISE integerRoot :: Int -> Natural -> Natural #-}
{-# SPECIALISE integerRoot :: Word -> Int -> Int #-}
{-# SPECIALISE integerRoot :: Word -> Word -> Word #-}
{-# SPECIALISE integerRoot :: Word -> Integer -> Integer #-}
{-# SPECIALISE integerRoot :: Word -> Natural -> Natural #-}
{-# SPECIALISE integerRoot :: Integer -> Integer -> Integer #-}
{-# SPECIALISE integerRoot :: Natural -> Natural -> Natural #-}
integerRoot :: (Integral a, Integral b) => b -> a -> a
integerRoot 1 n         = n
integerRoot 2 n         = P2.integerSquareRoot n
integerRoot 3 n         = P3.integerCubeRoot n
integerRoot 4 n         = P4.integerFourthRoot n
integerRoot k n
  | k < 1             = error "integerRoot: negative exponent or exponent 0"
  | n < 0 && even k   = error "integerRoot: negative radicand for even exponent"
  | n < 0             =
    let r = negate . fromInteger . integerRoot k . negate $ fromIntegral n
    in if r^k == n then r else r - 1
  | n == 0            = 0
  | n < 31            = 1
  | kTooLarge         = 1
  | otherwise         = fromInteger $ newtonK (toInteger k) (toInteger n) a
    where
      a  = appKthRoot (fromIntegral k) (toInteger n)
      kTooLarge = (toInteger k /= toInteger (fromIntegral k `asTypeOf` n))    -- k doesn't fit in n's type
                  || (toInteger k > toInteger (maxBound :: Int))  -- 2^k doesn't fit in Integer
#ifdef MIN_VERSION_integer_gmp
                  || (I# (integerLog2# (toInteger n)) < fromIntegral k) -- n < 2^k
#else
                  || (W# (integerLog2# (toInteger n)) < fromIntegral k) -- n < 2^k
#endif

-- | For a positive exponent \( k \)
-- calculate the exact integer \( k \)-th root of the second argument if it exists,
-- otherwise return 'Nothing'.
--
-- >>> map (uncurry exactRoot) [(6, 65), (5, 243), (4, 624), (3, -124), (1, 5)]
-- [Nothing,Just 3,Nothing,Nothing,Just 5]
exactRoot :: (Integral a, Integral b) => b -> a -> Maybe a
exactRoot 1 n = Just n
exactRoot 2 n = P2.exactSquareRoot n
exactRoot 3 n = P3.exactCubeRoot n
exactRoot 4 n = P4.exactFourthRoot n
exactRoot k n
  | n == 1          = Just 1
  | k < 1           = Nothing
  | n < 0 && even k = Nothing
  | n < 0           = let m = negate n in
                      if m < 0
                        then fmap (fromInteger . negate) (exactRoot k (negate (toInteger n)))
                        else fmap negate (exactRoot k m)
  | n < 2           = Just n
  | n < 31          = Nothing
  | kTooLarge       = Nothing
  | otherwise       = case k `rem` 12 of
                        0 | c4 && c3 && ok -> Just r
                          | otherwise -> Nothing
                        2 | c2 && ok -> Just r
                          | otherwise -> Nothing
                        3 | c3 && ok -> Just r
                          | otherwise -> Nothing
                        4 | c4 && ok -> Just r
                          | otherwise -> Nothing
                        6 | c3 && c2 && ok -> Just r
                          | otherwise -> Nothing
                        8 | c4 && ok -> Just r
                          | otherwise -> Nothing
                        9 | c3 && ok -> Just r
                          | otherwise -> Nothing
                        10 | c2 && ok -> Just r
                           | otherwise -> Nothing
                        _ | ok -> Just r
                          | otherwise -> Nothing

    where
      k' :: Int
      k' = fromIntegral k
      r  = integerRoot k' n
      c2 = P2.isPossibleSquare n
      c3 = P3.isPossibleCube n
      c4 = P4.isPossibleFourthPower n
      ok = r^k == n
      kTooLarge = (toInteger k /= toInteger (fromIntegral k `asTypeOf` n))    -- k doesn't fit in n's type
                  || (toInteger k > toInteger (maxBound :: Int))  -- 2^k doesn't fit in Integer
#ifdef MIN_VERSION_integer_gmp
                  || (I# (integerLog2# (toInteger n)) < fromIntegral k) -- n < 2^k
#else
                  || (W# (integerLog2# (toInteger n)) < fromIntegral k) -- n < 2^k
#endif

-- | For a positive exponent \( k \) test whether the second argument
-- is a perfect \( k \)-th power.
--
-- >>> map (uncurry isKthPower) [(6, 65), (5, 243), (4, 624), (3, -124), (1, 5)]
-- [False,True,False,False,True]
isKthPower :: (Integral a, Integral b) => b -> a -> Bool
isKthPower k n = isJust (exactRoot k n)

-- | Test whether the argument is a non-trivial perfect power
-- (e. g., square, cube, etc.).
--
-- >>> map isPerfectPower [0..10]
-- [True,True,False,False,True,False,False,False,True,True,False]
-- >>> map isPerfectPower [-10..0]
-- [False,False,True,False,False,False,False,False,False,True,True]
isPerfectPower :: Integral a => a -> Bool
isPerfectPower n
  | n == 0 || n == 1    = True
  | otherwise           = k > 1
    where
      (_,k) = highestPower n

-- | For \( n \not\in \{ -1, 0, 1 \} \)
-- find the largest exponent \( k \) for which
-- an exact integer \( k \)-th root \( r \) exists.
-- Return \( (r, k) \).
--
-- For \( n \in \{ -1, 0, 1 \} \) arbitrarily large exponents exist;
-- by arbitrary convention 'highestPower' returns \( (n, 3) \).
--
-- >>> map highestPower [0..10]
-- [(0,3),(1,3),(2,1),(3,1),(2,2),(5,1),(6,1),(7,1),(2,3),(3,2),(10,1)]
-- >>> map highestPower [-10..0]
-- [(-10,1),(-9,1),(-2,3),(-7,1),(-6,1),(-5,1),(-4,1),(-3,1),(-2,1),(-1,3),(0,3)]
highestPower :: Integral a => a -> (a, Word)
highestPower n'
  | abs n <= 1  = (n', 3)
  | n < 0       = case integerHighPower (negate n) of
                    (r,e) -> case countTrailingZeros e of
                               k -> (negate $ fromInteger (sqr k r), e `shiftR` k)
  | otherwise   = case integerHighPower n of
                    (r,e) -> (fromInteger r, e)
    where
      n :: Integer
      n = toInteger n'

      sqr :: Int -> Integer -> Integer
      sqr 0 m = m
      sqr k m = sqr (k-1) (m*m)

------------------------------------------------------------------------------------------
--                                  Auxiliary functions                                 --
------------------------------------------------------------------------------------------

newtonK :: Integer -> Integer -> Integer -> Integer
newtonK k n a = go (step a)
  where
    step m = ((k - 1) * m + n `quot` m ^ (k - 1)) `quot` k
    go m
      | l < m     = go l
      | otherwise = m
        where
          l = step m

-- find an approximation to the k-th root
-- here, k > 4 and n > 31
appKthRoot :: Int -> Integer -> Integer
appKthRoot (I# k#) (IS n#) = IS (double2Int# (int2Double# n# **## (1.0## /## int2Double# k#)))
#ifdef MIN_VERSION_integer_gmp
appKthRoot k@(I# k#) n
  | k >= 256 = 1 `shiftLInteger` (integerLog2# n `quotInt#` k# +# 1#)
  | otherwise =
    case integerLog2# n of
      l# -> case l# `quotInt#` k# of
              0# -> 1
              1# -> 3
              2# -> 5
              3# -> 11
              h# | isTrue# (h# <# 500#) ->
                   floor (scaleFloat (I# h#)
                          (fromInteger (n `shiftRInteger` (h# *# k#)) ** (1/fromIntegral k) :: Double))
                 | otherwise ->
                   floor (scaleFloat 400 (fromInteger (n `shiftRInteger` (h# *# k#)) ** (1/fromIntegral k) :: Double))
                          `shiftLInteger` (h# -# 400#)
#else
appKthRoot k@(fromIntegral -> W# k#) n
  | k >= 256 = 1 `integerShiftL#` (integerLog2# n `quotWord#` k# `plusWord#` 1##)
  | otherwise =
    case integerLog2# n of
      l# -> case l# `quotWord#` k# of
              0## -> 1
              1## -> 3
              2## -> 5
              3## -> 11
              h# | isTrue# (h# `ltWord#` 500##) ->
                   floor (scaleFloat (I# (word2Int# h#))
                          (fromInteger (n `integerShiftR#` (h# `timesWord#` k#)) ** (1/fromIntegral k) :: Double))
                 | otherwise ->
                   floor (scaleFloat 400 (fromInteger (n `integerShiftR#` (h# `timesWord#` k#)) ** (1/fromIntegral k) :: Double))
                          `integerShiftL#` (h# `minusWord#` 400##)
#endif

-- assumption: argument is > 1
integerHighPower :: Integer -> (Integer, Word)
integerHighPower n
  | n < 4       = (n,1)
  | otherwise   = case splitOff 2 n of
                    (e2,m) | m == 1     -> (2,e2)
                           | otherwise  -> findHighPower e2 (if e2 == 0 then [] else [(2,e2)]) m r smallOddPrimes
                             where
                               r = P2.integerSquareRoot m

findHighPower :: Word -> [(Integer, Word)] -> Integer -> Integer -> [Integer] -> (Integer, Word)
findHighPower 1 pws m _ _ = (foldl' (*) m [p^e | (p,e) <- pws], 1)
findHighPower e pws 1 _ _ = (foldl' (*) 1 [p^(ex `quot` e) | (p,ex) <- pws], e)
findHighPower e pws m s (p:ps)
  | s < p       = findHighPower 1 pws m s []
  | otherwise   =
    case splitOff p m of
      (0,_) -> findHighPower e pws m s ps
      (k,r) -> findHighPower (gcd k e) ((p,k):pws) r (P2.integerSquareRoot r) ps
findHighPower e pws m _ [] = finishPower e pws m

splitOff :: Integer -> Integer -> (Word, Integer)
splitOff !_ 0 = (0, 0) -- prevent infinite loop
splitOff p n = go 0 n
  where
    go !k m = case m `quotRem` p of
      (q, 0) -> go (k + 1) q
      _      -> (k, m)
{-# INLINABLE splitOff #-}

smallOddPrimes :: [Integer]
smallOddPrimes
  = takeWhile (< spBound)
  $ map (\(I# k#) -> IS (word2Int# (
#if MIN_VERSION_base(4,16,0)
#ifdef WORDS_BIGENDIAN
  byteSwap16# (word16ToWord# (indexWord16OffAddr# smallPrimesAddr# k#))
#else
  word16ToWord# (indexWord16OffAddr# smallPrimesAddr# k#)
#endif
#else
#ifdef WORDS_BIGENDIAN
  byteSwap16# (indexWord16OffAddr# smallPrimesAddr# k#)
#else
  indexWord16OffAddr# smallPrimesAddr# k#
#endif
#endif
  )))
    [1 .. smallPrimesLength - 1]
  where
    !(Ptr smallPrimesAddr#) = smallPrimesPtr

spBEx :: Word
spBEx = 14

spBound :: Integer
spBound = 2^spBEx

-- n large, has no prime divisors < spBound
finishPower :: Word -> [(Integer, Word)] -> Integer -> (Integer, Word)
finishPower e pws n
  | n < (1 `shiftL` wordToInt (2*spBEx))  = (foldl' (*) n [p^ex | (p,ex) <- pws], 1)    -- n is prime
  | e == 0  = rawPower maxExp n
  | otherwise = go divs
    where
#ifdef MIN_VERSION_integer_gmp
      maxExp = (W# (int2Word# (integerLog2# n))) `quot` spBEx
#else
      maxExp = W# (integerLog2# n) `quot` spBEx
#endif
      divs = divisorsTo maxExp e
      go [] = (foldl' (*) n [p^ex | (p,ex) <- pws], 1)
      go (d:ds) = case exactRoot d n of
                    Just r -> (foldl' (*) r [p^(ex `quot` d) | (p,ex) <- pws], d)
                    Nothing -> go ds

rawPower :: Word -> Integer -> (Integer, Word)
rawPower mx n
  | mx < 2      = (n,1)
  | mx == 2     = case P2.exactSquareRoot n of
                    Just r  -> (r,2)
                    Nothing -> (n,1)
rawPower mx n = case P4.exactFourthRoot n of
                  Just r -> case rawPower (mx `quot` 4) r of
                              (m,e) -> (m, 4*e)
                  Nothing -> case P2.exactSquareRoot n of
                               Just r -> case rawOddPower (mx `quot` 2) r of
                                           (m,e) -> (m, 2*e)
                               Nothing -> rawOddPower mx n

rawOddPower :: Word -> Integer -> (Integer, Word)
rawOddPower mx n
  | mx < 3       = (n,1)
rawOddPower mx n = case P3.exactCubeRoot n of
                     Just r -> case rawOddPower (mx `quot` 3) r of
                                 (m,e) -> (m, 3*e)
                     Nothing -> badPower mx n

badPower :: Word -> Integer -> (Integer, Word)
badPower mx n
  | mx < 5      = (n,1)
  | otherwise   = go 1 mx n (takeWhile (<= mx) $ scanl (+) 5 $ cycle [2,4])
    where
      go !e b m (k:ks)
        | b < k     = (m,e)
        | otherwise = case exactRoot k m of
                        Just r -> go (e*k) (b `quot` k) r (k:ks)
                        Nothing -> go e b m ks
      go e _ m []   = (m,e)

-- | List divisors of n, which are <= mx.
divisorsTo :: Word -> Word -> [Word]
divisorsTo mx n = sortBy (flip compare) $ go [1] n (2 : 3 : 5 : prs)
  where
    -- unP p m = (k, m / p ^ k), where k is as large as possible such that p ^ k still divides m
    unP :: Word -> Word -> (Word, Word)
    unP p m = goP 0 m
      where
        goP :: Word -> Word -> (Word, Word)
        goP !i j = case j `quotRem` p of
                     (q,r) | r == 0 -> goP (i+1) q
                           | otherwise -> (i,j)

    mset k = filter (<= mx) . map (* k)

    prs :: [Word]
    prs = 7 : filter prm (scanl (+) 11 $ cycle [2, 4, 2, 4, 6, 2, 6, 4])
    prm :: Word -> Bool
    prm k = td prs
      where
        td (p:ps) = (p*p > k) || (k `rem` p /= 0 && td ps)
        td []     = True

    go !st m (p:ps)
      | m == 1  = st
      | m < p*p = st ++ mset m st
      | otherwise =
        case unP p m of
          (0,_) -> go st m ps
          -- iterate f x = [x, f x, f (f x)...]
          (k,r) -> go (concat (take (wordToInt k + 1) (iterate (mset p) st))) r ps
    go st m [] = go st m [m+1]
