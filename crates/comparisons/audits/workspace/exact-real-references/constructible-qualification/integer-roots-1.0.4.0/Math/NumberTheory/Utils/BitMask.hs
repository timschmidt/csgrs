-- |
-- Module:      Math.NumberTheory.Utils.BitMask
-- Copyright:   (c) 2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Bit mask utilities.
--

{-# LANGUAGE CPP          #-}
{-# LANGUAGE MagicHash    #-}

module Math.NumberTheory.Utils.BitMask
  ( indexBitSet
  -- , vectorToAddrLiteral
  ) where

#include "MachDeps.h"

import Data.Bits (countTrailingZeros, finiteBitSize, testBit, (.&.))
import GHC.Exts (Int(..), Word(..), Ptr(..), iShiftRL#, indexWordOffAddr#)
#ifdef WORDS_BIGENDIAN
import GHC.Exts (byteSwap#)
#endif

-- import Data.Bits (unsafeShiftL)
-- import Data.List (unfoldr)
-- import Data.Char (chr)

indexBitSet :: Ptr Word -> Int -> Bool
indexBitSet (Ptr addr#) i@(I# i#) = word `testBit` (i .&. (fbs - 1))
  where
    fbs = finiteBitSize (0 :: Word)
    logFbs# = case countTrailingZeros fbs of
      I# l# -> l#
    word# = indexWordOffAddr# addr# (i# `iShiftRL#` logFbs#)
#ifdef WORDS_BIGENDIAN
    word = W# (byteSwap# word#)
#else
    word = W# word#
#endif

-- vectorToAddrLiteral :: [Bool] -> String
-- vectorToAddrLiteral = map (chr . toByte . padTo8) . unfoldr go
--   where
--     go :: [a] -> Maybe ([a], [a])
--     go [] = Nothing
--     go xs = Just (take 8 xs, drop 8 xs)

--     padTo8 :: [Bool] -> [Bool]
--     padTo8 xs
--       | length xs >= 8 = xs
--       | otherwise = xs ++ replicate (8 - length xs) False

--     toByte :: [Bool] -> Int
--     toByte xs = sum $ map (\i -> if xs !! i then 1 `unsafeShiftL` i else 0) [0..7]
