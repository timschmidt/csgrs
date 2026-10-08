-- |
-- Module:      Math.NumberTheory.Utils.FromIntegral
-- Copyright:   (c) 2017 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Monomorphic `fromIntegral`.
--

module Math.NumberTheory.Utils.FromIntegral
  ( wordToInt
  , wordToInteger
  , intToWord
  , intToInteger
  , naturalToInteger
  , integerToNatural
  , integerToWord
  , integerToInt
  ) where

import Numeric.Natural (Natural)

wordToInt :: Word -> Int
wordToInt = fromIntegral
{-# INLINE wordToInt #-}

wordToInteger :: Word -> Integer
wordToInteger = fromIntegral
{-# INLINE wordToInteger #-}

intToWord :: Int -> Word
intToWord = fromIntegral
{-# INLINE intToWord #-}

intToInteger :: Int -> Integer
intToInteger = fromIntegral
{-# INLINE intToInteger #-}

naturalToInteger :: Natural -> Integer
naturalToInteger = fromIntegral
{-# INLINE naturalToInteger #-}

integerToNatural :: Integer -> Natural
integerToNatural = fromIntegral
{-# INLINE integerToNatural #-}

integerToWord :: Integer -> Word
integerToWord = fromIntegral
{-# INLINE integerToWord #-}

integerToInt :: Integer -> Int
integerToInt = fromIntegral
{-# INLINE integerToInt #-}
