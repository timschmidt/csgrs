-- |
-- Module:      Math.NumberTheory.Roots.CubesTests
-- Copyright:   (c) 2016 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Tests for Math.NumberTheory.Roots.Cubes
--

{-# OPTIONS_GHC -fno-warn-type-defaults #-}

module Math.NumberTheory.Roots.CubesTests
  ( testSuite
  ) where

import Data.Bits
import Test.Tasty
import Test.Tasty.HUnit

import Math.NumberTheory.Roots
import Math.NumberTheory.TestUtils

-- | Check that 'integerCubeRoot' returns the largest integer @m@ with @m^3 <= n@.
integerCubeRootProperty :: Integral a => AnySign a -> Bool
integerCubeRootProperty (AnySign n)
  = toInteger m ^ 3 <= toInteger n
  && toInteger n < (toInteger m + 1) ^ 3
  where
    m = integerCubeRoot n

-- | Specialized to trigger 'cubeRootInt''.
integerCubeRootProperty_Int :: AnySign Int -> Bool
integerCubeRootProperty_Int = integerCubeRootProperty

-- | Specialized to trigger 'cubeRootWord'.
integerCubeRootProperty_Word :: AnySign Word -> Bool
integerCubeRootProperty_Word = integerCubeRootProperty

-- | Specialized to trigger 'cubeRootIgr'.
integerCubeRootProperty_Integer :: AnySign Integer -> Bool
integerCubeRootProperty_Integer = integerCubeRootProperty

-- | Check that 'integerCubeRoot' returns the largest integer @m@ with @m^3 <= n@, where @n@ has form @k@^3-1.
integerCubeRootProperty2 :: Integral a => AnySign a -> Bool
integerCubeRootProperty2 (AnySign k)
  = k == 0
  || toInteger m ^ 3 <= toInteger n
  && toInteger n < (toInteger m + 1) ^ 3
  where
    n = k ^ 3 - 1
    m = integerCubeRoot n

-- | Specialized to trigger 'cubeRootInt''.
integerCubeRootProperty2_Int :: AnySign Int -> Bool
integerCubeRootProperty2_Int = integerCubeRootProperty2

-- | Specialized to trigger 'cubeRootWord'.
integerCubeRootProperty2_Word :: AnySign Word -> Bool
integerCubeRootProperty2_Word = integerCubeRootProperty2

-- | Check that 'integerCubeRoot' of 2^63-1 is 2^21-1, not 2^21.
integerCubeRootSpecialCase1_Int :: Assertion
integerCubeRootSpecialCase1_Int =
  assertEqual "integerCubeRoot" (integerCubeRoot (maxBound :: Int)) (2 ^ 21 - 1)

-- | Check that 'integerCubeRoot' of 2^63-1 is 2^21-1, not 2^21.
integerCubeRootSpecialCase1_Word :: Assertion
integerCubeRootSpecialCase1_Word =
  assertEqual "integerCubeRoot" (integerCubeRoot (maxBound `div` 2 :: Word)) (2 ^ 21 - 1)

-- | Check that 'integerCubeRoot' of 2^64-1 is 2642245.
integerCubeRootSpecialCase2 :: Assertion
integerCubeRootSpecialCase2 =
  assertEqual "integerCubeRoot" (integerCubeRoot (maxBound :: Word)) 2642245

-- | Check that the number 'isCube' iff its 'integerCubeRoot' is exact.
isCubeProperty :: Integral a => AnySign a -> Bool
isCubeProperty (AnySign n) = (n /= m ^ 3 && not t) || (n == m ^ 3 && t)
  where
    t = isCube n
    m = integerCubeRoot n

-- | Check that 'exactCubeRoot' returns an exact integer cubic root
-- and is consistent with 'isCube'.
exactCubeRootProperty :: Integral a => AnySign a -> Bool
exactCubeRootProperty (AnySign n) = case exactCubeRoot n of
  Nothing -> not (isCube n)
  Just m  -> isCube n && n == m ^ 3

testSuite :: TestTree
testSuite = testGroup "Cubes"
  [ testGroup "integerCubeRoot" $
    [ testIntegralProperty "generic"         integerCubeRootProperty
    , testSmallAndQuick    "generic Int"     integerCubeRootProperty_Int
    , testSmallAndQuick    "generic Word"    integerCubeRootProperty_Word
    , testSmallAndQuick    "generic Integer" integerCubeRootProperty_Integer

    , testIntegralProperty "almost cube"      integerCubeRootProperty2
    , testSmallAndQuick    "almost cube Int"  integerCubeRootProperty2_Int
    , testSmallAndQuick    "almost cube Word" integerCubeRootProperty2_Word
    ] ++ if finiteBitSize (0 :: Word) /= 64 then [] else
    [ testCase             "maxBound :: Int"      integerCubeRootSpecialCase1_Int
    , testCase             "maxBound / 2 :: Word" integerCubeRootSpecialCase1_Word
    , testCase             "maxBound :: Word"     integerCubeRootSpecialCase2
    ]
  , testIntegralProperty "isCube"           isCubeProperty
  , testIntegralProperty "exactCubeRoot"    exactCubeRootProperty
  ]
