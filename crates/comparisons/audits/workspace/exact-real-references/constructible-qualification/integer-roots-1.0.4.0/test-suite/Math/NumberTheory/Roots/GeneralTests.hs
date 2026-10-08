-- |
-- Module:      Math.NumberTheory.Roots.GeneralTests
-- Copyright:   (c) 2016 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Tests for Math.NumberTheory.Roots.General
--

{-# OPTIONS_GHC -fno-warn-type-defaults #-}

module Math.NumberTheory.Roots.GeneralTests
  ( testSuite
  ) where

import Data.Bits
import Test.Tasty
import Test.Tasty.HUnit
import Test.Tasty.QuickCheck as QC
import Numeric.Natural

import Math.NumberTheory.Roots
import Math.NumberTheory.TestUtils

-- | Check that 'integerRoot' @pow@ returns the largest integer @m@ with @m^pow <= n@.
integerRootProperty :: (Integral a, Integral b) => AnySign a -> Power b -> Bool
integerRootProperty (AnySign n) (Power pow)
  =  (even pow && n < 0)
  || (pow == 1 && root == n)
  || (toInteger root ^ pow <= toInteger n && toInteger n < toInteger (root + 1) ^ pow)
  where
    root = integerRoot pow n

integerRootHugeProperty :: Huge Natural -> Large Word -> Bool
integerRootHugeProperty (Huge n) (Large pow') = pow == 0 ||
  toInteger root ^ pow <= toInteger n && toInteger n < toInteger (root + 1) ^ pow
    where
      pow = pow' `mod` 2000
      root = integerRoot pow n

-- | Check that the number 'isKthPower' iff its 'integerRoot' is exact.
isKthPowerProperty :: (Integral a, Integral b) => AnySign a -> Power b -> Bool
isKthPowerProperty (AnySign n) (Power pow) = (even pow && n < 0 && not t) || (n /= root ^ pow && not t) || (n == root ^ pow && t)
  where
    t = isKthPower pow n
    root = integerRoot pow n

-- | Check that 'exactRoot' returns an exact integer root
-- and is consistent with 'isKthPower'.
exactRootProperty :: (Integral a, Integral b) => AnySign a -> Power b -> Bool
exactRootProperty (AnySign n) (Power pow) = case exactRoot pow n of
  Nothing   -> not (isKthPower pow n)
  Just root -> isKthPower pow n && n == root ^ pow

-- | Check that 'isPerfectPower' is consistent with 'highestPower'.
isPerfectPowerProperty :: Integral a => AnySign a -> Bool
isPerfectPowerProperty (AnySign n) = (k > 1 && t) || (k == 1 && not t)
  where
    t = isPerfectPower n
    (_, k) = highestPower n

-- | Check that the first component of 'highestPower' is square-free.
highestPowerProperty :: Integral a => AnySign a -> Bool
highestPowerProperty (AnySign n) = (n + 1 `elem` [0, 1, 2] && k == 3) || (b ^ k == n && b' == b && k' == 1)
  where
    (b, k) = highestPower n
    (b', k') = highestPower b

highestPowerProperty2 :: Integral a => AnySign a -> Bool
highestPowerProperty2 (AnySign n) = case k of
  1 -> not (isSquare n) && not (isCube n)
  2 -> isSquare n && not (isCube n)
  3 -> n + 1 `elem` [0, 1, 2] || (not (isSquare n) && isCube n)
  4 -> isSquare n && not (isCube n)
  _ -> all (\l -> isKthPower l n == (k `mod` l == 0)) [1..k]
  where
    (_, k) = highestPower n

highestPowerSpecialCases :: [Assertion]
highestPowerSpecialCases =
  -- Freezes before d44a13b.
  [ a ( 1013582159576576
      , 1013582159576576
      , 1)
  -- Freezes before d44a13b.
  , a ( 1013582159576576 ^ 7
      , 1013582159576576
      , 7)

  , a ( 9 :: Int
      , 3
      , 2)

  , a ( minBound :: Int
      , -2 :: Int
      , fromIntegral (finiteBitSize (0 :: Int) - 1))

  , a ( (2 ^ 63 - 1) ^ 21
      , 2 ^ 63 - 1
      , 21)

  , a ( 576116746989720969230211509779286598589421531472851155101032940901763389787901933902294777750323196846498573545522289802689311975294763847414975335235584
      , 576116746989720969230211509779286598589421531472851155101032940901763389787901933902294777750323196846498573545522289802689311975294763847414975335235584
      , 1)

  , a ( -340282366920938463500268095579187314689
      , -340282366920938463500268095579187314689
      , 1)

  , a ( 268398749 :: Int
      , 268398749 :: Int
      , 1)

  , a ( 118372752099 :: Int
      , 118372752099 :: Int
      , 1)

  , a ( 1409777209 :: Int
      , 37547 :: Int
      , 2)

  , a ( -6277101735386680764856636523970481806547819498980467802113
      , -18446744073709551617
      , 3)

  , a ( -18446744073709551619 ^ 5
      , -18446744073709551619
      , 5)
  ]
  where
    a (n, b, k) = assertEqual "highestPower" (b, k) (highestPower n)

testSuite :: TestTree
testSuite = testGroup "General"
  [ testIntegralProperty  "integerRoot 1"  (`integerRootProperty` 1)
  , testIntegral2Property "integerRoot"    integerRootProperty
  , QC.testProperty "big integerRoot"    integerRootHugeProperty
  , testIntegral2Property "isKthPower"     isKthPowerProperty
  , testIntegral2Property "exactRoot"      exactRootProperty
  , testIntegralProperty  "isPerfectPower" isPerfectPowerProperty
  , testGroup "highestPower"
    ( testIntegralProperty  "highestPower"   highestPowerProperty
    : testIntegralProperty  "highestPower 2" highestPowerProperty2
    : zipWith (\i a -> testCase ("special case " ++ show i) a) [1..] highestPowerSpecialCases
    )
  ]
