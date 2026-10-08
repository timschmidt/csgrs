{-# LANGUAGE ScopedTypeVariables #-}

module Ord
  ( ord
  ) where

import Test.QuickCheck (Arbitrary)
import Test.QuickCheck.Checkers (EqProp)
import Test.QuickCheck.Classes.Extra (complement, strictTotalOrd)
import Test.Tasty.Extra (testGroup, TestTree)
import Test.Tasty.QuickCheck (Gen, arbitrary, testProperty, property)

ord :: forall a. (Arbitrary a, Show a, EqProp a, Ord a) => a -> TestTree
ord _ = testGroup "Test Ord instance" ts
  where
    gen :: a -> Gen a
    gen = const arbitrary
    ts = [ -- It's a bit of a pain, but basically the arbitrary instance could
           -- generate for precision p:
           --
           -- 2, 1, 3. Where 2 <= 1 and 2<=3 (because they compare equal at the low
           -- precision p) and hence 1 <= 3 (which is detected and fails the test).
           --
           -- testTreeFromNamedBatch "<= is a total ordering" (ordRel (<=) gen)
           -- testTreeFromNamedBatch ">= is a total ordering" (ordRel (>=) gen)
           strictTotalOrd "< is a strict total ordering" gen (<)
         , strictTotalOrd "< is a strict total ordering" gen (>)
         , complement "< is the complement of >=" gen (<) (>=)
         , complement "> is the complement of <=" gen (>) (<=)
         , testProperty "max x y >= x and y" (property $ \x y ->
                               let m = max x y :: a
                               in m >= x && m >= y)
         , testProperty "max x y == x or y" (property $ \x y ->
                               let m = max x y :: a
                               in m == x || m == y)
         , testProperty "min x y >= x and y" (property $ \x y ->
                               let m = min x y :: a
                               in m <= x && m <= y)
         , testProperty "min x y == x or y" (property $ \x y ->
                               let m = min x y :: a
                               in m == x || m == y)
         ]
