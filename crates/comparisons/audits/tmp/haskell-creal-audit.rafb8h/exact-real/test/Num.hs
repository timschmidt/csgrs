{-# LANGUAGE ScopedTypeVariables #-}

module Num
  ( num
  , numAuxTests
  ) where

import Test.QuickCheck (Arbitrary)
import Test.QuickCheck.Checkers (idempotent, EqProp, (=-=))
import Test.QuickCheck.Classes.Extra (commutativeRing)
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty)

num :: forall a. (Arbitrary a, EqProp a, Show a, Num a) => a -> TestTree
num _ = testGroup "Test Num instance" ts
  where
    ts = [commutativeRing "commutativeRing" (undefined :: a), numAuxTests (undefined :: a)]

numAuxTests :: forall a. (Arbitrary a, EqProp a, Show a, Num a) => a -> TestTree
numAuxTests _ = testGroup "Num instance aux tests" ts
  where
    ts = [ testProperty "x + negate y = x - y" (\x (y :: a) -> x + negate y =-= x - y)
         , testProperty "abs is idempotent" (idempotent (abs :: a -> a))
         , testProperty "abs-signum law" (\(x :: a) -> abs x * signum x =-= x)
         ]

