{-# LANGUAGE ScopedTypeVariables #-}

module Fractional
  ( fractional
  ) where

import Data.Ratio ((%))
import Test.QuickCheck (Arbitrary)
import Test.QuickCheck.Checkers (EqProp, (=-=))
import Test.QuickCheck.Classes.Extra (field)
import Test.QuickCheck.Modifiers (NonZero(..))
import Test.QuickCheck.Extra ()
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty)
import Num (numAuxTests)

-- TODO: Reduce Ord to Eq on the new quickcheck release
-- TODO: Write a program to email me for todo's like that when the conditions
-- are met
fractional :: forall a. (Arbitrary a, EqProp a, Show a, Fractional a, Ord a) =>
              a -> TestTree
fractional _ = testGroup "Test Fractional instance" ts
  where
    ts = [ field "field" (undefined :: a)
         , numAuxTests (undefined :: a)
         , testProperty "x * recip y = x / y" (\x (NonZero (y :: a)) -> x * recip y =-= x / y)
         , testProperty "fromRational (x % y) = fromInteger x / fromInteger y"
             (\x (NonZero y) -> fromRational (x % y) =-= fromInteger x / (fromInteger y :: a))
         ]
