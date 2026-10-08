{-# LANGUAGE ScopedTypeVariables #-}

module Real
  ( real
  ) where

import Test.QuickCheck.Checkers (EqProp, inverseL)
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, Arbitrary)

real :: forall a. (Arbitrary a, EqProp a, Show a, Fractional a, Real a) =>
        (a -> Rational) -> TestTree
real maxError = testGroup "Test Real instance" ts
  where ts = [ testProperty "fromRational toRational left inverse"
                            (inverseL fromRational (toRational :: a -> Rational))
             , testProperty "toRational fromRational left inverse (within error)"
                            (\r -> let x :: a
                                       x = fromRational r
                                       r' = toRational x
                                   in r - r' <= maxError x)
             ]
