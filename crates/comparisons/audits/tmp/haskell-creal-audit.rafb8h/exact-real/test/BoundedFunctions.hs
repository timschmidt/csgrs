{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE TypeFamilies #-}
module BoundedFunctions
  ( boundedFunctions
  ) where

import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck ((==>), testProperty)
import Test.QuickCheck.Extra (BiunitInterval(..), UnitInterval(..))
import Test.QuickCheck.Checkers ((=-=))
import GHC.TypeLits (KnownNat)

import Data.CReal.Internal
import Data.CReal.Extra ()

boundedFunctions :: forall a n. (KnownNat n, a ~ CReal n) =>  a -> TestTree
boundedFunctions _ = testGroup "bounded functions" ts
  where ts = [ testProperty "mulBounded"
                  (\(BiunitInterval x) (BiunitInterval y) ->
                    (x :: a) * y =-= x .*. y)
             , testProperty "mulBoundedL"
                  (\(BiunitInterval x) y ->
                    (x :: a) * y =-= x .* y)
             , testProperty "mulBoundedR"
                  (\x (BiunitInterval y) ->
                    (x :: a) * y =-= x *. y)
             , testProperty "recipBounded"
                  (\x -> (abs x >= 1) ==> recip (x::a) =-= recipBounded x)
             , testProperty "expBounded"
                  (\(BiunitInterval x) ->
                    exp (x :: a) =-= expBounded x)
             , testProperty "logBounded"
                  (\(UnitInterval x) ->
                    let x' = (x :: a) * (2 - 2/3) + 2/3
                    in log x' =-= logBounded x')
             , testProperty "sinBounded"
                  (\(BiunitInterval x) ->
                    sin (x :: a) =-= sinBounded x)
             , testProperty "cosBounded"
                  (\(BiunitInterval x) ->
                    cos (x :: a) =-= cosBounded x)
             , testProperty "atanBounded"
                  (\(BiunitInterval x) ->
                    atan (x :: a) =-= atanBounded x)
             ]

