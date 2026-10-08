{-# LANGUAGE NoMonomorphismRestriction #-}
{-# LANGUAGE ScopedTypeVariables #-}

module Floating
  ( floating
  ) where

import Fractional (fractional)
import System.Random (Random)
import Test.QuickCheck.Checkers (EqProp, (=-=), inverseL)
import Test.QuickCheck.Extra (UnitInterval(..), Tiny(..), BiunitInterval)
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, NonNegative(..), Positive(..), Arbitrary, (==>))
import Test.Tasty.HUnit (testCase, (@?=))

floating :: forall a. (Arbitrary a, EqProp a, Show a, Floating a, Ord a, Random a) =>
            a -> TestTree
floating _ = testGroup "Test Floating instance" ts
  where e = exp 1
        ts = [ fractional (undefined :: a)
             , testCase "π/4 = atan 1" ((pi::a) @?= 4 * atan 1)
             , testProperty "log == logBase e"
                            (log =-= logBase (e :: Positive a))
             , testProperty "exp == (e **)" (exp =-= ((e::a) **))
             , testProperty "sqrt x * sqrt x = x"
                            (\(NonNegative (x :: a)) -> let r = sqrt x
                                                        in r * r == x)
             , testProperty "law of exponents"
                            (\(Positive (base :: a)) x y ->
                              base ** (x + y) =-= base ** x * base ** y)
             , testProperty "logarithm definition"
                            (\(Positive (b :: a)) (Tiny c) ->
                              let x = b ** c
                              in b /= 1 ==> c =-= logBase b x)
             , testProperty "sine cosine definition"
                            (\x (y :: a) ->
                              cos (x - y) =-= cos x * cos y + sin x * sin y)
               -- TODO: Use open interval
             , testProperty "0 < x cos x"
                            (\(x::UnitInterval a) -> 0 <= x * cos x)
               -- Use <= here because of precision issues :(
             , testProperty "x cos x < sin x"
                            (\(x::UnitInterval a) -> x * cos x <= sin x)
             , testProperty "sin x < x" (\(x::UnitInterval a) -> sin x <= x)
             , testProperty "tangent definition"
                            (\(x::a) -> cos x /= 0 ==> tan x =-= sin x / cos x)
             , testProperty "asin left inverse"
                            (inverseL sin (asin :: BiunitInterval a -> BiunitInterval a))
             , testProperty "acos left inverse"
                            (inverseL cos (acos :: BiunitInterval a -> BiunitInterval a))
             , testProperty "atan left inverse" (inverseL tan (atan :: a -> a))
             , testProperty "sinh definition"
                            (\(x::a) -> sinh x =-= (exp x - exp (-x)) / 2)
             , testProperty "cosh definition"
                            (\(x::a) -> cosh x =-= (exp x + exp (-x)) / 2)
             , testProperty "tanh definition"
                            (\(x::a) -> tanh x =-= sinh x / cosh x)
             , testProperty "sinh left inverse"
                            (inverseL asinh (sinh :: a -> a))
             , testProperty "cosh left inverse"
                            (acosh . cosh =-= (abs :: a -> a))
             , testProperty "tanh left inverse"
                            (inverseL atanh (tanh :: Tiny a -> Tiny a))
             ]


