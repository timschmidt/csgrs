{-# LANGUAGE ScopedTypeVariables #-}

module RealFrac
  ( realFrac
  ) where

import Data.Function (on)
import Test.QuickCheck.Checkers (EqProp, (=-=))
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, Arbitrary)

-- TODO: Test the other functions
realFrac :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
            a -> TestTree
realFrac x = testGroup "Test RealFrac instance" ts
  where ts = [ properFractionLaws "properFraction laws" x
             , truncateLaws "truncate laws" x
             , roundLaws "round laws" x
             , ceilingLaws "ceiling laws" x
             , floorLaws "floor laws" x
             ]

-- Thses are used to cope with CReal 0 being a little weird with comparisons
-- between non-integers
infix 4 <., >.
(<.), (>.) :: (Ord a, Num a) => a -> a -> Bool
(<.) = (<) `on` (*2)
(>.) = (>) `on` (*2)

-- | This tests a slightly different law for n having the same sign as x
properFractionLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
                      String -> a -> TestTree
properFractionLaws s _ = testGroup s ts
  where ts = [ testProperty "x = n + f"
                            (\x -> let (n, f) = properFraction (x :: a)
                                   in x =-= fromInteger n + f)
             , testProperty "n has same sign or is zero"
                            (\x -> let (n, _) = properFraction (x :: a)
                                   in n == 0 || sign x == sign (n::Integer))
             , testProperty "abs f < 1"
                            (\x -> let (_::Int, f) = properFraction (x :: a)
                                   in abs f <. 1)
             , testProperty "f has same sign or is zero"
                            (\x -> let (_::Int, f) = properFraction (x :: a)
                                   in f == 0 || sign x == sign f)
             ]

truncateLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
                           String -> a -> TestTree
truncateLaws s _ = testGroup s ts
  where ts = [ testProperty "abs (truncate x) <= abs x"
                            (\x -> let t = truncate (x :: a)
                                   in fromInteger (abs t) <= abs x)
             , testProperty "abs (truncate x) + 1 > abs x"
                            (\x -> let t = truncate (x :: a)
                                   in fromInteger (abs t + 1) >. abs x)
             , testProperty "truncate x has same sign or is zero"
                            (\x -> let t = truncate (x :: a)
                                   in t == 0 || sign x == sign (t::Integer))
             ]

roundLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
                           String -> a -> TestTree
roundLaws s _ = testGroup s ts
  where ts = [ testProperty "abs (round x - x) <= 0.5"
                            (\x -> let r = round (x :: a)
                                       in abs (fromInteger r - x) <= 0.5)
             , testProperty "round to even if eqiudistant"
                            (\i -> let x = fromInteger i + 0.5 :: a
                                   in even (round x :: Integer))
             ]

ceilingLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
                           String -> a -> TestTree
ceilingLaws s _ = testGroup s ts
  where ts = [ testProperty "ceiling x - 1 < x"
                            (\x -> let c = ceiling (x :: a)
                                   in fromInteger c - 1 <. x)
             , testProperty "ceiling x >= x"
                            (\x -> let c = ceiling (x :: a)
                                   in fromInteger c >= x)
             ]

floorLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFrac a) =>
                           String -> a -> TestTree
floorLaws s _ = testGroup s ts
  where ts = [ testProperty "floor x + 1 > x"
                            (\x -> let f = floor (x :: a)
                                   in fromInteger f + 1 >. x)
             , testProperty "floor x <= x"
                            (\x -> let f = floor (x :: a)
                                   in fromInteger f <= x)
             ]

data Sign = Positive
          | Negative
  deriving (Eq, Show)

-- | Note that this returns Positive on zero rather than 0 like signum
sign :: (Ord a, Num a) => a -> Sign
sign x = if x < 0 then Negative
                  else Positive
