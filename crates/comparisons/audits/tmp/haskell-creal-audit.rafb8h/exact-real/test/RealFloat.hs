{-# LANGUAGE ScopedTypeVariables #-}

module RealFloat
  ( realFloat
  ) where

import Test.QuickCheck.Checkers (EqProp, (=-=), inverseL)
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, Arbitrary, (==>))

realFloat :: forall a. (Arbitrary a, EqProp a, Show a, RealFloat a) =>
            a -> TestTree
realFloat x = testGroup "Test RealFloat instance" ts
  where ts = [ decodeFloatLaws "decodeFloat laws" x
             , testProperty "encodeFloat decodeFloat left inverse"
                            (inverseL (uncurry encodeFloat) (decodeFloat :: a -> (Integer, Int)))
             , testProperty "scaleFloat definition"
                            (\y i -> let r = floatRadix y
                                     in scaleFloat i (y::a) =-= y * fromIntegral r ^^ i)
             , atan2Laws "atan2 laws" x
             , testProperty "exponent of 0 is 0" (exponent (0 :: a) =-= 0)
             , testProperty "exponent x = snd (decodeFloat x) + floatDigits x"
                            (\x -> exponent (x :: a) =-= snd (decodeFloat x) + floatDigits x)
             , testProperty "x = significand x * b ^^ exponent x, where b is the floating-point radix"
                            (\x -> (x :: a) =-= significand x * fromInteger (floatRadix x) ^^ exponent x)
             ]

decodeFloatLaws :: forall a. (Arbitrary a, EqProp a, Show a, RealFloat a) =>
                      String -> a -> TestTree
decodeFloatLaws s _ = testGroup s ts
  where ts = [ testProperty "x = m*b^^n"
                            (\x -> let (m, n) = decodeFloat (x :: a)
                                       b = floatRadix x
                                   in not (isNaN x || isInfinite x) ==>
                                      (x =-= fromInteger m * fromInteger b ^^ n))
             ]

atan2Laws :: forall a. (Arbitrary a, EqProp a, Show a, RealFloat a) =>
             String -> a -> TestTree
atan2Laws s _ = testGroup s ts
  where ts = [ testProperty "atan2 range" (\y x -> let θ = atan2 y (x :: a)
                                                   in abs θ <= pi)
             , testProperty "atan2 y 1 = atan y" (\y -> let θ = atan2 y (1 :: a)
                                                        in θ =-= atan y)
             ]

