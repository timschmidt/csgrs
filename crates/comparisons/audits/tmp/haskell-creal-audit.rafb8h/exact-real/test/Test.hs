{-# LANGUAGE DataKinds           #-}
{-# LANGUAGE FlexibleContexts    #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE TypeApplications    #-}
{-# OPTIONS_GHC -fno-warn-orphans #-}

module Main (main) where

import           Data.CReal.Converge
import           Data.CReal.Extra      ()
import           Data.CReal.Internal
import           Data.List             (inits)
import           Data.Maybe            (fromJust)
import           Data.Proxy
import           Data.Ratio            ((%))
import           GHC.TypeNats
import           Numeric.Natural
import           Test.Tasty            (TestTree, defaultMain, testGroup)
import           Test.Tasty.HUnit      (Assertion, testCase, (@=?))
import           Test.Tasty.QuickCheck (Positive (..), Property, testProperty,
                                        (.&&.), (===), (==>))

import           BoundedFunctions      (boundedFunctions)
import           Floating              (floating)
import           Ord                   (ord)
import           Random                (random)
import           Read                  (read')
import           Real                  (real)
import           RealFloat             (realFloat)
import           RealFrac              (realFrac)

{-# ANN test_floating "HLint: ignore Use camelCase" #-}
test_floating :: forall p proxy. KnownNat p => proxy p -> TestTree
test_floating _ = floating (undefined :: CReal p)

{-# ANN test_ord "HLint: ignore Use camelCase" #-}
test_ord :: forall p proxy. KnownNat p => proxy p -> TestTree
test_ord _ = ord (undefined :: CReal p)

{-# ANN test_real "HLint: ignore Use camelCase" #-}
test_real :: forall p proxy . KnownNat p => proxy p -> TestTree
test_real _ =
  real (\x -> 1 % toInteger (max 1 (crealPrecision (x :: CReal p))))

{-# ANN test_realFrac "HLint: ignore Use camelCase" #-}
test_realFrac :: forall p proxy. KnownNat p => proxy p -> TestTree
test_realFrac _ = realFrac (undefined :: CReal p)

{-# ANN test_realFloat "HLint: ignore Use camelCase" #-}
test_realFloat :: forall p proxy. KnownNat p => proxy p -> TestTree
test_realFloat _ = realFloat (undefined :: CReal p)

{-# ANN test_read "HLint: ignore Use camelCase" #-}
test_read :: forall p proxy. KnownNat p => proxy p -> TestTree
test_read _ = read' (undefined :: CReal p)

{-# ANN test_random "HLint: ignore Use camelCase" #-}
test_random :: forall p proxy. KnownNat p => proxy p -> TestTree
test_random _ = random (undefined :: CReal p)

prop_decimalDigits :: Positive Int -> Property
prop_decimalDigits (Positive p) = let d = decimalDigitsAtPrecision p
                                  in 10^d >= (2^p :: Integer) .&&.
                                     (d > 0 ==> 10^(d-1) < (2^p :: Integer))

prop_showIntegral :: Integer -> Property
prop_showIntegral i = show i === show (fromInteger i :: CReal 0)

prop_shiftL :: forall p . KnownNat p => CReal p -> Int -> Property
prop_shiftL x s = x `shiftL` s === x * 2 ** fromIntegral s

prop_shiftR :: forall p . KnownNat p => CReal p -> Int -> Property
prop_shiftR x s = x `shiftR` s === x / 2 ** fromIntegral s

prop_showNumDigits :: Positive Int -> Rational -> Property
prop_showNumDigits (Positive places) x =
  let s = rationalToDecimal places x
  in length (dropWhile (/= '.') s) === places + 1

--
-- Testing Data.CReal.Converge
--

case_convergeErrEmptyCReal :: Assertion
case_convergeErrEmptyCReal = convergeErr undefined [] @=? (Nothing :: Maybe (CReal 0))

case_convergeErrEmptyUnit :: Assertion
case_convergeErrEmptyUnit = convergeErr undefined [] @=? (Nothing :: Maybe ())

case_convergeEmptyCReal :: Assertion
case_convergeEmptyCReal = converge [] @=? (Nothing :: Maybe (CReal 0))

case_convergeEmptyUnit :: Assertion
case_convergeEmptyUnit = converge [] @=? (Nothing :: Maybe ())

prop_convergeCollatzInteger :: Positive Integer -> Property
prop_convergeCollatzInteger (Positive x) = converge (iterate collatz x) === Just 1
  where collatz :: Integer -> Integer
        collatz c | c == 1 = 1
                  | even c = c `div` 2
                  | otherwise = c * 3 + 1


case_convergePointNineRecurringCReal
  :: forall p proxy . KnownNat p => proxy p -> Assertion
case_convergePointNineRecurringCReal _ = (Just 1 :: Maybe (CReal p)) @=?
                                         converge (read <$> pointNineRecurring)
  where pointNineRecurring = ("0.9" ++) <$> inits (repeat '9')

prop_convergeErrSqrtCReal
  :: forall p . KnownNat p => Positive (CReal p) -> Property
prop_convergeErrSqrtCReal (Positive x) = sqrt' (x ^ (2::Int)) === x
  where sqrt' x' = let initialGuess = x'
                       improve y = (y + x' / y) / 2
                       err y = abs (x' - y * y)
                   in fromJust $ convergeErr err (tail $ iterate improve initialGuess)

-- Test that the behavior when error is too small is correct
prop_convergeErrSmallSqrtCReal
  :: forall p . KnownNat p => Positive (CReal p) -> Property
prop_convergeErrSmallSqrtCReal (Positive x) = sqrt' (x ^ (2::Int)) === x
  where sqrt' x' = let initialGuess = x'
                       improve y = (y + x' / y) / 2
                       err y = abs (x' - y * y) / 128
                   in fromJust $ convergeErr err (tail $ iterate improve initialGuess)

prop_convergeErrSqrtInteger :: Positive Integer -> Property
prop_convergeErrSqrtInteger (Positive x) = sqrt' (x ^ (2::Int)) === x
  where sqrt' x' = let initialGuess = x'
                       improve y = (y + x' `quot` y) `quot` 2
                       err y = abs (x' - y * y)
                   in fromJust $ convergeErr err (tail $ iterate improve initialGuess)


{-# ANN test_boundedFunctions "HLint: ignore Use camelCase" #-}
test_boundedFunctions :: forall p proxy. KnownNat p => proxy p -> TestTree
test_boundedFunctions _ = boundedFunctions (undefined :: CReal p)

prop_expPosNeg :: KnownNat p => CReal p -> Property
prop_expPosNeg x = expPosNeg x === (exp x, exp (-x))

prop_square :: KnownNat p => CReal p -> Property
prop_square x = square x === x * x

--
--
--

precisionTests :: Natural -> TestTree
precisionTests n = case someNatVal n of
  SomeNat (_ :: Proxy p) -> testGroup
    ("Precision Tests @" <> show n)
    [ test_floating (Proxy @p)
    , test_ord (Proxy @p)
    , test_real (Proxy @p)
    , test_realFrac (Proxy @p)
    , test_realFloat (Proxy @p)
    , test_read (Proxy @p)
    , test_random (Proxy @p)
    , testProperty "shiftL" (prop_shiftL @p)
    , testProperty "shiftR" (prop_shiftR @p)
    , testCase "convergePointNineRecurringCReal"
               (case_convergePointNineRecurringCReal (Proxy @p))
    , testProperty "convergeErrSqrtCReal" (prop_convergeErrSqrtCReal @p)
    , testProperty "convergeErrSmallSqrtCReal"
                   (prop_convergeErrSmallSqrtCReal @p)
    , test_boundedFunctions (Proxy @p)
    , testProperty "expPosNeg" (prop_expPosNeg @p)
    , testProperty "square"    (prop_square @p)
    ]

nonPrecisionTests :: TestTree
nonPrecisionTests = testGroup
  "Non precision Tests"
  [ testProperty "decimalDigits" prop_decimalDigits
  , testProperty "showIntegral"  prop_showIntegral
  , testProperty "showNumDigits" prop_showNumDigits
  , testCase "convergeErrEmptyCReal" case_convergeErrEmptyCReal
  , testCase "convergeErrEmptyUnit"  case_convergeErrEmptyUnit
  , testCase "convergeEmptyCReal"    case_convergeEmptyCReal
  , testCase "convergeEmptyUnit"     case_convergeEmptyUnit
  , testProperty "convergeCollatzInteger" prop_convergeCollatzInteger
  , testProperty "convergeErrSqrtInteger" prop_convergeErrSqrtInteger
  ]

main :: IO ()
main =
  let precisions = [0, 1, 2, 10, 30]
  in  defaultMain
        (testGroup "Main" (nonPrecisionTests : (precisionTests <$> precisions)))
