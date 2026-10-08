{-# LANGUAGE FlexibleInstances, MultiParamTypeClasses, StandaloneDeriving, DeriveGeneric #-}
import Test.Tasty
import Test.Tasty.SmallCheck as SC
import Test.Tasty.QuickCheck as QC
import Test.Tasty.HUnit
import Test.SmallCheck.Series as SC
import GHC.Generics

import Control.Applicative (ZipList (..))
import Control.Monad (liftM, liftM3)
import Data.Functor

import Data.List
import Data.Ord
import Data.Fixed (mod')
import Data.CDAR


-- Main test

main = defaultMain tests

tests :: TestTree
tests = testGroup "Tests" [dyadic, localOption (mkTimeout 4000000) approx, creal, unitTests]
  -- [dyadic
  -- ,approx
  -- ,creal
  -- ,unitTests]

-- General auxiliary properties.

idempotent :: Eq a => (a -> a) -> a -> Bool
idempotent f = \x -> f(x) == f(f(x))

autoinverse :: Eq a => (a -> a) -> a -> Bool
autoinverse f = \x -> x == f(f(x))

dyadic :: TestTree
dyadic = testGroup "Dyadic" [propertiesD]

deriving instance Generic Dyadic
instance (Monad m) => Serial m Dyadic

instance Arbitrary Dyadic where
    arbitrary = do a <- arbitrary
                   s <- choose (-1000,100)
                   return (a :^ s)

propertiesD :: TestTree
propertiesD = testGroup "Properties of Dyadic" [scPropD, qcPropD]

scPropD = testGroup "(checked by SmallCheck)"
  [ SC.testProperty "compare" $
      \x y -> compare (x :: Dyadic) (y :: Dyadic) == (compare (toRational x) (toRational y))
  , SC.testProperty "multiplication" $
      \a b -> toRational (a*b :: Dyadic) == (toRational a)*(toRational b)
  , SC.testProperty "addition" $
      \a b -> toRational (a+b :: Dyadic) == (toRational a)+(toRational b)
  , SC.testProperty "negate" $
      \a -> (a :: Dyadic) + negate a == 0
  , SC.testProperty "negation exists" $
      \a b -> (a :: Dyadic) + b == 0 SC.==> a == negate b
  , SC.testProperty "negation autoinverse" $ autoinverse (negate :: Dyadic -> Dyadic)
  , SC.testProperty "abs idempotent" $ idempotent (abs :: Dyadic -> Dyadic)
  , SC.testProperty "signum idempotent" $ idempotent (signum :: Dyadic -> Dyadic)
  , SC.testProperty "signum in {-1,0,1}" $ \x -> signum (x :: Dyadic) `elem` [-1,0,1]
  , SC.testProperty "negate abs signum" $
      \a -> (a :: Dyadic) + (negate (signum a) * (abs a)) == 0
  , SC.testProperty "read show" $
      \a -> (a :: Dyadic) == read (show a)
  , SC.testProperty "shift left identity" $ \d@(a :^ s) (SC.NonNegative n) -> shiftD (s - n) d == d
  , SC.testProperty "sqrt" $ \d@(a :^ s) -> sqrtD (min 0 (2*s)) (d*d) == abs d
  ]

qcPropD = testGroup "(checked by QuickCheck)"
  [ QC.testProperty "compare" $ 
      \x y -> compare (x :: Dyadic) (y :: Dyadic) == (compare (toRational x) (toRational y))
  , QC.testProperty "multiplication" $
      \a b -> toRational (a*b :: Dyadic) == (toRational a)*(toRational b)
  , QC.testProperty "addition" $
      \a b -> toRational (a+b :: Dyadic) == (toRational a)+(toRational b)
  , QC.testProperty "negate" $
      \a -> (a :: Dyadic) + negate a == 0
  , QC.testProperty "negate abs signum" $
      \a -> (a :: Dyadic) + (negate (signum a) * (abs a)) == 0:^0
  , QC.testProperty "read show" $
      \a -> (a :: Dyadic) == read (show a)
  , QC.testProperty "negation autoinverse" $ autoinverse (negate :: Dyadic -> Dyadic)
  , QC.testProperty "abs idempotent" $ idempotent (abs :: Dyadic -> Dyadic)
  , QC.testProperty "signum idempotent" $ idempotent (signum :: Dyadic -> Dyadic)
  , QC.testProperty "signum in {-1,0,1}" $ \x -> signum (x :: Dyadic) `elem` [-1,0,1]
  , QC.testProperty "shift left identity" $ \d@(a :^ s) (QC.NonNegative n) -> shiftD (s - n) d == d
  , QC.testProperty "sqrt" $ \d@(a :^ s) -> sqrtD (min 0 (2*s)) (d*d) == abs d
  ]

{-
Multiplication and addition of Approx is not tested. Those two operations should return the exact Approx of the result.

There are no test making sure that the loss of information is acceptable for reciprocal, exp, log, ... .
-}
approx :: TestTree
approx = testGroup "Approx" [scPropA, qcPropA]

instance Monad m => Serial m Approx where
  series = cons0 Bottom \/ (tripleToApprox <$> series)
    where
      tripleToApprox (m, SC.NonNegative e, s) = Approx m e s

instance Arbitrary Approx where
    arbitrary = frequency [(10,
                            liftM3 Approx
                                   arbitrary 
                                   (return 0)
                                   (choose (-100,20))),
                           (25,
                            liftM3 Approx
                                   arbitrary 
                                   (elements [1,2,3,254,255,256,257,510,511,512,513,767,768,1020,1021,1022,1023,1024,1025])
                                   (choose (-100,20))),
                           (60,
                            liftM3 Approx
                                   arbitrary 
                                   (choose (0,10))
                                   (choose (-100,20))),
                           (5, return Bottom)]

scPropA :: TestTree
scPropA = testGroup "(checked by smallcheck)"
  [ SC.testProperty "read . show = id" $ \a -> (a :: Approx) == read (show a)
  , SC.testProperty "diameter" $ \a -> diameter a == upperBound a - lowerBound a
  , SC.testProperty "fromDyadic exact and centre" $ \d -> let a = fromDyadic d in exact a && Just d == centre a
  , SC.testProperty "abs negate sign" $ \a -> approximatedBy 0 $ (a ::Approx) + (negate (signum a) * (abs a))
  , SC.testProperty "a+(-a) contains 0" $ \a -> 0 `approximatedBy` ((a :: Approx) + negate a)
  , SC.testProperty "toApprox r contains r" $ \r -> approximatedBy r $ toApprox 20 (r :: Rational)
  , SC.testProperty "1/(1/a) contains a" $ \a -> not (0 `approximatedBy` a) SC.==> 
                                                       a `better` (1/(1/a))
  , SC.testProperty "a*(1/a) contains 1" $ \a -> not (0 `approximatedBy` a) SC.==> 1 `approximatedBy` (a*(1/a))
  , SC.testProperty "boundErrorTerm" $
    \a -> case a of
            Approx _ _ _ -> let b@(Approx _ e _) = boundErrorTerm a
                            in (a `better` b) && (e < 1024) -- 1024 is errorBound when errorBits is 10
            Bottom       -> better a (boundErrorTerm a)
  , SC.testProperty "limitSize" $
    \a -> case a of
            Approx _ _ _ -> let b@(Approx _ _ s) = limitSize 2 a
                            in (a `better` b) && (s >= -2)
            Bottom       -> better a (limitSize 2 a)
  , SC.testProperty "sqrt" $ \a -> let b = abs a in better b $ (sqrtA 0 b)^2
  , SC.testProperty "recipA" $ \a -> let b = abs a in 0 `approximatedBy` b || (lowerBound (recipA 100 b) * upperBound b <= 1
                                                                               && upperBound (recipA 100 b) * lowerBound b >= 1)
  ]

qcPropA :: TestTree
qcPropA = testGroup "(checked by quickcheck)"
  [ QC.testProperty "read . show = id" $ \a -> (a :: Approx) == read (show a)
  , QC.testProperty "diameter" $ \a -> diameter a == upperBound a - lowerBound a
  , QC.testProperty "fromDyadic exact and centre" $ \d -> let a = fromDyadic d in exact a && Just d == centre a
  , QC.testProperty "abs negate sign" $ \a -> approximatedBy 0 $ (a ::Approx) + (negate (signum a) * (abs a))
  , QC.testProperty "addition precision" $ \a b -> 
      collect (precision (boundErrorTerm (a+b)) - (precision a `min` precision b))
              True
{-
  , QC.testProperty "multiplication precision" $ \a b -> 
      collect (precision (boundErrorTerm (a*b)) - (precision a `min` precision b))
              True
-}
  , QC.testProperty "a+(-a) contains 0" $ \a -> 0 `approximatedBy` ((a :: Approx) + negate a)
  , QC.testProperty "toApprox r contains r" $ \r -> approximatedBy r $ toApprox 20 (r :: Rational)
  , QC.testProperty "1/(1/a) contains a" $ \a -> not (0 `approximatedBy` a) QC.==> 
                                                       a `better` (1/(1/a))
  , QC.testProperty "a*(1/a) contains 1" $ \a -> not (0 `approximatedBy` a) QC.==> 1 `approximatedBy` (a*(1/a))
  , QC.testProperty "boundErrorTerm" $
    \a -> case a of 
            Approx _ _ _ -> let b@(Approx _ e _) = boundErrorTerm a
                            in (a `better` b) && (e < 1024) -- 1024 is errorBound when errorBits is 10
            Bottom       -> better a (boundErrorTerm a)
  , QC.testProperty "limitSize" $
    \a -> case a of 
            Approx _ _ _ -> let b@(Approx _ _ s) = limitSize 2 a
                            in (a `better` b) && (s >= -2)
            Bottom       -> better a (limitSize 2 a)
  , QC.testProperty "values" $ \a -> let types = a :: Approx in collect a True
  , QC.testProperty "sqr . sqrt" $ \a -> let b = abs a in b `better` sqrA (sqrtA 0 b)
  , QC.testProperty "sqrt . sqr" $ \a -> abs a `better` sqrtA 0 (sqrA a)
  , QC.testProperty "recipA" $ \a -> let b = abs a in 0 `approximatedBy` b || (lowerBound (recipA 100 b) * upperBound b <= 1
                                                                               && upperBound (recipA 100 b) * lowerBound b >= 1)
  ]

creal :: TestTree
creal = testGroup "CReal" [propertiesCR]

genCRApprox :: Gen Approx -> Gen CR
genCRApprox = liftM (CR . pure)

genCR :: Gen CR
genCR = do
  x <- choose (0,1) :: Gen Double
  let (m,_) = decodeFloat x
  s <- choose (-125,-42) :: Gen Int
  s' <- choose (-56,-51) :: Gen Int
  frequency [(2, return . CR . pure $ Approx m 0 s)
            ,(2, return . CR . pure $ Approx m 0 s')
            ,(2, return . CR . pure $ Approx (-m) 0 s)
            ,(2, return . CR . pure $ Approx (-m) 0 s')
            ,(1, return . CR . pure $ Approx m 1 s)
            ,(1, return . CR . pure $ Approx m 1 s')
            ,(1, return . CR . pure $ Approx (-m) 1 s)
            ,(1, return . CR . pure $ Approx (-m) 1 s')
            ]

genCROpen :: CR -> CR -> Gen CR
genCROpen a b = do
  x <- choose (0,1) :: Gen Double
  return $ a + (b-a) * fromDoubleAsExactValue x

genCRClosed :: CR -> CR -> Gen CR
genCRClosed a b = do
  x <- choose (0,1) :: Gen Double
  frequency [(96, return $ a + (b-a) * fromDoubleAsExactValue x)
            ,(2, return a)
            ,(2, return b)
            ]

instance Arbitrary CR where
  arbitrary = genCR

instance Show CR where
  show (CR x) = showA . head . getZipList $ x

checkCRN :: Int -> CR -> CR -> Bool
checkCRN n (CR x) (CR y) = and $ zipWith better (take n $ getZipList x) (take n $ getZipList y)

checkFun :: (CR -> CR) -> CR -> Bool
checkFun f x = let a = require 1 x
                   l = lowerA a
                   u = upperA a
                   fx = require 1 (f x)
               in better (require 200 . f . CR . pure $ l) fx &&
                  better (require 200 . f . CR . pure $ u) fx

propertiesCR :: TestTree
propertiesCR = testGroup "Properties of CReal" [scPropCR, qcPropCR]

scPropCR = testGroup "(checked by smallCheck)"
  []

{-
Some of the test should hold for larger domains than tested over. Consider,
for example, atanh . tanh: with an input of -1000, tanh will give essentially
-1 with some error bound. Given that atanh is computed by a logarithm applied
to (x+1)/(x-1), which is essentially 0, we may exhaust memory before a good
enough result is computed.
-}
qcPropCR = testGroup "(checked by quickCheck)"
  [ QC.testProperty "values" $ \x -> let types = x :: CR in collect (show x) True
  , QC.testProperty "Pythagorean identity" $ QC.forAll (genCRClosed (-pi) pi) $ \x -> checkCRN 5 1 ((sin x)^2 + (cos x)^2)
  , QC.testProperty "^2 . sqrt" $ QC.forAll (genCRClosed 0 100) $ \x -> checkCRN 5 x ((sqrt x)^2)
  , QC.testProperty "sqrt . ^2" $ \x -> checkCRN 5 (abs x) (sqrt (x^2))
  , QC.testProperty "log . exp" $ \x -> checkCRN 5 x (log (exp x))
  , QC.testProperty "exp . log" $ QC.forAll (genCROpen 0 100) $ \x -> checkCRN 5 x (exp (log x))
  , QC.testProperty "asin . sin" $ QC.forAll (genCROpen (-pi/2) (pi/2)) $ \x -> checkCRN 5 x (asin (sin x))
  , QC.testProperty "sin . asin" $ QC.forAll (genCROpen (-1) 1) $ \x -> checkCRN 5 x (sin (asin x))
  , QC.testProperty "acos . cos" $ QC.forAll (genCROpen 0 pi) $ \x -> checkCRN 5 x (acos (cos x))
  , QC.testProperty "cos . acos" $ QC.forAll (genCROpen (-1) 1) $ \x -> checkCRN 5 x (cos (acos x))
  , QC.testProperty "atan . tan" $ QC.forAll (genCROpen (-pi/2) (pi/2)) $ \x -> checkCRN 5 x (atan (tan x))
  , QC.testProperty "tan . atan" $ \x -> checkCRN 1 x (tan (atan x))
  , QC.testProperty "asinh . sinh" $ \x -> checkCRN 5 x (asinh (sinh x))
  , QC.testProperty "sinh . asinh" $ \x -> checkCRN 5 x (sinh (asinh x))
  , QC.testProperty "acosh . cosh" $ QC.forAll (genCRClosed 0 10) $ \x -> checkCRN 5 x (acosh (cosh x))
  , QC.testProperty "cosh . acosh" $ QC.forAll (genCRClosed 1 10) $ \x -> checkCRN 5 x (cosh (acosh x))
  , QC.testProperty "atanh . tanh" $ QC.forAll (genCRClosed (-100) 100) $ \x -> checkCRN 5 x (atanh (tanh x))
  , QC.testProperty "tanh . atanh" $ QC.forAll (genCROpen (-1) 1) $ \x -> checkCRN 5 x (tanh (atanh x))
  , QC.testProperty "recip . recip" $ \x -> checkCRN 5 x (1/(1/x))
  , QC.testProperty "sqrt" $ QC.forAll (genCRClosed 0 100) $ checkFun sqrt
  , QC.testProperty "sin" $ checkFun sin
  , QC.testProperty "cos" $ checkFun cos
  , QC.testProperty "tan" $ checkFun tan
  , QC.testProperty "exp" $ QC.forAll (genCRClosed (-100) 100) $ checkFun exp
  , QC.testProperty "log" $ QC.forAll (genCROpen 0 100) $ checkFun log
  , QC.testProperty "asin" $ QC.forAll (genCRClosed (-1) 1) $ checkFun asin
  , QC.testProperty "acos" $ QC.forAll (genCRClosed (-1) 1) $ checkFun asin
  , QC.testProperty "atan" $ checkFun atan
  , QC.testProperty "sinh" $ QC.forAll (genCRClosed (-100) 100) $ checkFun sinh
  , QC.testProperty "asinh" $ checkFun asinh
  , QC.testProperty "cosh" $ QC.forAll (genCRClosed 0 10) $ checkFun cosh
  , QC.testProperty "acosh" $ QC.forAll (genCRClosed 1 10) $ checkFun acosh
  , QC.testProperty "tanh" $ QC.forAll (genCRClosed (-10) 10) $ checkFun tanh
  , QC.testProperty "atanh" $ QC.forAll (genCROpen (-1) 1) $ checkFun atanh
  ]

unitTests = testGroup "Unit tests"
  [ testCase "showA 1" $ showA (Approx 7 2 2) @?= "3~"
  , testCase "showA 2" $ showA (Approx 7 2 3) @?= "~~"
  , testCase "showA (Approx 99 2 0)" $ showA (Approx 99 2 0) @?= "10~"
  , testCase "sin π/2 = 1 (1000 bits)" $ approximatedBy 1 (require 1000 . sin $ pi/2) @?= True
  ]
