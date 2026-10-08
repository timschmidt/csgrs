module Data.Real.ICReal 
              (CReal, inject, around, approx, approxRange,
               realPowerInt, 
               realRecipWitness, realLnWitness,
               realBasePolynomial, realBasePolynomial2,
               erf,
               sumRealList,
               answer, unsafeMkCReal)  where

import qualified Data.Real.CReal as CReal
import Data.Interval
import Data.Real.Base
import Maybe
import Data.Multinomial
import Test.QuickCheck

radius = CReal.radius

clip a b c | c < a = a
           | b < c = b
           | otherwise = c

intervalExp (Interval (lb, ub)) = 
  Interval (max 0 (lowerBound sample - err), upperBound sample)
 where
  sample = exp $ CBase ub
  maxSlope = upperBound $ sample
  delta = (ub - lb)
  err = maxSlope * delta

intervalLn (Interval (lb, ub)) = 
  Interval (lowerBound mid - err, upperBound mid + err)
 where
  mid = log $ CBase ((lb + ub)/2)
  maxSlope = recip $ lb
  delta = (ub - lb)/2
  err = maxSlope * delta

intervalArcTan (Interval (lb, ub)) = 
  Interval (max (-2) (lowerBound mid - err), min (upperBound mid + err) 2)
 where
  mid = atan $ CBase ((lb + ub)/2)
  err = (ub - lb)/2

intervalSqrt (Interval (lb, ub)) = 
  Interval (max 0 lb'', ub')
 where
  lb' = max 0 lb
  sample = sqrt $ CBase lb'
  lb'' = lowerBound sample
  maxSlope = recip (2*lb'')
  delta = (ub - lb')
  err = maxSlope*delta
  ub' | lb'' > 0 = max ub (upperBound sample + err)
      | otherwise = ub

prop_intervalSqrt :: Interval Rational -> Rational -> Property
prop_intervalSqrt i x = (x^2) `isIn` i  ==> (abs x) `isIn` j
 where
  j = intervalSqrt i

intervalSin (Interval (lb, ub))
  | (-3) < lb && ub < 3 = Interval (clip (-1) 0 lb, clip 0 1 ub)
  | otherwise = Interval (-1, 1) 

intervalCos (Interval (lb, ub)) = Interval (max (-1) (min (f lb) (f ub)), 1)
 where
  f x = 1 - x^2/2

-- For now we just give this simple result --
intervalErf _ = Interval (-1, 1)

data CReal = CReal (Interval Base) CReal.CReal
           | CBase Base

makeCReal x = CReal (CReal.integerInterval x) x
unsafeMkCReal = makeCReal . (CReal.unsafeMkCReal)

setInterval i (CReal _ x) = CReal i x
setInterval _ x = x

lowerBound (CReal (Interval (l,_)) _) = l
lowerBound (CBase x) = x

upperBound (CReal (Interval (_,u)) _) = u
upperBound (CBase x) = x

inject :: Base -> CReal
inject = CBase

dropI :: CReal -> CReal.CReal
dropI (CReal _ x) = x
dropI (CBase x) = CReal.inject x

interval :: CReal -> Interval Base
interval (CBase x) = point x
interval (CReal i _) = i

approx = CReal.approx . dropI

approxRange = CReal.approxRange . dropI

instance Show CReal where
 show = show . dropI

instance Eq CReal where
 (CBase a) == (CBase b) = a == b
 a == b = (dropI a) == (dropI b)

instance Num CReal where
 (CBase a) + (CBase b) = CBase (a+b)
 (CBase a) + (CReal j y) = CReal (intervalTranslate a j) (CReal.realTranslate a y)
 (CReal i x) + (CBase b) = CReal (intervalTranslate b i) (CReal.realTranslate b x)
 (CReal i x) + (CReal j y) = CReal (intervalPlus i j) (CReal.realPlus x y)
 negate (CBase a) = CBase (negate a)
 negate (CReal i x) = CReal (intervalNegate i) (CReal.realNegate x)
 (CBase a) * (CBase b) = CBase (a*b)
 (CBase a) * (CReal j y) = CReal (intervalScale a j) (CReal.realScale a y)
 (CReal i x) * (CBase b) = CReal (intervalScale b i) (CReal.realScale b x)
 (CReal i x) * (CReal j y) = CReal (intervalMult i j) 
                              (CReal.realMultBound (x,i) y)
 abs (CBase a) = CBase (abs a)
 abs (CReal i x) = CReal (intervalAbs i) (CReal.realAbs x)
 signum (CBase a) = CBase (signum a)
 signum x | 0 < lowerBound x = fromInteger 1
          | upperBound x < 0 = fromInteger (-1)
          | otherwise = CBase $ signum $ CReal.proveNonZero $ dropI x
 fromInteger x = CBase (fromInteger x)

instance Fractional CReal where
 recip (CBase a) = CBase (recip a)
 recip (CReal i x) | 0 < lb = CReal (fromJust (intervalRecip i))
                               (CReal.realRecipWitness lb x)
                   | ub < 0 = CReal (fromJust (intervalRecip i))
                               (CReal.realRecipWitness ub x)
                   | otherwise = makeCReal (recip x)
  where
   (Interval (lb,ub)) = i
 fromRational x = CBase (fromRational x)

instance Floating CReal where
 exp (CBase a) = makeCReal $ CReal.rationalExp radius a
 exp (CReal i x) = CReal (intervalExp i) (CReal.realExpBound (x,i))
 log (CBase x) = makeCReal $ CReal.rationalLn x
 log (CReal i x) | 0 < lb = CReal j (CReal.realLnWitness lb x)
                 | otherwise = makeCReal (log x)
  where
   (Interval (lb,ub)) = i
   j = intervalLn i
 pi = CReal (Interval (3,13/4)) pi
 sin (CBase a) | 0==a = CBase 0
               | otherwise = CReal (intervalSin (point a)) (CReal.rationalSin radius a)
 sin (CReal i x) = CReal (intervalSin i) (sin x)
 cos (CBase a) | 0==a = CBase 1
               | otherwise = CReal (intervalCos (point a)) (CReal.rationalCos radius a)
 cos (CReal i x) = CReal (intervalCos i) (sin x)
 atan (CBase a) | 0==a = CBase 0
                | otherwise = makeCReal $ CReal.rationalArcTan a
 atan (CReal i x) = CReal (intervalArcTan i) (CReal.realArcTan x)
 sqrt (CBase a) = makeCReal $ CReal.rationalSqrt a
 sqrt (CReal i x) = CReal (intervalSqrt i) (sqrt x)
 sinh x = (1/2)*(exp x - (exp (-x)))
 cosh x = (1/2)*(exp x + (exp (-x)))
 asin x = atan (x/sqrt(1 - (realPowerInt x 2)))
 acos x = pi/2 - asin x
 acosh x = log (x+sqrt((realPowerInt x 2) - 1))
 asinh x = log (x+sqrt(1+(realPowerInt x 2)))
 atanh x = (1/2)*log ((1 + x) / (1 - x))

erf (CBase a) | 0 == a = CBase 0
              | otherwise = CReal (intervalErf (point a)) (CReal.rationalErrorFunction a)
erf (CReal i x) = CReal (intervalErf i) (CReal.realErrorFunction x)

realPowerInt (CBase x) n = CBase (x^n)
realPowerInt (CReal i x) n = 
  CReal (intervalPower i n) (CReal.realPowerIntBound (x,i) n)

realBasePolynomial p (CBase x) = CBase (evalP p x)
realBasePolynomial p (CReal i x) = 
  CReal (boundPolynomial i p) (CReal.realBasePolynomialBound p (x, i))

realBasePolynomial2 p (CBase x) y = realBasePolynomial (evalP p (constP x)) y
realBasePolynomial2 p (CReal i x) y' =
  CReal (boundPolynomial (i,j) p) 
        (CReal.realBasePolynomial2Bound p (x, i) (y, j))
 where
  j = interval y'
  y = dropI y'

answer n x = CReal.answer n (dropI x)

around x = CReal.around (dropI x)

realRecipWitness _ x@(CBase a) = recip x
realRecipWitness a x | 0 < a = 
  recip $ setInterval (Interval (max a (lowerBound x), (upperBound x))) x
                     | a < 0 =
  recip $ setInterval (Interval ((lowerBound x), min a (upperBound x))) x

realLnWitness _ x@(CBase a) = log x
realLnWitness a x | 0 < a =
  log $ setInterval (Interval (max a (lowerBound x), (upperBound x))) x

sumRealList l = CReal i x
 where
  x = CReal.sumRealList (map dropI l)
  i = foldr intervalPlus (point 0) (map interval l)