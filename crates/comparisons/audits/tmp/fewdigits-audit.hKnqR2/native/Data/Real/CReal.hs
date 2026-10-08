{-# OPTIONS_GHC -fglasgow-exts #-}
module Data.Real.CReal
             (CReal, inject, around,
              approx, approxRange, integerInterval,
              BoundedCReal, compact, choke,
              proveNonZeroFrom, proveNonZero,
              realNegate, realTranslate, realPlus, 
              realScale, realMultBound,
              realAbs,
              realRecipWitness,
              realPowerIntBound, realPowerInt, 
              realBasePolynomialBound, realBasePolynomial2Bound,
              rationalExp, realExpBound, 
              rationalSin, realSin, rationalCos, realCos,
              rationalLn, realLnWitness, rationalArcTan, realArcTan,
              scalePi, real2Pi, realPi, realPi2, realPi4,
              rationalSqrt, realSqrt,
              rationalErrorFunction, realErrorFunction,
              sumRealList, unsafeMkCReal,
              radius, answer)  where

import Data.Real.Base
import Data.Real.Complete
import Maybe
import Data.Multinomial
import Data.Interval
import Combinatorics
import Control.Exception

{- Thoughts:  Polynomails should always return a Complete Base.  Fractions grow to large othewise.
              Separate interval aritmetic and (R+Q) optimisations into another file that implements the same interface. -}

radius = 2^^(-51)

newtype CReal = CReal {approx :: Complete Base}
unsafeMkCReal = CReal

inject :: Base -> CReal
inject x = CReal (unit x)

{- The real number is assumed to lie within the closed interval -}
type BoundedCReal = (CReal,Interval Base)

around :: CReal -> Integer
around (CReal f) = round (f (1/2))

integerInterval f = Interval (i-1, i+1)
 where
  i = fromInteger $ around f

compact :: CReal -> BoundedCReal 
compact x = (x,integerInterval x)

choke :: BoundedCReal -> CReal
choke (CReal x, (Interval (lb,ub))) = CReal f
 where
  f eps | y < lb = lb
        | ub < y = ub
        | otherwise = y
   where
    y = x eps

{- produces a regular function whose resulting approximations are 
   small in memory size -}
compress :: Complete Base -> Complete Base
compress x eps = approxBase (x (eps/2)) (eps/2)

mapCR f (CReal x) = CReal $ mapC f (compress x)

mapCR2 f (CReal x) (CReal y) = 
  CReal $ mapC2 f (compress x) (compress y)

bindR f (CReal x) = CReal $ bind f (compress x)

instance Show CReal where
 show x = error "Cannot show a CReal"
 {- show x = show $ map (\n -> squish x ((1/2)^n)) [0..] -}

approxRange :: CReal -> Gauge -> Interval Base
approxRange x eps = Interval (r-eps, r+eps)
 where 
  r = approx x eps

{- proveNonZeroFrom will not terminate if the input is 0 -}
{- Finds some y st 0 < (abs y) <= (abs x) -}
proveNonZeroFrom :: Gauge -> CReal -> Base
proveNonZeroFrom g r | high < 0 = high
                     | 0 < low = low
                     | otherwise = proveNonZeroFrom (g/2) r
 where
  Interval (low, high) = approxRange r g

proveNonZero x = proveNonZeroFrom 1 x

negateCts = mkUniformCts id negate

realNegate :: CReal -> CReal
realNegate = mapCR negateCts

plusBaseCts :: Base -> Base :=> Base
plusBaseCts a = mkUniformCts id (a+)

realTranslate :: Base -> CReal -> CReal
realTranslate a = mapCR (plusBaseCts a)

plusCts :: Base :=> Base :=> Base
plusCts = mkUniformCts id plusBaseCts

realPlus :: CReal -> CReal -> CReal
realPlus = mapCR2 plusCts

-- (==) never returns True.
instance Eq CReal where
 a==b = 0==proveNonZero (realPlus a (realNegate b))

multBaseCts :: Base -> Base :=> Base
multBaseCts 0 = constCts 0
multBaseCts a = mkUniformCts mu (a*)
 where
  mu eps = eps/(abs a)

realScale :: Base -> CReal -> CReal
realScale a = mapCR (multBaseCts a)

{- \x -> (\y -> (x*y)) is uniformly continuous on the domain (abs y) <= maxy -}
multUniformCts :: Base -> Base :=> Base :=> Base
multUniformCts maxy = mkUniformCts mu multBaseCts
 where
  mu eps = assert (maxy>0) (eps/maxy)

{- We need to bound the value of x or y.  I think it is better to bound
   x so I actually compute y*x -}
realMultBound :: BoundedCReal -> CReal -> CReal
realMultBound (x,i) y = mapCR2 (multUniformCts b) y (choke (x,i))
 where
  b = bound i

absCts = mkUniformCts id abs

realAbs :: CReal -> CReal
realAbs = mapCR absCts

instance Num CReal where
 (+) = realPlus
 x * y = realMultBound (compact x) y
 negate = realNegate
 abs = realAbs
 signum = inject . signum . proveNonZero
 fromInteger = inject . fromInteger

{- domain is (-inf, nonZero] if nonZero < 0
   domain is [nonZero, inf) if nonZero > 0 -}
recipUniformCts :: Base -> Base :=> Base
recipUniformCts nonZero = mkUniformCts mu f
 where
  f a | 0 <= nonZero = recip (max nonZero a)
      | otherwise = recip (min a nonZero)
  mu eps = eps*(nonZero^2)

realRecipWitness :: Base -> CReal -> CReal
realRecipWitness nonZero = mapCR (recipUniformCts nonZero)

instance Fractional CReal where
 recip x = realRecipWitness (proveNonZero x) x
 fromRational = inject . fromRational

intPowerCts _ 0 = constCts 1
intPowerCts maxx n = mkUniformCts mu (^n)
 where
  mu eps = assert (maxx > 0) $ eps/((fromIntegral n)*(maxx^(n-1)))

realPowerIntBound :: (Integral b) => BoundedCReal -> b -> CReal
realPowerIntBound (x,i) n = mapCR (intPowerCts b n) (choke (x,i))
 where
  b = bound i

realPowerInt :: (Integral b) => CReal -> b -> CReal
realPowerInt = realPowerIntBound . compact

polynomialUniformCts :: Interval Base -> 
                        Polynomial Base -> Base :=> Base
polynomialUniformCts i p |maxSlope == 0 = constCts (evalP p 0)
                         |otherwise = mkUniformCts mu (evalP p)
 where
  maxSlope = bound $ boundPolynomial i (dx p)
  mu eps = assert (maxSlope > 0) $ eps/maxSlope

realBasePolynomialBound :: Polynomial Base -> BoundedCReal -> CReal
realBasePolynomialBound p (x,i) = mapCR (polynomialUniformCts i p) (choke (x,i))

class MultinomialUniformCts i p r | i p -> r where
 multinomialUniformCts :: i -> p -> r

instance MultinomialUniformCts (Interval Base) (Polynomial Base) (Base :=> Base)
 where
  multinomialUniformCts = polynomialUniformCts

instance (MultinomialUniformCts i p r, VectorQ p, Num p,
          BoundPolynomial i p) =>
          MultinomialUniformCts (Interval Base,i) (Polynomial p) (Base :=> r)
 where
  multinomialUniformCts i p | maxSlope == 0 = constCts $
    multinomialUniformCts (snd i) (evalP p 0)
                            | otherwise = mkUniformCts mu $
    \x -> multinomialUniformCts (snd i) (evalP p (x<*>1))
   where 
    maxSlope = bound $ boundPolynomial i (dx p)
    mu eps = assert (maxSlope > 0) $ eps/maxSlope

realBasePolynomial2Bound :: 
  Polynomial (Polynomial Base) -> BoundedCReal -> BoundedCReal -> CReal
realBasePolynomial2Bound p (x,i) (y,j) = 
  mapCR2 (multinomialUniformCts (i,j) p)
    (choke (x,i)) (choke (y,j))

{- only valid for x <= ln(2).  Works best for |x| <= 1/2 -}
rationalSmallExp :: Base -> CReal
rationalSmallExp x = assert ((abs x)<=(1/2)) $
  CReal $ expTaylorApprox
 where
  m | x <= 0 = 1
    | otherwise = 2
  expTaylorApprox eps =
    sumBase terms
   where
    terms = takeWhile highError $ zipWith (\x y -> x/(fromInteger y)) (powers x) factorials
    highError t = m*(abs t) >= eps

rationalExp :: Base -> Base -> CReal
rationalExp tol x | (abs x) <= tol = rationalSmallExp x
                  | otherwise = realPowerInt (rationalExp tol (x/2)) 2

expUniformCts :: Integer -> Base :=> (Complete Base)
expUniformCts upperBound = mkUniformCts mu (approx . rationalExp radius)
 where
  mu eps | upperBound <= 0 = eps*(2^(-upperBound))
         | otherwise = eps/(3^upperBound)

realExpBound :: BoundedCReal -> CReal
realExpBound a@(x,(Interval (_,ub))) =
  bindR (expUniformCts (ceiling ub)) (choke a)

{-Requires that abs(a!!i+1) < abs(a!!i) and the sign of the terms alternate -}
alternatingSeries :: [Base] -> Complete Base
alternatingSeries a eps = sumBase partSeries
 where
  partSeries = (takeWhile (\x -> (abs x) > eps) a)

rationalSin :: Base -> Base -> CReal
rationalSin tol x | tol <= (abs x) = 
                     realBasePolynomialBound (3*xP - 4*xP^3) 
                       ((rationalSin tol (x/3)),Interval (-1,1))
                  | otherwise = CReal (alternatingSeries series)
 where
  series = fst $ unzip $ iterate (\(t,n) -> (-t*(x^2)/(n^2+n),n+2)) (x, 2)

sinCts :: Base :=> (Complete Base)
sinCts = mkUniformCts id (approx . rationalSin radius)

realSlowSin :: CReal -> CReal
realSlowSin = bindR sinCts

realSin :: CReal -> CReal
realSin x | 0==m = realSlowSin x'
          | 1==m = realSlowCos x'
          | 2==m = negate $ realSlowSin x'
          | 3==m = negate $ realSlowCos x'
 where
  n = around (x / realPi2)
  m = n `mod` 4
  x' = x - (realScale (fromInteger n) realPi2)

rationalCos :: Base -> Base -> CReal
rationalCos tol x = realBasePolynomialBound (1-2*xP^2) 
  ((rationalSin tol (x/2)),Interval (-1,1))

cosCts :: Base :=> (Complete Base)
cosCts = mkUniformCts id (approx . rationalCos radius)

realSlowCos :: CReal -> CReal
realSlowCos = bindR cosCts

realCos :: CReal -> CReal
realCos x | 3==m = realSlowSin x'
          | 0==m = realSlowCos x'
          | 1==m = negate $ realSlowSin x'
          | 2==m = negate $ realSlowCos x'
 where
  n = around (x / realPi2)
  m = n `mod` 4
  x' = x - (realScale (fromInteger n) realPi2)

{- computes ln(x).  only valid for 1<=x<2 -}
rationalSmallLn :: Base -> CReal
rationalSmallLn x = assert (1<=x && x<=(3/2)) $
  CReal $ alternatingSeries (zipWith (*) (poly 1) (tail (powers (x-1))))
 where
  poly n = (1/n):(-1/(n+1)):(poly (n+2))

{- requires that 0<=x -}
rationalLn :: Base -> CReal
rationalLn x | x<1 = negate (posLn (recip x))
             | otherwise = posLn x
 where
  ln43 = rationalSmallLn (4/3)
  ln2 = wideLn 2
  {- good for 1<=x<=2 -}
  wideLn x | x < (3/2) = rationalSmallLn x
           | otherwise = (rationalSmallLn ((3/4)*x)) + ln43
  {- requires that 1<=x -}
  posLn x | n==0 = wideLn x
          | otherwise = (wideLn x') + (realScale n ln2)
   where
    (x',n) = until (\(x,n) -> (x<=2)) (\(x,n) -> (x/2,n+1)) (x,0)

{- domain is [nonZero, inf) -}
lnUniformCts :: Base -> Base :=> (Complete Base)
lnUniformCts nonZero = mkUniformCts mu f
 where
  f x = approx $ rationalLn (max x nonZero)
  mu eps = assert (nonZero > 0) $ eps*nonZero

realLnWitness :: Base -> CReal -> CReal
realLnWitness nonZero = bindR (lnUniformCts nonZero)

{- only valid for (abs x) < 1 -}
rationalSmallArcTan :: Base -> CReal
rationalSmallArcTan x = assert ((abs x)<(1/2)) $ CReal $
  alternatingSeries (zipWith (\x y->x*(y^2)) (series 0) (powers x))
 where
  series n = (x/(n+1)):(-x/(n+3)):(series (n+4))

rationalArcTan :: Base -> CReal
rationalArcTan x | x <= (-1/2) = negate $ posArcTan $ negate x
                 | otherwise = posArcTan x
 where
  {-requires (-1/2) < x-}
  posArcTan x | 2 < x = realPi2 - rationalSmallArcTan (recip x)
              | (1/2) <= x = realPi4 + rationalSmallArcTan y
              | otherwise = rationalSmallArcTan x
   where
    y = (x-1)/(x+1)

arcTanCts :: Base :=> (Complete Base)
arcTanCts = mkUniformCts id (approx . rationalArcTan)

realArcTan :: CReal -> CReal
realArcTan = bindR arcTanCts

{- Computes x * Pi -}
{- http://aemes.mae.ufl.edu/~uhk/PI.html -}
scalePi :: Base -> CReal
scalePi x =
 ((realScale (x*48) (rationalSmallArcTan (1/38))) + 
  (realScale (x*80) (rationalSmallArcTan (1/57)))) +
 ((realScale (x*28) (rationalSmallArcTan (1/239))) +
  (realScale (x*96) (rationalSmallArcTan (1/268))))

real2Pi = scalePi 2
realPi = scalePi 1
realPi2 = scalePi (1/2)
realPi4 = scalePi (1/4)

{- Wolfram's algorithm -}
rationalSqrt :: Base -> CReal
rationalSqrt n | n < 1 = realScale (1/2) (rationalSqrt (4*n))
               | 4 <= n = realScale 2 (rationalSqrt (n/4))
               | otherwise = CReal f
 where
  f eps = vf*ef/8
   where
    (_,vf,ef) = until (\(u,v,e) -> e <= eps) wolfram (n, 0, 4)
  wolfram (u,v,e) | u >= v + 1 = (4*(u-v-1), 2*(v+2), e/2)
                  | otherwise = (4*u, 2*v, e/2)

sqrtCts :: Base :=> (Complete Base)
sqrtCts = mkUniformCts (^2) (approx . rationalSqrt)

realSqrt :: CReal -> CReal
realSqrt = bindR sqrtCts

instance Floating CReal where
 exp x = realExpBound (compact x)
 log x = realLnWitness (proveNonZero x) x
 pi = realPi
 sin = realSin
 cos = realCos
 atan = realArcTan
 sqrt = realSqrt
 sinh x = realScale (1/2) (exp x - (exp (-x)))
 cosh x = realScale (1/2) (exp x + (exp (-x)))
 asin x = atan (x/sqrt(realTranslate 1 (negate (realPowerInt x 2))))
 acos x = realPi2 - asin x
 acosh x = log (x+sqrt(realTranslate (-1) (realPowerInt x 2)))
 asinh x = log (x+sqrt(realTranslate 1 (realPowerInt x 2)))
 atanh x = realScale (1/2) 
   (log ((realTranslate 1 x) / (realTranslate 1 (negate x))))

{- best for (abs x) < 1 -}
rationalErrorFunction :: Base -> CReal
rationalErrorFunction x = (2/(sqrt pi)*) . CReal $
  alternatingSeries (zipWith (\x y->x*(y^2)) (series 0) (powers x))
 where
  series n = (x/((2*n'+1)*facts!!n)):(-x/((2*n'+3)*(facts!!(n+1)))):
             (series (n+2))
   where
    facts = map (fromIntegral) factorials
    n' = fromIntegral n

erfCts :: Base :=> (Complete Base)
erfCts = mkUniformCts mu (approx . rationalErrorFunction)
 where
  mu eps = eps*(4/5)

realErrorFunction :: CReal -> CReal
realErrorFunction = bindR erfCts

sumRealList :: [CReal] -> CReal
sumRealList [] = inject 0
sumRealList l = CReal (\eps -> sum (map (\x -> approx x (eps/n)) l))
 where
  n = fromIntegral $ length l

{- testing stuff is below -}
test0 = CReal id

answer n x = shows (around (realScale (10^n) x))
  "x10^-"++(show n)

