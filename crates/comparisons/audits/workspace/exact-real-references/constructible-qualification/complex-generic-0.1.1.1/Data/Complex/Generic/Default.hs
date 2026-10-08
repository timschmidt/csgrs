{- |
Module      :  Data.Complex.Generic.Default
Copyright   :  (c) Claude Heiland-Allen 2012
License     :  BSD3

Maintainer  :  claude@mathr.co.uk
Stability   :  unstable
Portability :  MultiParamTypeClasses

Default implementations of complex number operations.
-}
{- heavily based on:
-- Module      :  Data.Complex
-- Copyright   :  (c) The University of Glasgow 2001
-- License     :  BSD-style (see the file libraries/base/LICENSE)
-- http://hackage.haskell.org/packages/archive/base/4.5.0.0/doc/html/src/Data-Complex.html
--}

module Data.Complex.Generic.Default where

import Data.Complex.Generic.Class

realDefault :: (Num r, ComplexRect c r) => r -> c
realDefault r = r .+ 0

imagDefault :: (Num r, ComplexRect c r) => r -> c
imagDefault i = 0 .+ i

rectDefault :: ComplexRect c r => c -> (r, r)
rectDefault c = (realPart c, imagPart c)

realPartDefault :: ComplexRect c r => c -> r
realPartDefault = fst . rect

imagPartDefault :: ComplexRect c r => c -> r
imagPartDefault = snd . rect

conjugateDefault :: (Num r, ComplexRect c r) => c -> c
conjugateDefault c =
  let (x, y) = rect c
  in  x .+ negate y

magnitudeSquaredDefault :: (Num r, ComplexRect c r) => c -> r
magnitudeSquaredDefault c =
  let (x, y) = rect c
  in  x * x + y * y

sqrDefault :: (Num r, ComplexRect c r) => c -> c
sqrDefault z =
  let (x, y) = rect z
      xy = x * y
  in  (x + y) * (x - y) .+ (xy + xy)

sqrDefaultRF :: (RealFloat r, ComplexRect c r) => c -> c
sqrDefaultRF z =
  let (x, y) = rect z
  in  (x + y) * (x - y) .+ scaleFloat 1 (x * y)  -- FIXME assumes binary

rmulDefault :: (Num r, ComplexRect c r) => r -> c -> c
rmulDefault a z =
  let (x, y) = rect z
  in  (a * x) .+ (a * y)

mulrDefault :: (Num r, ComplexRect c r) => c -> r -> c
mulrDefault z a =
  let (x, y) = rect z
  in  (x * a) .+ (y * a)

mkPolarDefault :: (Floating r, ComplexRect c r) => r -> r -> c
mkPolarDefault r theta = r * cos theta .+ r * sin theta

cisDefault :: (Floating r, ComplexRect c r) => r -> c
cisDefault theta = cos theta .+ sin theta

polarDefault :: (ComplexPolar c r) => c -> (r, r)
polarDefault c = (magnitude c, phase c)

magnitudeDefault :: (Floating r, ComplexRect c r) => c -> r
magnitudeDefault = sqrt . magnitudeSquared

magnitudeDefaultRF :: (RealFloat r, ComplexRect c r) => c -> r
magnitudeDefaultRF w =
  let (x, y) = rect w
      k = max (exponent x) (exponent y)
      mk = - k
      sqr z = z * z
  in  scaleFloat k (sqrt (sqr (scaleFloat mk x) + sqr (scaleFloat mk y)))

phaseDefault :: (Ord r, Floating r, ComplexRect c r) => c -> r
phaseDefault c
  | x > 0            =  atan (y/x)
  | x == 0 && y > 0  =  pi/2
  | x <  0 && y > 0  =  pi + atan (y/x)
  | x <= 0 && y < 0  = -phaseDefault (conjugate c)
  | y == 0 && x < 0  =  pi    -- must be after the previous test on zero y
  | x==0 && y==0     =  y     -- must be after the other double zero tests
  | otherwise        =  x + y -- x or y is a NaN, return a NaN (via +)
  where
    x = realPart c
    y = imagPart c

phaseDefaultRF :: (RealFloat r, ComplexRect c r) => c -> r
phaseDefaultRF c = case (realPart c, imagPart c) of
  (0, 0) -> 0
  (x, y) -> atan2 y x

addDefault :: (Num r, ComplexRect c r) => c -> c -> c
addDefault z w =
  let (x,y) = rect z
      (x',y') = rect w
  in  (x+x') .+ (y+y')

subDefault :: (Num r, ComplexRect c r) => c -> c -> c
subDefault z w =
  let (x,y) = rect z
      (x',y') = rect w
  in  (x-x') .+ (y-y')

mulDefault :: (Num r, ComplexRect c r) => c -> c -> c
mulDefault z w =
  let (x,y) = rect z
      (x',y') = rect w
  in  (x*x'-y*y') .+ (x*y'+y*x')

negateDefault :: (Num r, ComplexRect c r) => c -> c
negateDefault z =
  let (x,y) = rect z
  in  negate x .+ negate y

absDefault :: (Num r, ComplexRect c r, ComplexPolar c r) => c -> c
absDefault = real . magnitude

signumDefault :: (Eq r, Fractional r, ComplexRect c r, ComplexPolar c r) => c -> c
signumDefault z = case rect z of
  (0, 0) -> 0 .+ 0
  (x, y) -> x/r .+ y/r
  where r = magnitude z

fromIntegerDefault :: (Num r, ComplexRect c r) => Integer -> c
fromIntegerDefault = real . fromInteger

divDefault :: (Fractional r, ComplexRect c r) => c -> c -> c
divDefault z w =
  let (x,y) = rect z
      (x',y') = rect w
      d = x'*x' + y'*y'
  in  (x*x'+y*y') / d .+ (y*x'-x*y') / d

divDefaultRF :: (RealFloat r, ComplexRect c r) => c -> c -> c
divDefaultRF z w =
  let (x,y) = rect z
      (x',y') = rect w
      x'' = scaleFloat k x'
      y'' = scaleFloat k y'
      k = max (exponent x') (exponent y')
      d = x'*x'' + y'*y''
  in  (x*x''+y*y'') / d .+ (y*x''-x*y'') / d

fromRationalDefault :: (Fractional r, ComplexRect c r) => Rational -> c
fromRationalDefault = real . fromRational

piDefault :: (Floating r, ComplexRect c r) => c
piDefault = real pi

expDefault :: (Floating r, ComplexRect c r) => c -> c
expDefault z =
  let (x, y) = rect z
      expx = exp x
  in  expx * cos y .+ expx * sin y

logDefault :: (Floating r, ComplexRect c r, ComplexPolar c r) => c -> c
logDefault z = log (magnitude z) .+ phase z

sqrtDefault :: (Eq r, Ord r, Floating r, ComplexRect c r, ComplexPolar c r) => c -> c
sqrtDefault z = case rect z of
  (0, 0) -> 0 .+ 0
  (x, y) ->
    let (u,v) = if x < 0 then (v',u') else (u',v')
        v'    = abs y / (u'*2)
        u'    = sqrt ((magnitude z + abs x) / 2)
    in  u .+ (if y < 0 then -v else v)

sinDefault :: (Floating r, ComplexRect c r) => c -> c
sinDefault z =
  let (x, y) = rect z
  in  sin x * cosh y .+ cos x * sinh y

cosDefault :: (Floating r, ComplexRect c r) => c -> c
cosDefault z =
  let (x, y) = rect z
  in  cos x * cosh y .+ (- sin x * sinh y)

tanDefault :: (Floating r, Fractional c, ComplexRect c r) => c -> c
tanDefault z =
  let (x, y) = rect z
      sinx  = sin x
      cosx  = cos x
      sinhy = sinh y
      coshy = cosh y
  in  (sinx*coshy.+cosx*sinhy)/(cosx*coshy.+(-sinx*sinhy))

sinhDefault :: (Floating r, ComplexRect c r) => c -> c
sinhDefault z =
  let (x, y) = rect z
  in  cos y * sinh x .+ sin  y * cosh x

coshDefault :: (Floating r, ComplexRect c r) => c -> c
coshDefault z =
  let (x, y) = rect z
  in  cos y * cosh x .+ sin y * sinh x

tanhDefault :: (Floating r, Floating c, ComplexRect c r) => c -> c
tanhDefault z =
  let (x, y) = rect z
      siny  = sin y
      cosy  = cos y
      sinhx = sinh x
      coshx = cosh x
  in  (cosy*sinhx.+siny*coshx)/(cosy*coshx.+siny*sinhx)

asinDefault :: (Num r, Floating c, ComplexRect c r) => c -> c
asinDefault z =
  let (x, y) = rect z
      (x', y') = rect $ log (((-y).+x) + sqrt (1 - z*z))
  in  y'.+(-x')

acosDefault :: (Num r, Floating c, ComplexRect c r) => c -> c
acosDefault z =
  let (x'',y'') = rect $ log (z + ((-y').+x'))
      (x',y')   = rect $ sqrt (1 - z*z)
  in  y''.+(-x'')

atanDefault :: (Num r, Floating c, ComplexRect c r) => c -> c
atanDefault z =
  let (x, y) = rect z
      (x',y') = rect $ log (((1-y).+x) / sqrt (1+z*z))
  in  y'.+(-x')

asinhDefault :: (Floating c, ComplexRect c r) => c -> c
asinhDefault z = log (z + sqrt (1+z*z))

acoshDefault :: (Floating c, ComplexRect c r) => c -> c
acoshDefault z = log (z + (z+1) * sqrt ((z-1)/(z+1)))

atanhDefault :: (Floating c, ComplexRect c r) => c -> c
atanhDefault z =  0.5 * log ((1.0+z) / (1.0-z))
