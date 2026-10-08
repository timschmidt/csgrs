{-# LANGUAGE TemplateHaskell, MultiParamTypeClasses, FlexibleInstances, UndecidableInstances #-}
{- |
Module      :  Data.Complex.Generic.TH
Copyright   :  (c) Claude Heiland-Allen 2012,2017
License     :  BSD3

Maintainer  :  claude@mathr.co.uk
Stability   :  unstable
Portability :  TemplateHaskell, MultiParamTypeClasses, FlexibleInstances, UndecidableInstances

Derive instances for complex numbers using template haskell.
-}
module Data.Complex.Generic.TH where

import Data.Typeable (typeOf, typeOf1)
import Language.Haskell.TH

import Data.Complex.Generic.Class
import Data.Complex.Generic.Default

-- | Derive instances for 'RealFloat' types.
deriveComplexRF :: Name {- ^ complex type -} -> Name {- ^ real type -} -> Name {- ^ constructor -} -> Name {- ^ destructor -} -> Q [Dec]
deriveComplexRF cTy' rTy' mkRectI' rectI' = [d|
    instance ComplexRect ($(cTy) $(rTy)) $(rTy) where
      mkRect = $(mkRectI)
      rect = $(rectI)
      real = realDefault
      imag = imagDefault
      realPart = realPartDefault
      imagPart = imagPartDefault
      conjugate = conjugateDefault
      magnitudeSquared = magnitudeSquaredDefault
      sqr = sqrDefault
      (.*) = rmulDefault
      (*.) = mulrDefault

    instance ComplexPolar ($(cTy) $(rTy)) $(rTy) where
      mkPolar = mkPolarDefault
      cis = cisDefault
      polar = polarDefault
      magnitude = magnitudeDefaultRF
      phase = phaseDefaultRF

    instance Num ($(cTy) $(rTy)) where
      (+) = addDefault
      (-) = subDefault
      (*) = mulDefault
      negate = negateDefault
      fromInteger = fromIntegerDefault
      abs = absDefault
      signum = signumDefault

    instance Fractional ($(cTy) $(rTy)) where
      (/) = divDefaultRF
      fromRational = fromRationalDefault

    instance Floating ($(cTy) $(rTy)) where
      pi = piDefault
      exp = expDefault
      log = logDefault
      sqrt = sqrtDefault
      sin = sinDefault
      cos = cosDefault
      tan = tanDefault
      sinh = sinhDefault
      cosh = coshDefault
      tanh = tanhDefault
      asin = asinDefault
      acos = acosDefault
      atan = atanDefault
      asinh = asinhDefault
      acosh = acoshDefault
      atanh = atanhDefault
  |]
  where
    cTy = conT cTy'
    rTy = conT rTy'
    mkRectI = varE mkRectI'
    rectI = varE rectI'

-- | Derive instances for 'Floating' types.
deriveComplexF :: Name {- ^ complex type -} -> Name {- ^ real type -} -> Name {- ^ constructor -} -> Name {- ^ destructor -} -> Q [Dec]
deriveComplexF cTy' rTy' mkRectI' rectI' = [d|
    instance ComplexRect ($(cTy) $(rTy)) $(rTy) where
      mkRect = $(mkRectI)
      rect = $(rectI)
      real = realDefault
      imag = imagDefault
      realPart = realPartDefault
      imagPart = imagPartDefault
      conjugate = conjugateDefault
      magnitudeSquared = magnitudeSquaredDefault
      sqr = sqrDefault
      (.*) = rmulDefault
      (*.) = mulrDefault

    instance ComplexPolar ($(cTy) $(rTy)) $(rTy) where
      mkPolar = mkPolarDefault
      cis = cisDefault
      polar = polarDefault
      magnitude = magnitudeDefault
      phase = phaseDefault

    instance Num ($(cTy) $(rTy)) where
      (+) = addDefault
      (-) = subDefault
      (*) = mulDefault
      negate = negateDefault
      fromInteger = fromIntegerDefault
      abs = absDefault
      signum = signumDefault

    instance Fractional ($(cTy) $(rTy)) where
      (/) = divDefault
      fromRational = fromRationalDefault

    instance Floating ($(cTy) $(rTy)) where
      pi = piDefault
      exp = expDefault
      log = logDefault
      sqrt = sqrtDefault
      sin = sinDefault
      cos = cosDefault
      tan = tanDefault
      sinh = sinhDefault
      cosh = coshDefault
      tanh = tanhDefault
      asin = asinDefault
      acos = acosDefault
      atan = atanDefault
      asinh = asinhDefault
      acosh = acoshDefault
      atanh = atanhDefault
  |]
  where
    cTy = conT cTy'
    rTy = conT rTy'
    mkRectI = varE mkRectI'
    rectI = varE rectI'

-- | Derive instances for 'Num' types.
deriveComplexN :: Name {- ^ complex type -} -> Name {- ^ real type -} -> Name {- ^ constructor -} -> Name {- ^ destructor -} -> Q [Dec]
deriveComplexN cTy' rTy' mkRectI' rectI' = [d|
    instance ComplexRect ($(cTy) $(rTy)) $(rTy) where
      mkRect = $(mkRectI)
      rect = $(rectI)
      real = realDefault
      imag = imagDefault
      realPart = realPartDefault
      imagPart = imagPartDefault
      conjugate = conjugateDefault
      magnitudeSquared = magnitudeSquaredDefault
      sqr = sqrDefault
      (.*) = rmulDefault
      (*.) = mulrDefault

    instance Num ($(cTy) $(rTy)) where
      (+) = addDefault
      (-) = subDefault
      (*) = mulDefault
      negate = negateDefault
      fromInteger = fromIntegerDefault
      abs = error $ "Num.abs: not implementable for " ++ show (typeOf (undefined :: ($(cTy) $(rTy))))
      signum = error $ "Num.signum: not implementable for " ++ show (typeOf (undefined :: ($(cTy) $(rTy))))

  |]
  where
    cTy = conT cTy'
    rTy = conT rTy'
    mkRectI = varE mkRectI'
    rectI = varE rectI'

{-
-- | Derive instances for 'Fractional' types with one class constraint.
deriveComplex1F :: Name {- ^ complex type -} -> Name {- ^ constraint class -} -> Name {- ^ real type constructor -} -> Name {- ^ constructor -} -> Name {- ^ destructor -} -> Q [Dec]
deriveComplex1F cTy' sTy' rTy' mkRectI' rectI' = do
  t' <- newName "t"
  let t = varT t'
  c <- classP sTy' [t]
  is <- [d|
    instance ComplexRect ($(cTy) ($(rTy) $(t))) ($(rTy) $(t)) where
      mkRect = $(mkRectI)
      rect = $(rectI)
      real = realDefault
      imag = imagDefault
      realPart = realPartDefault
      imagPart = imagPartDefault
      conjugate = conjugateDefault
      magnitudeSquared = magnitudeSquaredDefault
      sqr = sqrDefault
      (.*) = rmulDefault
      (*.) = mulrDefault

    instance Num ($(cTy) ($(rTy) $(t))) where
      (+) = addDefault
      (-) = subDefault
      (*) = mulDefault
      negate = negateDefault
      fromInteger = fromIntegerDefault
      abs = error $ "Num.abs: not implementable for " ++ show (typeOf1 (undefined :: $(cTy) (($(rTy) $(t))))) ++ " " ++ show (typeOf1 (undefined :: $(rTy) $(t)))
      signum = error $ "Num.signum: not implementable for " ++ show (typeOf1 (undefined :: $(cTy) (($(rTy) $(t))))) ++ " " ++ show (typeOf1 (undefined :: $(rTy) $(t)))

    instance Fractional ($(cTy) ($(rTy) $(t))) where
      (/) = divDefault
      fromRational = fromRationalDefault
    |]
  return (map (\(InstanceD _ ty decs) -> InstanceD [c] ty decs) is)
  where
    cTy = conT cTy'
    sTy = conT sTy'
    rTy = conT rTy'
    mkRectI = varE mkRectI'
    rectI = varE rectI'
-}
