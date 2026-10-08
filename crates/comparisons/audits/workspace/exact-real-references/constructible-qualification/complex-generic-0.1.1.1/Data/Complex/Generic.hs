{-# LANGUAGE DeriveDataTypeable, TemplateHaskell, MultiParamTypeClasses, FlexibleInstances #-}
{- |
Module      :  Data.Complex.Generic
Copyright   :  (c) Claude Heiland-Allen 2012
License     :  BSD3

Maintainer  :  claude@mathr.co.uk
Stability   :  unstable
Portability :  DeriveDataTypeable, TemplateHaskell, MultiParamTypeClasses, FlexibleInstances

Complex numbers.
-}
module Data.Complex.Generic
  ( module Data.Complex.Generic.Class
  , Complex((:+))
  , toDataComplex
  , fromDataComplex
  ) where

import Data.Data (Data)
import Data.Typeable (Typeable)

import Foreign.C (CFloat, CDouble)
import Data.Int
import Data.Word
import Data.Fixed
import Data.Ratio

import qualified Data.Complex as X

import Data.Complex.Generic.Class
import Data.Complex.Generic.Default
import Data.Complex.Generic.TH

-- | Complex numbers in rectangular form.
data Complex a = !a :+ !a
  deriving (Eq, Show, Read, Data, Typeable)
infix 6 :+

-- | Convert to 'Data.Complex.Complex'.
toDataComplex :: Complex r -> X.Complex r
toDataComplex (x :+ y) = x X.:+ y

-- | Convert from 'Data.Complex.Complex'.
fromDataComplex :: X.Complex r -> Complex r
fromDataComplex (x X.:+ y) = x :+ y

instance Functor Complex where
  fmap f (x :+ y) = f x :+ f y

mk :: a -> a -> Complex a
{- needed because of this bug(?) in TemplateHaskell
    Illegal variable name: `:+'
    When splicing a TH declaration
-}
mk = (:+)

toPair :: Complex a -> (a, a)
toPair (x :+ y) = (x, y)

deriveComplexRF ''Complex ''Float 'mk 'toPair
deriveComplexRF ''Complex ''Double 'mk 'toPair
deriveComplexRF ''Complex ''CFloat 'mk 'toPair
deriveComplexRF ''Complex ''CDouble 'mk 'toPair

deriveComplexN ''Complex ''Integer 'mk 'toPair
deriveComplexN ''Complex ''Int 'mk 'toPair
deriveComplexN ''Complex ''Int8 'mk 'toPair
deriveComplexN ''Complex ''Int16 'mk 'toPair
deriveComplexN ''Complex ''Int32 'mk 'toPair
deriveComplexN ''Complex ''Int64 'mk 'toPair
deriveComplexN ''Complex ''Word 'mk 'toPair
deriveComplexN ''Complex ''Word8 'mk 'toPair
deriveComplexN ''Complex ''Word16 'mk 'toPair
deriveComplexN ''Complex ''Word32 'mk 'toPair
deriveComplexN ''Complex ''Word64 'mk 'toPair
{-
deriveComplex1F ''Complex ''HasResolution ''Fixed 'mk 'toPair
deriveComplex1F ''Complex ''Integral ''Ratio 'mk 'toPair
-}

instance HasResolution t => ComplexRect (Complex (Fixed t)) (Fixed t) where
  mkRect = (:+)
  rect (x :+ y) = (x, y)
  real = realDefault
  imag = imagDefault
  realPart = realPartDefault
  imagPart = imagPartDefault
  conjugate = conjugateDefault
  magnitudeSquared = magnitudeSquaredDefault
  sqr = sqrDefault
  (.*) = rmulDefault
  (*.) = mulrDefault

instance HasResolution t => Num (Complex (Fixed t)) where
  (+) = addDefault
  (-) = subDefault
  (*) = mulDefault
  negate = negateDefault
  fromInteger = fromIntegerDefault
  abs = error $ "Num.abs: not implementable for Complex Fixed"
  signum = error $ "Num.signum: not implementable for Complex Fixed"

instance HasResolution t => Fractional (Complex (Fixed t)) where
  (/) = divDefault
  fromRational = fromRationalDefault

instance Integral t => ComplexRect (Complex (Ratio t)) (Ratio t) where
  mkRect = (:+)
  rect (x :+ y) = (x, y)
  real = realDefault
  imag = imagDefault
  realPart = realPartDefault
  imagPart = imagPartDefault
  conjugate = conjugateDefault
  magnitudeSquared = magnitudeSquaredDefault
  sqr = sqrDefault
  (.*) = rmulDefault
  (*.) = mulrDefault

instance Integral t => Num (Complex (Ratio t)) where
  (+) = addDefault
  (-) = subDefault
  (*) = mulDefault
  negate = negateDefault
  fromInteger = fromIntegerDefault
  abs = error $ "Num.abs: not implementable for Complex Ratio"
  signum = error $ "Num.signum: not implementable for Complex Ratio"

instance Integral t => Fractional (Complex (Ratio t)) where
  (/) = divDefault
  fromRational = fromRationalDefault
