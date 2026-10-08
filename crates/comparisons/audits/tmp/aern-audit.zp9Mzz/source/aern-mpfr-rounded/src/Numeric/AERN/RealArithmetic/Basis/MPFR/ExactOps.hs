{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
    Description :  one, zero, negation etc for MPFR numbers
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    One, zero, negation etc for MPFR numbers.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.Rounded as R

instance HasSampleFromContext MPFR where
    sampleFromContext = withPrec defaultPrecision 0 -- useless...

instance HasZero MPFR where
    zero sample = withPrec (getPrecision sample) 0
    
instance HasOne MPFR where
    one sample = withPrec (getPrecision sample) 1
    
instance HasInfinities MPFR where
    plusInfinity sample = withPrec (getPrecision sample) $ 1/0
    minusInfinity sample = withPrec (getPrecision sample)  $ -1/0
    excludesPlusInfinity a = (a /= plusInfinity a)
    excludesMinusInfinity a = (a /= minusInfinity a)
    
instance Neg MPFR where
    neg = liftRoundedToMPFR1 negate


    