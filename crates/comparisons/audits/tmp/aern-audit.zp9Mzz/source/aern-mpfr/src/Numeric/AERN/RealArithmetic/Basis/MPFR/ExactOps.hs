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

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up
import qualified Data.Number.MPFR.Mutable as MM

instance HasSampleFromContext MPFR where
    sampleFromContext = 0

instance HasZero MPFR where
    zero sample = M.set M.Near (M.getPrec sample) 0
    
instance HasOne MPFR where
    one sample = M.set M.Near (M.getPrec sample) 1
    
instance HasInfinities MPFR where
    plusInfinity sample = M.set M.Near (M.getPrec sample) $ 1/0
    minusInfinity sample = M.set M.Near (M.getPrec sample)  $ -1/0
    excludesPlusInfinity a = (a /= plusInfinity a)
    excludesMinusInfinity a = (a /= minusInfinity a)
    
instance Neg MPFR where
    neg = negate


    