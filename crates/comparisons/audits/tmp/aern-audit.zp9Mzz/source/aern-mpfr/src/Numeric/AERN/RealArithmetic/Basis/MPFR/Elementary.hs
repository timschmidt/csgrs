{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Elementary
    Description :  rounded elementary ops based on MPFR
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded elementary ops based on MPFR.
    
    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Elementary 
()
where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Effort

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up ()

instance RoundedExponentiationEffort MPFR where
    type ExpEffortIndicator MPFR = ()
    expDefaultEffort _ = ()

instance RoundedExponentiation MPFR where
    expUpEff _ d1 = 
        M.exp M.Up (M.getPrec d1) d1
    expDnEff _ d1 =
        M.exp M.Down (M.getPrec d1) d1

instance RoundedSquareRootEffort MPFR where
    type SqrtEffortIndicator MPFR = () 
    sqrtDefaultEffort _ = ()

instance RoundedSquareRoot MPFR where
    sqrtUpEff _ d1 = 
        M.sqrt M.Up (M.getPrec d1) d1
    sqrtDnEff _ d1 =
        M.sqrt M.Down (M.getPrec d1) d1


