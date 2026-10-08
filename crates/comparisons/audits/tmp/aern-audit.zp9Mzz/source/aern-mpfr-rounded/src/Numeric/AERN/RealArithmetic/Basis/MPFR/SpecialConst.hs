{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst
    Description :  common constants such as pi
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Common constants such as pi.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps

import Numeric.AERN.RealArithmetic.NumericOrderRounding

instance  
    RoundedSpecialConstEffort MPFR
    where
    type SpecialConstEffortIndicator MPFR = MPFRPrec
    specialConstDefaultEffort sample = getPrecision sample

instance 
    RoundedSpecialConst MPFR
    where
    piUpEff prec _ = withPrec prec pi
    piDnEff prec _ = negate $ withPrecRoundDown prec (- pi)
    eUpEff prec _ = withPrec prec (exp 1)
    eDnEff prec _ = 1 /. (withPrec prec (exp (-1)))
