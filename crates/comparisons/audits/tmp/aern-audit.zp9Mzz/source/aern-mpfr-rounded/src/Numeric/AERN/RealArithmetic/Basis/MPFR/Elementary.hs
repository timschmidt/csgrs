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

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps

import Numeric.AERN.RealArithmetic.NumericOrderRounding
import Numeric.AERN.RealArithmetic.NumericOrderRounding.Operators

instance RoundedExponentiationEffort MPFR where
    type ExpEffortIndicator MPFR = ()
    expDefaultEffort _ = ()

instance RoundedExponentiation MPFR where
    expUpEff _ d1 = 
        liftRoundedToMPFR1 exp d1
    expDnEff _ d1 =
        1 /. (liftRoundedToMPFR1 exp (-d1))

instance RoundedSquareRootEffort MPFR where
    type SqrtEffortIndicator MPFR = () 
    sqrtDefaultEffort _ = ()

instance RoundedSquareRoot MPFR where
    sqrtUpEff _ d1 = 
        liftRoundedToMPFR1 sqrt d1
    sqrtDnEff _ d1 =
        1 /. (liftRoundedToMPFR1 sqrt (1 /^ d1))


