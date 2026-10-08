{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR
    Description :  Instances for MPFR as interval endpoints.  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable (indirect FFI)
    
    Instances of MPFR required for serving as interval endpoints,
    namely providing granularity, Comparison, lattice, rounded field and 
    rounded elementary operations.
-}
module Numeric.AERN.RealArithmetic.Basis.MPFR 
(
--   MPFR, MPFRPrec,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.Basics,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.ShowInternals,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.Conversion,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.MixedFieldOps,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.Elementary,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.Measures,
   module Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
)
where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.ShowInternals
import Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.MPFR.Conversion
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.MixedFieldOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst
import Numeric.AERN.RealArithmetic.Basis.MPFR.Elementary
import Numeric.AERN.RealArithmetic.Basis.MPFR.Measures
import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception

import Data.Word

instance RoundedReal MPFR where
    type RoundedRealEffortIndicator MPFR = ()
    roundedRealDefaultEffort _ = ()
    rrEffortComp _ _ = ()
    rrEffortMinmax _ _ = ()
    rrEffortDistance _ p = ()
    rrEffortToSelf _ _ = ()
    rrEffortToInt _ _ = ()
    rrEffortFromInt _ p = ()
    rrEffortToInteger _ _ = ()
    rrEffortFromInteger _ p = ()
    rrEffortToDouble _ _ = () 
    rrEffortFromDouble _ p = ()
    rrEffortToRational _ _ = ()
    rrEffortFromRational _ p = ()
    rrEffortAbs _ _ = ()
    rrEffortField _ p = ()
    rrEffortIntMixedField _ _ = ()
    rrEffortIntegerMixedField _ _ = ()
    rrEffortDoubleMixedField _ _ = ()
    rrEffortRationalMixedField _ _ = ()
    
instance HasLegalValues MPFR where
    maybeGetProblem d@(MPFR v)
        | isNaN v = Just "A NaN MPFR"
--        | d == 1/0 = False
--        | d == -1/0 = False
        | otherwise = Nothing

    
  