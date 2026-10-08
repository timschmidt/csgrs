{-
  this file is needed to break the following dependency cycles:

  Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.Elementary
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
             Numeric.AERN.RealArithmetic.NumericOrderRounding.Elementary
  Numeric.AERN.RealArithmetic.Measures
    imports: Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
  Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding
  Numeric.AERN.RealArithmetic.Laws
    imports: Numeric.AERN.RealArithmetic.Measures
  Numeric.AERN.RealArithmetic.NumericOrderRounding
    imports: Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace
             Numeric.AERN.RealArithmetic.NumericOrderRounding.Elementary
             Numeric.AERN.RealArithmetic.NumericOrderRounding.MixedFieldOps
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
  Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace
    imports: Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.Elementary
             Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.MixedFieldOps
             Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.FieldOps
  Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.MixedFieldOps
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
             Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.FieldOps
             Numeric.AERN.RealArithmetic.NumericOrderRounding.MixedFieldOps
  Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
  Numeric.AERN.RealArithmetic.NumericOrderRounding.InPlace.FieldOps
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
  Numeric.AERN.RealArithmetic.NumericOrderRounding.MixedFieldOps
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
  Numeric.AERN.RealArithmetic.NumericOrderRounding.Elementary
    imports: Numeric.AERN.RealArithmetic.Measures
             Numeric.AERN.RealArithmetic.Laws
             Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
  
-} 

{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE ImplicitParams #-}
{-# LANGUAGE FlexibleContexts #-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps where

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion

import Numeric.AERN.Basics.Effort
import qualified Numeric.AERN.NumericOrder as NumOrd

class
    (EffortIndicator (AddEffortIndicator t))
    => 
    RoundedAddEffort t 
    where
    type AddEffortIndicator t
    addDefaultEffort :: t -> AddEffortIndicator t

class (RoundedAddEffort t) => RoundedAdd t where
    addInEff :: AddEffortIndicator t -> t -> t -> t
    addOutEff :: AddEffortIndicator t -> t -> t -> t

class (RoundedAdd t, Neg t) => RoundedSubtr t where
    subtrInEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrOutEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrInEff effort a b = addInEff effort a (neg b)
    subtrOutEff effort a b = addOutEff effort a (neg b)
