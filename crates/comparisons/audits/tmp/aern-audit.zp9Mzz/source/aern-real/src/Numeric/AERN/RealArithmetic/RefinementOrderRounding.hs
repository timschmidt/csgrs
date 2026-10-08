{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.RefinementOrderRounding
    Description :  common arithmetical operations rounded in/out  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common arithmetical operations rounded in/out.
    
    This module is meant to be imported qualified.
    It is recommended to use the prefix ArithInOut.
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding
(
    module Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion,
    module Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps,
    module Numeric.AERN.RealArithmetic.RefinementOrderRounding.MixedFieldOps,
    module Numeric.AERN.RealArithmetic.RefinementOrderRounding.SpecialConst,
    module Numeric.AERN.RealArithmetic.RefinementOrderRounding.Elementary,
    RoundedReal(..),
    dblToReal, dbldblToReal
)
where

import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.MixedFieldOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.SpecialConst
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Elementary

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.SizeLimits

{-|
   An aggregate class collecting together all functionality
   normally expected from inward/outward rounded approximations to
   real numbers such as real intervals with floating-point endpoints.
   
   It also provides a single aggregate effort indicator type
   from which effort indicators for all the rounded operations can
   be extracted.
-}
class 
    (HasZero t, HasOne t, HasInfinities t, Neg t,
     NumOrd.PartialComparison t, NumOrd.RefinementRoundedLattice t,
     RefOrd.PartialComparison t, RefOrd.RoundedLattice t,
     HasImprecision t, 
--     NumOrd.PartialComparison (Imprecision t), RoundedField (Imprecision t), 
     HasDistance t, 
--     NumOrd.PartialComparison (Distance t), RoundedField (Distance t),
     Convertible t t, ArithUpDn.Convertible t t,
     Convertible Int t, ArithUpDn.Convertible t Int,
     Convertible Integer t, ArithUpDn.Convertible t Integer,  
     Convertible Double t, ArithUpDn.Convertible t Double,  
     Convertible Rational t, ArithUpDn.Convertible t Rational,  
     RoundedAbs t,
     RoundedField t,
     RoundedMixedField t t, 
     RoundedMixedField t Int, 
     RoundedMixedField t Integer, 
     RoundedMixedField t Double, 
     RoundedMixedField t Rational,
     HasSizeLimits t, CanChangeSizeLimits t,
     EffortIndicator (SizeLimits t),
     EffortIndicator (RoundedRealEffortIndicator t)) 
    => 
    RoundedReal t
    where
    type RoundedRealEffortIndicator t
    roundedRealDefaultEffort :: t -> RoundedRealEffortIndicator t
    rrEffortNumComp :: t -> (RoundedRealEffortIndicator t) -> (NumOrd.PartialCompareEffortIndicator t)
    rrEffortMinmaxInOut :: t -> (RoundedRealEffortIndicator t) -> (NumOrd.MinmaxInOutEffortIndicator t)
    rrEffortRefComp :: t -> (RoundedRealEffortIndicator t) -> (RefOrd.PartialCompareEffortIndicator t)
    rrEffortPartialJoin :: t -> (RoundedRealEffortIndicator t) -> (RefOrd.PartialJoinEffortIndicator t)
    rrEffortJoinMeet :: t -> (RoundedRealEffortIndicator t) -> (RefOrd.JoinMeetEffortIndicator t)
    rrEffortImprecision :: t -> (RoundedRealEffortIndicator t) -> (ImprecisionEffortIndicator t)
--    rrEffortImprecisionComp :: t -> (RoundedRealEffortIndicator t) -> (NumOrd.PartialCompareEffortIndicator (Imprecision t))
--    rrEffortImprecisionField :: t -> (RoundedRealEffortIndicator t) -> (FieldOpsEffortIndicator (Imprecision t))
    rrEffortDistance :: t -> (RoundedRealEffortIndicator t) -> (DistanceEffortIndicator t)
--    rrEffortDistanceComp :: t -> (RoundedRealEffortIndicator t) -> (NumOrd.PartialCompareEffortIndicator (Distance t))
--    rrEffortDistanceField :: t -> (RoundedRealEffortIndicator t) -> (FieldOpsEffortIndicator (Distance t))
    rrEffortToSelf :: t -> (RoundedRealEffortIndicator t) -> (ArithUpDn.ConvertEffortIndicator t t)
    rrEffortToInt :: t -> (RoundedRealEffortIndicator t) -> (ArithUpDn.ConvertEffortIndicator t Int)
    rrEffortFromInt :: t -> (RoundedRealEffortIndicator t) -> (ConvertEffortIndicator Int t)
    rrEffortToInteger :: t -> (RoundedRealEffortIndicator t) -> (ArithUpDn.ConvertEffortIndicator t Integer)
    rrEffortFromInteger :: t -> (RoundedRealEffortIndicator t) -> (ConvertEffortIndicator Integer t)
    rrEffortToDouble :: t -> (RoundedRealEffortIndicator t) -> (ArithUpDn.ConvertEffortIndicator t Double)
    rrEffortFromDouble :: t -> (RoundedRealEffortIndicator t) -> (ConvertEffortIndicator Double t)
    rrEffortToRational :: t -> (RoundedRealEffortIndicator t) -> (ArithUpDn.ConvertEffortIndicator t Rational)
    rrEffortFromRational :: t -> (RoundedRealEffortIndicator t) -> (ConvertEffortIndicator Rational t)
    rrEffortAbs :: t -> (RoundedRealEffortIndicator t) -> (AbsEffortIndicator t)
    rrEffortField :: t -> (RoundedRealEffortIndicator t) -> (FieldOpsEffortIndicator t)
    rrEffortSelfMixedField :: t -> (RoundedRealEffortIndicator t) -> (MixedFieldOpsEffortIndicator t t)
    rrEffortIntMixedField :: t -> (RoundedRealEffortIndicator t) -> (MixedFieldOpsEffortIndicator t Int)
    rrEffortIntegerMixedField :: t -> (RoundedRealEffortIndicator t) -> (MixedFieldOpsEffortIndicator t Integer)
    rrEffortDoubleMixedField :: t -> (RoundedRealEffortIndicator t) -> (MixedFieldOpsEffortIndicator t Double)
    rrEffortRationalMixedField :: t -> (RoundedRealEffortIndicator t) -> (MixedFieldOpsEffortIndicator t Rational)


dbldblToReal ::
    (RoundedReal real) 
    =>
    real
    -> 
    Double -> Double -> real
dbldblToReal sampleReal l r =
    (dblToReal sampleReal l)
    `union`
    (dblToReal sampleReal r)
    where
    union = RefOrd.meetOutEff (RefOrd.joinmeetDefaultEffort sampleReal)

dblToReal ::
    (RoundedReal real)
    => 
    real 
    -> 
    Double -> real
dblToReal sampleReal d =
    mixedAddOutEff (mixedAddDefaultEffort sampleReal d) z d
    where
    z = zero sampleReal
    
    