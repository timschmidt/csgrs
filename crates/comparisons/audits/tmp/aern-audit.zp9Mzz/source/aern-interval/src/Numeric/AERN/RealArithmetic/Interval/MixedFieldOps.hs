{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE CPP #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.MixedFieldOps
    Description :  rounded basic arithmetic operations mixing 2 types
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations mixing an interval and another type.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.MixedFieldOps 
(
    intervalMixedAddInEff, intervalMixedAddOutEff,
    intervalMixedMultInEff, intervalMixedMultOutEff,
    intervalMixedDivInEff, intervalMixedDivOutEff
)
where

import Numeric.AERN.Basics.Interval

--import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Interval.ExactOps
import Numeric.AERN.RealArithmetic.Interval.FieldOps (multiplyIntervals, recipInterval)

import Numeric.AERN.RealArithmetic.Interval.Effort

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.RefinementOrderRounding

import qualified 
       Numeric.AERN.NumericOrder as NumOrd
import qualified 
       Numeric.AERN.RefinementOrder as RefOrd

{---- mixed addition ----}

instance (ArithUpDn.RoundedMixedAddEffort e Integer) => 
    RoundedMixedAddEffort (Interval e) Integer 
    where
    type MixedAddEffortIndicator (Interval e) Integer = ArithUpDn.MixedAddEffortIndicator e Integer
    mixedAddDefaultEffort (Interval l _) n = ArithUpDn.mixedAddDefaultEffort l n

instance (ArithUpDn.RoundedMixedAdd e Integer) => 
    RoundedMixedAdd (Interval e) Integer 
    where
    mixedAddInEff = intervalMixedAddInEff
    mixedAddOutEff = intervalMixedAddOutEff 
    
instance (ArithUpDn.RoundedMixedAddEffort e Int) => 
    RoundedMixedAddEffort (Interval e) Int 
    where
    type MixedAddEffortIndicator (Interval e) Int = ArithUpDn.MixedAddEffortIndicator e Int
    mixedAddDefaultEffort (Interval l _) n = ArithUpDn.mixedAddDefaultEffort l n

instance (ArithUpDn.RoundedMixedAdd e Int) => 
    RoundedMixedAdd (Interval e) Int 
    where
    mixedAddInEff = intervalMixedAddInEff
    mixedAddOutEff = intervalMixedAddOutEff 
    
    
instance (ArithUpDn.RoundedMixedAddEffort e Rational) => 
    RoundedMixedAddEffort (Interval e) Rational 
    where
    type MixedAddEffortIndicator (Interval e) Rational = ArithUpDn.MixedAddEffortIndicator e Rational
    mixedAddDefaultEffort (Interval l _) n = ArithUpDn.mixedAddDefaultEffort l n

instance (ArithUpDn.RoundedMixedAdd e Rational) => 
    RoundedMixedAdd (Interval e) Rational 
    where
    mixedAddInEff = intervalMixedAddInEff
    mixedAddOutEff = intervalMixedAddOutEff 
    
instance (ArithUpDn.RoundedMixedAddEffort e Double) => 
    RoundedMixedAddEffort (Interval e) Double 
    where
    type MixedAddEffortIndicator (Interval e) Double = ArithUpDn.MixedAddEffortIndicator e Double
    mixedAddDefaultEffort (Interval l _) n = ArithUpDn.mixedAddDefaultEffort l n

instance (ArithUpDn.RoundedMixedAdd e Double) => 
    RoundedMixedAdd (Interval e) Double 
    where
    mixedAddInEff = intervalMixedAddInEff
    mixedAddOutEff = intervalMixedAddOutEff 
    
instance (ArithUpDn.RoundedMixedAddEffort e1 e2) => 
    RoundedMixedAddEffort (Interval e1) (Interval e2)
    where
    type MixedAddEffortIndicator (Interval e1) (Interval e2) =
        ArithUpDn.MixedAddEffortIndicator e1 e2 
    mixedAddDefaultEffort (Interval l1 _) (Interval l2 _) = 
        ArithUpDn.mixedAddDefaultEffort l1 l2

instance (ArithUpDn.RoundedMixedAdd e1 e2) => 
    RoundedMixedAdd (Interval e1) (Interval e2) 
    where
    mixedAddInEff effort (Interval l1 r1) (Interval l2 r2) =
        Interval 
            (ArithUpDn.mixedAddUpEff effort l1 l2)
            (ArithUpDn.mixedAddDnEff effort r1 r2)
    mixedAddOutEff effort (Interval l1 r1) (Interval l2 r2) =
        Interval 
            (ArithUpDn.mixedAddDnEff effort l1 l2)
            (ArithUpDn.mixedAddUpEff effort r1 r2)

intervalMixedAddInEff, intervalMixedAddOutEff :: 
    ArithUpDn.RoundedMixedAdd e tn 
    =>
    ArithUpDn.MixedAddEffortIndicator e tn
    -> Interval e -> tn -> Interval e
intervalMixedAddInEff effort (Interval l2 r2) n =
    Interval
        (ArithUpDn.mixedAddUpEff effort l2 n)
        (ArithUpDn.mixedAddDnEff effort r2 n)
intervalMixedAddOutEff effort (Interval l2 r2) n =
    Interval 
        (ArithUpDn.mixedAddDnEff effort l2 n)
        (ArithUpDn.mixedAddUpEff effort r2 n)


{---- mixed multiplication ----}

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiplyEffort (Interval e) Integer 
    where
    type MixedMultEffortIndicator (Interval e) Integer =
        IntervalRealMixedEffort e Integer 
    mixedMultDefaultEffort =
        defaultIntervalRealMixedEffort
     
instance
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiply (Interval e) Integer 
    where
    mixedMultInEff = intervalMixedMultInEff
    mixedMultOutEff = intervalMixedMultOutEff

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiplyEffort (Interval e) Int 
    where
    type MixedMultEffortIndicator (Interval e) Int = 
        IntervalRealMixedEffort e Int 
    mixedMultDefaultEffort =
        defaultIntervalRealMixedEffort

instance
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiply (Interval e) Int 
    where
    mixedMultInEff = intervalMixedMultInEff
    mixedMultOutEff = intervalMixedMultOutEff

instance
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiplyEffort (Interval e) Rational 
    where
    type MixedMultEffortIndicator (Interval e) Rational = 
        IntervalRealMixedEffort e Rational 
    mixedMultDefaultEffort =
        defaultIntervalRealMixedEffort

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiply (Interval e) Rational 
    where
    mixedMultInEff = intervalMixedMultInEff
    mixedMultOutEff = intervalMixedMultOutEff

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiplyEffort (Interval e) Double 
    where
    type MixedMultEffortIndicator (Interval e) Double = 
        IntervalRealMixedEffort e Double 
    mixedMultDefaultEffort =
        defaultIntervalRealMixedEffort

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    RoundedMixedMultiply (Interval e) Double 
    where
    mixedMultInEff = intervalMixedMultInEff
    mixedMultOutEff = intervalMixedMultOutEff

instance 
    (ArithUpDn.RoundedReal e1,
     ArithUpDn.RoundedMixedFieldEffort e1 e2,
     NumOrd.PartialComparison e2)
    => 
    RoundedMixedMultiplyEffort (Interval e1) (Interval e2) 
    where
    type MixedMultEffortIndicator (Interval e1) (Interval e2) = 
        IntervalRealMixedEffort e1 e2
    mixedMultDefaultEffort i1 (Interval l2 _) =
        defaultIntervalRealMixedEffort i1 l2


instance 
    (ArithUpDn.RoundedMixedField e1 e2,
     ArithUpDn.RoundedReal e1,
     NumOrd.PartialComparison e2, HasZero e2)
    => 
    RoundedMixedMultiply (Interval e1) (Interval e2) 
    where
    mixedMultInEff effort i1@(Interval sampleE1 _) i2@(Interval sampleE2 _) =
        fromEndpoints $
        multiplyIntervals 
            (pNonnegNonposEff effortComp1) (pConsAnticons1)
            (pNonnegNonposEff effortComp2) (pConsAnticons2)
            (ArithUpDn.mixedMultUpEff effortMult) (ArithUpDn.mixedMultDnEff effortMult)
            (NumOrd.minUpEff effortMinmax1) -- minL
            (NumOrd.minDnEff effortMinmax1) -- minR
            (NumOrd.maxUpEff effortMinmax1) -- maxL
            (NumOrd.maxDnEff effortMinmax1) -- maxR
            (NumOrd.maxUpEff effortMinmax1) -- combineL
            (NumOrd.minDnEff effortMinmax1) -- combineR
            (getEndpoints i1) (getEndpoints i2)
        where
        pConsAnticons1 (l,r) = (NumOrd.pLeqEff effortComp1 l r, NumOrd.pLeqEff effortComp1 r l)
        pConsAnticons2 (l,r) = (NumOrd.pLeqEff effortComp2 l r, NumOrd.pLeqEff effortComp2 r l)
        effortComp1 = intrealeff_eComp sampleE1 $ intrealmxeff_eEffort effort
        effortComp2 = intrealmxeff_tnComp effort
        effortMinmax1 = intrealeff_eMinmax sampleE1 $ intrealmxeff_eEffort effort
        effortMult = ArithUpDn.mxfldEffortMult sampleE1 sampleE2 $ intrealmxeff_mixedField effort 
    mixedMultOutEff effort i1@(Interval sampleE1 _) i2@(Interval sampleE2 _) =
        fromEndpoints $
        multiplyIntervals 
            (pNonnegNonposEff effortComp1) (pConsAnticons1)
            (pNonnegNonposEff effortComp2) (pConsAnticons2)
            (ArithUpDn.mixedMultDnEff effortMult) (ArithUpDn.mixedMultUpEff effortMult)
            (NumOrd.minDnEff effortMinmax1) -- minL
            (NumOrd.minUpEff effortMinmax1) -- minR
            (NumOrd.maxDnEff effortMinmax1) -- maxL
            (NumOrd.maxUpEff effortMinmax1) -- maxR
            (NumOrd.minDnEff effortMinmax1)
            (NumOrd.maxUpEff effortMinmax1) 
            (getEndpoints i1) (getEndpoints i2)
        where
        pConsAnticons1 (l,r) = (NumOrd.pLeqEff effortComp1 l r, NumOrd.pLeqEff effortComp1 r l)
        pConsAnticons2 (l,r) = (NumOrd.pLeqEff effortComp2 l r, NumOrd.pLeqEff effortComp2 r l)
        effortComp1 = intrealeff_eComp sampleE1 $ intrealmxeff_eEffort effort
        effortComp2 = intrealmxeff_tnComp effort
        effortMinmax1 = intrealeff_eMinmax sampleE1 $ intrealmxeff_eEffort effort
        effortMult = ArithUpDn.mxfldEffortMult sampleE1 sampleE2 $ intrealmxeff_mixedField effort

--type IntervalMixedMultEffortIndicator e tn = 
--    ((NumOrd.PartialCompareEffortIndicator tn, 
--      NumOrd.PartialCompareEffortIndicator e), 
--     NumOrd.MinmaxEffortIndicator e,
--     ArithUpDn.MixedMultEffortIndicator e tn)
--
--intervalMixedMultDefaultEffort :: 
--     (NumOrd.PartialComparison t,
--      NumOrd.PartialComparison tn,
--      NumOrd.RoundedLatticeEffort t,
--      ArithUpDn.RoundedMixedMultiplyEffort t tn) 
--     =>
--     Interval t
--     -> tn
--     -> ((NumOrd.PartialCompareEffortIndicator tn,
--          NumOrd.PartialCompareEffortIndicator t),
--         NumOrd.MinmaxEffortIndicator t,
--         ArithUpDn.MixedMultEffortIndicator t tn)
--intervalMixedMultDefaultEffort (Interval l _) n = 
--    ((NumOrd.pCompareDefaultEffort n, 
--      NumOrd.pCompareDefaultEffort l), 
--     NumOrd.minmaxDefaultEffort l,
--     ArithUpDn.mixedMultDefaultEffort l n) 

intervalMixedMultInEff, intervalMixedMultOutEff :: 
    (ArithUpDn.RoundedReal t, 
     NumOrd.PartialComparison tn,
     HasZero tn,
     ArithUpDn.RoundedMixedField t tn) 
    =>
    IntervalRealMixedEffort t tn -> 
    Interval t -> tn -> Interval t
intervalMixedMultInEff eff i@(Interval l _) n =
    fromEndpoints $
    multiplySingletonWithInterval 
        (pNonnegNonposEff effortCompN)
        (pNonnegNonposEff effortCompE)
        (flip $ ArithUpDn.mixedMultUpEff effortMult)
        (flip $ ArithUpDn.mixedMultDnEff effortMult) 
        (NumOrd.maxUpEff effortMinmax) 
        (NumOrd.minDnEff effortMinmax)
        (zero l) (zero l)
        n i
        where
        effortCompN = intrealmxeff_tnComp eff
        effortMult = ArithUpDn.mxfldEffortMult l n $ intrealmxeff_mixedField eff
        effortMinmax = intrealeff_eMinmax l $ intrealmxeff_eEffort eff
        effortCompE = intrealeff_eComp l $ intrealmxeff_eEffort eff
        
intervalMixedMultOutEff eff i@(Interval l _) n =
    fromEndpoints $
    multiplySingletonWithInterval 
        (pNonnegNonposEff effortCompN)
        (pNonnegNonposEff effortCompE)
        (flip $ ArithUpDn.mixedMultDnEff effortMult) 
        (flip $ ArithUpDn.mixedMultUpEff effortMult)
        (NumOrd.minDnEff effortMinmax)
        (NumOrd.maxUpEff effortMinmax)
        (zero l) (zero l)
        n i
        where
        effortCompN = intrealmxeff_tnComp eff
        effortMult = ArithUpDn.mxfldEffortMult l n $ intrealmxeff_mixedField eff
        effortMinmax = intrealeff_eMinmax l $ intrealmxeff_eEffort eff
        effortCompE = intrealeff_eComp l $ intrealmxeff_eEffort eff
        
multiplySingletonWithInterval :: 
    (tn -> (Maybe Bool, Maybe Bool))
    -> (e -> (Maybe Bool, Maybe Bool))
    -> (tn -> e -> e)
    -> (tn -> e -> e)
    -> (e -> e -> e)
    -> (e -> e -> e)
    -> e
    -> e
    -> tn
    -> Interval e
    -> (e, e)
multiplySingletonWithInterval 
        sNonnegNonpos iNonnegNonpos timesL timesR 
        combineL combineR
        zeroResL zeroResR
        s1 (Interval l2 r2) =
    let _ = [combineL, combineR] in
        case (sNonnegNonpos s1, -- sign of s1 
              iNonnegNonpos l2, -- sign of l2
              iNonnegNonpos r2 -- sign of r2 
             ) of
             
            -- s1 is zero
            ((Just True, Just True), _, _) -> 
                (zeroResL, zeroResR)
 
            -- s1 non negative
            ((Just True, _), _, _) -> 
                (s1 `timesL` l2, s1 `timesR` r2)
            
            -- s1 non positive
            ((_, Just True), _, _) -> 
                (s1 `timesL` r2, s1 `timesR` l2)

            -- nothing known about s1, i2 positive
            (_, (Just True, _), (Just True, _)) -> 
                ((s1 `timesL` r2) `combineL` (s1 `timesL` l2), 
                 (s1 `timesR` r2) `combineR` (s1 `timesR` l2))

            -- nothing known about s1, i2 negative
            (_, (_, Just True), (_, Just True)) -> 
                ((s1 `timesL` r2) `combineL` (s1 `timesL` l2), 
                 (s1 `timesR` r2) `combineR` (s1 `timesR` l2))

            -- both s1 and i2 are around zero
            _ ->
                ((s1 `timesL` l2) `combineL` (s1 `timesL` r2) `combineL` zeroResL,
                 (s1 `timesR` l2) `combineR` (s1 `timesR` r2) `combineR` zeroResR) 
                -- need to include zero to account for 
                -- consistent vs anti-consistent cases giving constant 0
        
{---- mixed division ----}        
        
instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivideEffort (Interval e) Integer 
    where
    type MixedDivEffortIndicator (Interval e) Integer =
        IntervalRealMixedEffort e Integer
    mixedDivDefaultEffort =
        defaultIntervalRealMixedEffort

instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivide (Interval e) Integer 
    where
    mixedDivInEff = intervalMixedDivInEff
    mixedDivOutEff = intervalMixedDivOutEff
    
instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivideEffort (Interval e) Int 
    where
    type MixedDivEffortIndicator (Interval e) Int = 
        IntervalRealMixedEffort e Int
    mixedDivDefaultEffort =
        defaultIntervalRealMixedEffort

instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivide (Interval e) Int 
    where
    mixedDivInEff = intervalMixedDivInEff
    mixedDivOutEff = intervalMixedDivOutEff
    
instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivideEffort (Interval e) Rational 
    where
    type MixedDivEffortIndicator (Interval e) Rational = 
        IntervalRealMixedEffort e Rational
    mixedDivDefaultEffort =
        defaultIntervalRealMixedEffort

instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivide (Interval e) Rational 
    where
    mixedDivInEff = intervalMixedDivInEff
    mixedDivOutEff = intervalMixedDivOutEff
    
instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivideEffort (Interval e) Double 
    where
    type MixedDivEffortIndicator (Interval e) Double = 
        IntervalRealMixedEffort e Double
    mixedDivDefaultEffort =
        defaultIntervalRealMixedEffort

instance
    (ArithUpDn.RoundedReal e)
    => 
    RoundedMixedDivide (Interval e) Double 
    where
    mixedDivInEff = intervalMixedDivInEff
    mixedDivOutEff = intervalMixedDivOutEff
    
intervalMixedDivInEff, intervalMixedDivOutEff :: 
   (ArithUpDn.RoundedReal t, 
    NumOrd.PartialComparison tn,
    HasZero tn, 
    HasInfinities t,
    ArithUpDn.RoundedMixedField t tn) 
   =>
   (IntervalRealMixedEffort t tn) 
   -> 
   Interval t -> tn -> Interval t
intervalMixedDivInEff eff i@(Interval l _) n =
    fromEndpoints $
    multiplySingletonWithInterval 
        (pNonnegNonposEff effortCompN)
        (pNonnegNonposEff effortCompE)
        (flip $ ArithUpDn.mixedDivUpEff effortDiv)
        (flip $ ArithUpDn.mixedDivDnEff effortDiv) 
        (NumOrd.maxUpEff effortMinmax) 
        (NumOrd.minDnEff effortMinmax)
        (plusInfinity l) (minusInfinity l)
        n i
        where
        effortCompN = intrealmxeff_tnComp eff
        effortDiv = ArithUpDn.mxfldEffortDiv l n $ intrealmxeff_mixedField eff
        effortMinmax = intrealeff_eMinmax l $ intrealmxeff_eEffort eff
        effortCompE = intrealeff_eComp l $ intrealmxeff_eEffort eff

intervalMixedDivOutEff eff i@(Interval l _) n =
    fromEndpoints $
    multiplySingletonWithInterval 
        (pNonnegNonposEff effortCompN)
        (pNonnegNonposEff effortCompE)
        (flip $ ArithUpDn.mixedDivDnEff effortDiv) 
        (flip $ ArithUpDn.mixedDivUpEff effortDiv)
        (NumOrd.minDnEff effortMinmax)
        (NumOrd.maxUpEff effortMinmax) 
        (minusInfinity l) (plusInfinity l)
        n i
        where
        effortCompN = intrealmxeff_tnComp eff
        effortDiv = ArithUpDn.mxfldEffortDiv l n $ intrealmxeff_mixedField eff
        effortMinmax = intrealeff_eMinmax l $ intrealmxeff_eEffort eff
        effortCompE = intrealeff_eComp l $ intrealmxeff_eEffort eff
        
instance 
    (ArithUpDn.RoundedReal e1,
     ArithUpDn.RoundedMixedFieldEffort e1 e2,
     ArithUpDn.RoundedReal e2)
    => 
    RoundedMixedDivideEffort (Interval e1) (Interval e2) 
    where
    type MixedDivEffortIndicator (Interval e1) (Interval e2) = 
        (IntervalRealMixedEffort e1 e2, IntervalRealEffort e2)
    mixedDivDefaultEffort i1 i2@(Interval l2 _) =
        (defaultIntervalRealMixedEffort i1 l2,
         defaultIntervalRealEffort i2)
    
    
instance 
    (ArithUpDn.RoundedMixedField e1 e2,
     ArithUpDn.RoundedReal e1,
     ArithUpDn.RoundedReal e2,
     NumOrd.HasExtrema e2)
    => 
    RoundedMixedDivide (Interval e1) (Interval e2) 
    where
    mixedDivOutEff (effort, effort2) i1 i2@(Interval sampleE2 _) =
        mixedMultOutEff effort i1 $ 
            recipInterval 
                (pPosNonnegNegNonposEff effComp) 
                (ArithUpDn.divDnEff effDiv)
                (ArithUpDn.divUpEff effDiv)
                (RefOrd.bottom i2)
                i2
        where
        effComp = intrealeff_eComp sampleE2 effort2
        effDiv = ArithUpDn.fldEffortDiv sampleE2 effField
        effField = ArithUpDn.rrEffortField sampleE2 effE2
        effE2 = intrealeff_eRoundedReal effort2
    mixedDivInEff (effort, effort2) i1 i2@(Interval sampleE2 _) =
        mixedMultInEff effort i1 $ 
            recipInterval 
                (pPosNonnegNegNonposEff effComp) 
                (ArithUpDn.divUpEff effDiv)
                (ArithUpDn.divDnEff effDiv)
                (RefOrd.top i2)
                i2
        where
        effComp = intrealeff_eComp sampleE2 effort2
        effDiv = ArithUpDn.fldEffortDiv sampleE2 effField
        effField = ArithUpDn.rrEffortField sampleE2 effE2
        effE2 = intrealeff_eRoundedReal effort2



{---- mixed ring ----}
        
instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRingEffort (Interval e) Integer
    where
    type MixedRingOpsEffortIndicator (Interval e) Integer =
        IntervalRealMixedEffort e Integer
    mixedRingOpsDefaultEffort =
        defaultIntervalRealMixedEffort
    mxringEffortAdd (Interval l _) n eff =
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxringEffortMult _ _ eff = eff

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRing (Interval e) Integer

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRingEffort (Interval e) Int
    where
    type MixedRingOpsEffortIndicator (Interval e) Int =
        IntervalRealMixedEffort e Int
    mixedRingOpsDefaultEffort =
        defaultIntervalRealMixedEffort
    mxringEffortAdd (Interval l _) n eff =
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxringEffortMult _ _ eff = eff

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRing (Interval e) Int

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRingEffort (Interval e) Rational
    where
    type MixedRingOpsEffortIndicator (Interval e) Rational =
        IntervalRealMixedEffort e Rational
    mixedRingOpsDefaultEffort =
        defaultIntervalRealMixedEffort
    mxringEffortAdd (Interval l _) n eff =
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxringEffortMult _ _ eff = eff

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRing (Interval e) Rational

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRingEffort (Interval e) Double
    where
    type MixedRingOpsEffortIndicator (Interval e) Double =
        IntervalRealMixedEffort e Double
    mixedRingOpsDefaultEffort =
        defaultIntervalRealMixedEffort
    mxringEffortAdd (Interval l _) n eff =
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxringEffortMult _ _ eff = eff

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedRing (Interval e) Double


instance 
    (ArithUpDn.RoundedMixedFieldEffort e1 e2,
     ArithUpDn.RoundedReal e1,
     NumOrd.PartialComparison e2)
    => 
    RoundedMixedRingEffort (Interval e1) (Interval e2)
    where
    type MixedRingOpsEffortIndicator (Interval e1) (Interval e2) =
        IntervalRealMixedEffort e1 e2
    mixedRingOpsDefaultEffort i1 _i2@(Interval sampleE2 _) =
        defaultIntervalRealMixedEffort i1 sampleE2
    mxringEffortAdd (Interval l1 _) (Interval l2 _) eff = 
        ArithUpDn.mxfldEffortAdd l1 l2 $ intrealmxeff_mixedField eff
    mxringEffortMult _ _ eff = eff

instance 
    (ArithUpDn.RoundedMixedField e1 e2,
     ArithUpDn.RoundedReal e1,
     NumOrd.PartialComparison e2, HasZero e2)
    => 
    RoundedMixedRing (Interval e1) (Interval e2)
    
{---- mixed field ----}

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedFieldEffort (Interval e) Integer 
    where
    type MixedFieldOpsEffortIndicator (Interval e) Integer  =
        IntervalRealMixedEffort e Integer
    mixedFieldOpsDefaultEffort = 
        defaultIntervalRealMixedEffort
    mxfldEffortAdd (Interval l _) n eff = 
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxfldEffortMult _ _ eff = eff
    mxfldEffortDiv _ _ eff = eff
    
instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedField (Interval e) Integer 

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedFieldEffort (Interval e) Int 
    where
    type MixedFieldOpsEffortIndicator (Interval e) Int  =
        IntervalRealMixedEffort e Int
    mixedFieldOpsDefaultEffort = 
        defaultIntervalRealMixedEffort
    mxfldEffortAdd (Interval l _) n eff = 
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxfldEffortMult _ _ eff = eff
    mxfldEffortDiv _ _ eff = eff
    
instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedField (Interval e) Int 


instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedFieldEffort (Interval e) Rational 
    where
    type MixedFieldOpsEffortIndicator (Interval e) Rational  =
        IntervalRealMixedEffort e Rational
    mixedFieldOpsDefaultEffort = 
        defaultIntervalRealMixedEffort
    mxfldEffortAdd (Interval l _) n eff = 
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxfldEffortMult _ _ eff = eff
    mxfldEffortDiv _ _ eff = eff
    
instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedField (Interval e) Rational 

instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedFieldEffort (Interval e) Double 
    where
    type MixedFieldOpsEffortIndicator (Interval e) Double  =
        IntervalRealMixedEffort e Double
    mixedFieldOpsDefaultEffort = 
        defaultIntervalRealMixedEffort
    mxfldEffortAdd (Interval l _) n eff = 
        ArithUpDn.mxfldEffortAdd l n $ intrealmxeff_mixedField eff
    mxfldEffortMult _ _ eff = eff
    mxfldEffortDiv _ _ eff = eff
    
instance 
    (ArithUpDn.RoundedReal e) 
    =>
    RoundedMixedField (Interval e) Double 


instance 
    (ArithUpDn.RoundedMixedFieldEffort e1 e2,
     ArithUpDn.RoundedReal e1,
     ArithUpDn.RoundedReal e2)
    => 
    RoundedMixedFieldEffort (Interval e1) (Interval e2)
    where
    type MixedFieldOpsEffortIndicator (Interval e1) (Interval e2) =
        (IntervalRealMixedEffort e1 e2,
         IntervalRealEffort e2)
    mixedFieldOpsDefaultEffort i1 i2@(Interval sampleE2 _) =
        (defaultIntervalRealMixedEffort i1 sampleE2,
         defaultIntervalRealEffort i2)
    mxfldEffortAdd (Interval l1 _) (Interval l2 _) (eff, _) = 
        ArithUpDn.mxfldEffortAdd l1 l2 $ intrealmxeff_mixedField eff
    mxfldEffortMult _ _  (eff, _) = eff
    mxfldEffortDiv _ _  eff = eff

instance 
    (ArithUpDn.RoundedMixedField e1 e2,
     ArithUpDn.RoundedReal e1,
     ArithUpDn.RoundedReal e2,
     NumOrd.HasExtrema e2)
    => 
    RoundedMixedField (Interval e1) (Interval e2)


--{- In the past, the following weird mixed operations were
-- provided as a stepping stone to outer-rounded mixed interval operations: -}
--   
--instance (ArithUpDn.RoundedAddEffort e) => 
--    ArithUpDn.RoundedMixedAddEffort e (Interval e)
--    where
--    type MixedAddEffortIndicator e (Interval e) = 
--        ArithUpDn.AddEffortIndicator e
--    mixedAddDefaultEffort _e (Interval l _r) = 
--        ArithUpDn.addDefaultEffort l
--   
--instance (ArithUpDn.RoundedAdd e) => 
--    ArithUpDn.RoundedMixedAdd e (Interval e) 
--    where
--    mixedAddUpEff effort e i =
--        case addOutEff effort (Interval e e) i of
--            Interval _l r -> r  
--    mixedAddDnEff effort e i =
--        case addOutEff effort (Interval e e) i of
--            Interval l _r -> l  
--
--instance 
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e)) 
--    => 
--    ArithUpDn.RoundedMixedMultiplyEffort e (Interval e) 
--    where
--    type MixedMultEffortIndicator e (Interval e) = 
--        MultEffortIndicator (Interval e)
--    mixedMultDefaultEffort _e i = 
--        multDefaultEffort i
--   
--instance 
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e)) 
--    => 
--    ArithUpDn.RoundedMixedMultiply e (Interval e) 
--    where
--    mixedMultUpEff effort e i =
--        case multOutEff effort (Interval e e) i of
--            Interval _l r -> r  
--    mixedMultDnEff effort e i =
--        case multOutEff effort (Interval e e) i of
--            Interval l _r -> l  
--
--instance 
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e)) 
--    => 
--    ArithUpDn.RoundedMixedDivideEffort e (Interval e) 
--    where
--    type MixedDivEffortIndicator e (Interval e) = 
--        DivEffortIndicator (Interval e)
--    mixedDivDefaultEffort _e i = 
--        divDefaultEffort i
--   
--instance 
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e),
--     NumOrd.HasExtrema e) 
--    => 
--    ArithUpDn.RoundedMixedDivide e (Interval e) 
--    where
--    mixedDivUpEff effort e i =
--        case divOutEff effort (Interval e e) i of
--            Interval _l r -> r  
--    mixedDivDnEff effort e i =
--        case divOutEff effort (Interval e e) i of
--            Interval l _r -> l  
--
--instance
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e),
--     NumOrd.HasExtrema e) 
--    => 
--    ArithUpDn.RoundedMixedRingEffort e (Interval e)
--    where
--    type MixedRingOpsEffortIndicator e (Interval e) =
--        (RingOpsEffortIndicator (Interval e))
--    mixedRingOpsDefaultEffort _ sampleI = ringOpsDefaultEffort sampleI
--    mxringEffortAdd _ sampleI eff = ringEffortAdd sampleI eff
--    mxringEffortMult _ sampleI eff = ringEffortMult sampleI eff
--
--instance
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e),
--     NumOrd.HasExtrema e) 
--    => 
--    ArithUpDn.RoundedMixedRing e (Interval e)
--
--instance
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e),
--     NumOrd.HasExtrema e) 
--    => 
--    ArithUpDn.RoundedMixedFieldEffort e (Interval e)
--    where
--    type MixedFieldOpsEffortIndicator e (Interval e) =
--        (FieldOpsEffortIndicator (Interval e))
--    mixedFieldOpsDefaultEffort _ sampleI = fieldOpsDefaultEffort sampleI
--    mxfldEffortAdd _ sampleI eff = fldEffortAdd sampleI eff
--    mxfldEffortMult _ sampleI eff = fldEffortMult sampleI eff
--    mxfldEffortDiv _ sampleI eff = fldEffortDiv sampleI eff
--
--instance
--    (ArithUpDn.RoundedReal e, 
--     RoundedFieldEffort (Distance e),
--     RefOrd.RoundedLatticeEffort (Distance e),
--     NumOrd.HasExtrema e) 
--    => 
--    ArithUpDn.RoundedMixedField e (Interval e)
--
--{- end of weird mixed operations -}   
