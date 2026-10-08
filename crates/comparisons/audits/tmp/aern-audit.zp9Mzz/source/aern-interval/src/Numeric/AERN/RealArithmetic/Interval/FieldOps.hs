{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.FieldOps
    Description :  refinement rounded basic operations for intervals
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Refinement rounded basic operations for intervals.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.FieldOps
(multiplyIntervals, recipInterval)
where

import Numeric.AERN.Basics.Interval

import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Interval.ExactOps ()

import Numeric.AERN.RealArithmetic.Interval.Effort

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import Numeric.AERN.RealArithmetic.RefinementOrderRounding

import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd


instance (ArithUpDn.RoundedAddEffort e) => RoundedAddEffort (Interval e) where
    type AddEffortIndicator (Interval e) = ArithUpDn.AddEffortIndicator e
    addDefaultEffort (Interval sampleE _) = ArithUpDn.addDefaultEffort sampleE

instance (ArithUpDn.RoundedAdd e) => RoundedAdd (Interval e) where
    addInEff effort (Interval l1 r1) (Interval l2 r2) =
        Interval 
            (ArithUpDn.addUpEff effort l1 l2)
            (ArithUpDn.addDnEff effort r1 r2)
    addOutEff effort (Interval l1 r1) (Interval l2 r2) =
        Interval 
            (ArithUpDn.addDnEff effort l1 l2)
            (ArithUpDn.addUpEff effort r1 r2)

instance (ArithUpDn.RoundedAdd e, Neg e) => RoundedSubtr (Interval e)

instance 
    (NumOrd.PartialComparison e, 
     NumOrd.RoundedLatticeEffort e) => 
    RoundedAbsEffort (Interval e)
    where
    type AbsEffortIndicator (Interval e) = 
        IntervalOrderEffort e
    absDefaultEffort i = 
        defaultIntervalOrderEffort i 

instance 
    (ArithUpDn.RoundedAbs e,  
     HasZero e, Neg e,
     NumOrd.PartialComparison e, 
     NumOrd.RoundedLattice e) => 
    RoundedAbs (Interval e)
    where
    absOutEff eff = absOutUsingCompMax (eff, eff)
    absInEff eff = absInUsingCompMax (eff, eff)


instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e)) 
    => 
    RoundedMultiplyEffort (Interval e)
    where
    type MultEffortIndicator (Interval e) =
        IntervalRealEffort e
    multDefaultEffort i =
        defaultIntervalRealEffort i 

instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e)) 
    => 
    RoundedMultiply (Interval e)
    where
    multOutEff effort i1@(Interval sampleE _) i2 =
        fromEndpoints $
        multiplyIntervals 
            (pNonnegNonposEff effortComp) (pConsAnticons)
            (pNonnegNonposEff effortComp) (pConsAnticons)
            (ArithUpDn.multDnEff effortMult) (ArithUpDn.multUpEff effortMult)
            (NumOrd.minDnEff effortMinmax) -- minL
            (NumOrd.minUpEff effortMinmax) -- minR
            (NumOrd.maxDnEff effortMinmax) -- maxL
            (NumOrd.maxUpEff effortMinmax) -- maxR
            (NumOrd.minDnEff effortMinmax)
            (NumOrd.maxUpEff effortMinmax) 
            (getEndpoints i1) (getEndpoints i2)
        where
        pConsAnticons (l,r) = (NumOrd.pLeqEff effortComp l r, NumOrd.pLeqEff effortComp r l)

        effortComp = intrealeff_eComp sampleE effort
        effortMinmax = intrealeff_eMinmax sampleE effort
        effortMult = ArithUpDn.fldEffortMult sampleE effField 
        effField = ArithUpDn.rrEffortField sampleE effE 
        effE = intrealeff_eRoundedReal effort
        
    multInEff effort i1@(Interval sampleE _) i2 =
        fromEndpoints $
        multiplyIntervals 
            (pNonnegNonposEff effortComp) (pConsAnticons)
            (pNonnegNonposEff effortComp) (pConsAnticons)
            (ArithUpDn.multUpEff effortMult) (ArithUpDn.multDnEff effortMult)
            (NumOrd.minUpEff effortMinmax) -- minL
            (NumOrd.minDnEff effortMinmax) -- minR
            (NumOrd.maxUpEff effortMinmax) -- maxL
            (NumOrd.maxDnEff effortMinmax) -- maxR
            (NumOrd.maxUpEff effortMinmax) -- combineL
            (NumOrd.minDnEff effortMinmax) -- combineR
            (getEndpoints i1) (getEndpoints i2)
        where
        pConsAnticons (l,r) = (NumOrd.pLeqEff effortComp l r, NumOrd.pLeqEff effortComp r l)
        
        effortComp = intrealeff_eComp sampleE effort
        effortMinmax = intrealeff_eMinmax sampleE effort
        effortMult = ArithUpDn.fldEffortMult sampleE effField 
        effField = ArithUpDn.rrEffortField sampleE effE 
        effE = intrealeff_eRoundedReal effort
    
multiplyIntervals :: 
   HasZero t 
   =>
      (t1 -> (Maybe Bool, Maybe Bool))
   -> ((t1, t1) -> (Maybe Bool, Maybe Bool))
   -> (t2 -> (Maybe Bool, Maybe Bool))
   -> ((t2, t2) -> (Maybe Bool, Maybe Bool))
   -> (t1 -> t2 -> t)
   -> (t1 -> t2 -> t)
   -> (t -> t -> t)
   -> (t -> t -> t)
   -> (t -> t -> t)
   -> (t -> t -> t)
   -> (t -> t -> t)
   -> (t -> t -> t)
   -> (t1, t1)
   -> (t2, t2)
   -> (t, t)
multiplyIntervals
        pNonnegNonpos1 pConsistency1 
        pNonnegNonpos2 pConsistency2
        timesL timesR 
        minL minR maxL maxR 
        combineL combineR 
        (l1, r1) (l2, r2)
            --------------------------------------------------------------
            -- Cases where either i1 or i2 is shown to be positive/negative
            --------------------------------------------------------------
            -------- i1 negative:
            -- i1 negative, i2 positive
            | i1Nonpos && i2Nonneg = 
                (l1 `timesL` r2, r1 `timesR` l2)
            -- i1 negative, i2 negative
            | i1Nonpos && i2Nonpos = 
                (r1 `timesL` r2, l1 `timesR` l2)
            -- i1 negative, i2 consistent and containing zero
            | i1Nonpos && i2ContZero = 
                (l1 `timesL` r2, l1 `timesR` l2)
            -- i1 negative, i2 anti-consistent and anti-containing zero
            | i1Nonpos && i2AntiContZero = 
                (r1 `timesL` r2, r1 `timesR` l2)
            -- i1 negative, nothing known about i2:
            | i1Nonpos = 
                -- this could be any combination of the four cases above:
                ((r1 `timesL` r2) `combineL` (l1 `timesL` r2), 
                 (r1 `timesR` l2) `combineR` (l1 `timesR` l2))

            -------- i1 positive:
            -- i1 positive, i2 positive
            | i1Nonneg && i2Nonneg = 
                (l1 `timesL` l2, r1 `timesR` r2)
            -- i1 positive, i2 negative
            | i1Nonneg && i2Nonpos = 
                (r1 `timesL` l2, l1 `timesR` r2)
            -- i1 positive, i2 consistent and containing zero
            | i1Nonneg && i2ContZero = 
                (r1 `timesL` l2, r1 `timesR` r2)
            -- i1 positive, i2 anti-consistent and anti-containing zero
            | i1Nonneg && i2AntiContZero = 
                (l1 `timesL` l2, l1 `timesR` r2)
            -- i1 positive, nothing known about i2:
            | i1Nonneg = 
                -- this could be any combination of the four cases above:
                ((r1 `timesL` l2) `combineL` (l1 `timesL` l2), 
                 (r1 `timesR` r2) `combineR` (l1 `timesR` r2))
 
            -------- i2 positive:
            -- i1 consistent and containing zero, i2 positive
            | i1ContZero && i2Nonneg = 
                (l1 `timesL` r2, r1 `timesR` r2)
            -- i1 anti-consistent and anti-containing zero, i2 positive
            | i1AntiContZero && i2Nonneg = 
                (l1 `timesL` l2, r1 `timesR` l2)
            -- nothing known about i1, i2 positive
            | i2Nonneg = 
                ((l1 `timesL` r2) `combineL` (l1 `timesL` l2), 
                 (r1 `timesR` r2) `combineR` (r1 `timesR` l2))

            -------- i2 negative:
            -- i1 consistent and containing zero, i2 negative
            | i1ContZero && i2Nonpos = 
                (r1 `timesL` l2, l1 `timesR` l2)
            -- i1 anti-consistent and anti-containing zero, i2 negative
            | i1AntiContZero && i2Nonpos = 
                (r1 `timesL` r2, l1 `timesR` r2)
            -- nothing known about i1, i2 negative
            | i2Nonpos = 
                ((r1 `timesL` r2) `combineL` (r1 `timesL` l2), 
                 (l1 `timesR` r2) `combineR` (l1 `timesR` l2))

            -----------------------------------------------------------
            -- cases where both i1 or i2 are around zero
            -----------------------------------------------------------

            -- both consistent, at least one contains zero
            | (i1ContZero && i2Consistent) || (i2ContZero && i1Consistent) =
                ((l1 `timesL` r2) `minL` (r1 `timesL` l2), 
                 (l1 `timesR` l2) `maxR` (r1 `timesR` r2))
            -- i1 consistent and containing zero, i2 anti-consistent and anti-containing zero
            | i1ContZero && i2AntiContZero =
                (z, z)
            -- i1 consistent and containing zero, i2 unknown
            | i1ContZero =
                (((l1 `timesL` r2) `combineL` (r1 `timesL` l2)) `combineL` z,
                 ((l1 `timesR` l2) `combineR` (r1 `timesR` r2)) `combineR` z)
                
            -- i1 anti-consistent and anti-containing zero, i2 consistent and containing zero
            | i1AntiContZero && i2ContZero =
                (z, z)
            -- i1 anti-consistent and anti-containing zero, i2 anti-consistent and anti-containing zero
            | i1AntiContZero && i2AntiContZero =
--            | (i1AntiContZero && i2Anticonsistent) || (i1Anticonsistent && i2AntiContZero) = -- TODO: is this correct? 
                ((l1 `timesL` l2) `maxL` (r1 `timesL` r2),
                 (l1 `timesR` r2) `minR` (r1 `timesR` l2)) 
            -- i1 anti-consistent and anti-containing zero, i2 unknown
            | i1AntiContZero = 
                ((l1 `timesL` l2) `combineL` (r1 `timesL` r2) `combineL` z,
                 (l1 `timesR` r2) `combineR` (r1 `timesR` l2) `combineR` z) 
                
            -- i1 unknown, i2 anti-consistent and anti-containing zero
            | i2AntiContZero = 
                ((l1 `timesL` l2) `combineL` (r1 `timesL` r2) `combineL` z,
                 (l1 `timesR` r2) `combineR` (r1 `timesR` l2) `combineR` z) 

            -- i1 unknown, i2 consistent and containing zero
            | i2ContZero =
                ((l1 `timesL` r2) `combineL` (r1 `timesL` l2) `combineL` z, 
                 (l1 `timesR` l2) `combineR` (r1 `timesR` r2) `combineR` z)

            -- both i1 and i2 unknown
            | otherwise =
                (foldl1 combineL [l1 `timesL` r2, r1 `timesL` l2, l1 `timesL` l2, r1 `timesL` r2], 
                 foldl1 combineR [l1 `timesR` r2, r1 `timesR` l2, l1 `timesR` l2, r1 `timesR` r2])
        where
        z = zero sampleT
        sampleT = l1 `timesL` l2  

        -- the 2 x 4 booleans that determine the 16 cases of the Kaucher multiplication table: 
        i1Nonpos = l1Nonpos && r1Nonpos
        i1Nonneg = l1Nonneg && r1Nonneg
        i1ContZero = l1Nonpos && r1Nonneg
        i1AntiContZero = l1Nonneg && r1Nonpos
         
        i2Nonpos = l2Nonpos && r2Nonpos
        i2Nonneg = l2Nonneg && r2Nonneg
        i2ContZero = l2Nonpos && r2Nonneg
        i2AntiContZero = l2Nonneg && r2Nonpos
         
        [l1Nonneg,  l1Nonpos,  r1Nonneg,  r1Nonpos,  l2Nonneg,  l2Nonpos,  r2Nonneg,  r2Nonpos] =
            map (== Just True)
            [l1MNonneg, l1MNonpos, r1MNonneg, r1MNonpos, l2MNonneg, l2MNonpos, r2MNonneg, r2MNonpos]
        (l1MNonneg, l1MNonpos) = pNonnegNonpos1 l1 
        (r1MNonneg, r1MNonpos) = pNonnegNonpos1 r1 
        (l2MNonneg, l2MNonpos) = pNonnegNonpos2 l2 
        (r2MNonneg, r2MNonpos) = pNonnegNonpos2 r2
        
        -- consistency information can help when the above is not available:
        [i1Consistent,  i1Anticonsistent, i2Consistent,  i2Anticonsistent] =
            map (== Just True)
            [i1MConsistent,  i1MAnticonsistent, i2MConsistent,  i2MAnticonsistent]
        (i1MConsistent,  i1MAnticonsistent) = pConsistency1 (l1, r1)
        (i2MConsistent,  i2MAnticonsistent) = pConsistency2 (l2, r2)


instance
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e)) 
    => 
    RoundedPowerToNonnegIntEffort (Interval e)
    where
    type PowerToNonnegIntEffortIndicator (Interval e) =
        IntervalRealEffort e
    powerToNonnegIntDefaultEffort i =
        defaultIntervalRealEffort i

instance
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e)) 
    => 
    RoundedPowerToNonnegInt (Interval e)
    where
    powerToNonnegIntInEff effort i@(Interval l r) n =
        case (pNonnegNonposEff effComp l, pNonnegNonposEff effComp r) of
            ((Just True, _), (Just True, _)) -> -- both non-negative
                Interval lPowerUp hPowerDn
            ((_, Just True), (_, Just True)) -> -- both non-positive
                case even n of
                    True -> Interval hNegPowerUp lNegPowerDn -- switching sign!
                    False -> Interval lNegNegPowerUp hNegNegPowerDn
            _ -> -- may involve crossing zero, revert to the default:
                case even n of
                    True -> 
                        NumOrd.maxInEff effOrd z iPowerFromMult 
                        -- take advantage of the fact that the result is non-negative 
                    False -> iPowerFromMult 
        where
        lPowerUp = ArithUpDn.powerNonnegToNonnegIntUpEff effPowerEndpt l n
        hPowerDn = ArithUpDn.powerNonnegToNonnegIntDnEff effPowerEndpt r n
        lNegPowerDn = ArithUpDn.powerNonnegToNonnegIntDnEff effPowerEndpt (neg l) n
        hNegPowerUp = ArithUpDn.powerNonnegToNonnegIntUpEff effPowerEndpt (neg r) n
        lNegNegPowerUp = neg lNegPowerDn
        hNegNegPowerDn = neg hNegPowerUp
        iPowerFromMult = powerToNonnegIntInEffFromMult effort i n 
        z = zero i
        
        effComp = intrealeff_eComp sampleE effort
        effOrd = intrealeff_intordeff sampleE effort
        
        effPowerEndpt = ArithUpDn.fldEffortPow sampleE effField
        effField = ArithUpDn.rrEffortField sampleE effE
        effE = intrealeff_eRoundedReal effort 
        sampleE = l
        
    powerToNonnegIntOutEff effort i@(Interval l r) n =
        case (pNonnegNonposEff effComp l, pNonnegNonposEff effComp r) of
            ((Just True, _), (Just True, _)) -> -- both non-negative
                Interval lPowerDn hPowerUp
            ((_, Just True), (_, Just True)) -> -- both non-positive
                case even n of
                    True -> Interval hNegPowerDn lNegPowerUp -- switching sign!
                    False -> Interval lNegNegPowerDn hNegNegPowerUp
            _ -> -- may involve crossing zero, revert to the default:
                case even n of
                    True -> 
                        NumOrd.maxOutEff effOrd z iPowerFromMult 
                        -- take advantage of the fact that the result is non-negative 
                    False -> iPowerFromMult 
        where
        lPowerDn = ArithUpDn.powerNonnegToNonnegIntDnEff effPowerEndpt l n
        hPowerUp = ArithUpDn.powerNonnegToNonnegIntUpEff effPowerEndpt r n
        lNegPowerUp = ArithUpDn.powerNonnegToNonnegIntUpEff effPowerEndpt (neg l) n
        hNegPowerDn = ArithUpDn.powerNonnegToNonnegIntDnEff effPowerEndpt (neg r) n
        lNegNegPowerDn = neg lNegPowerUp
        hNegNegPowerUp = neg hNegPowerDn
        iPowerFromMult = powerToNonnegIntOutEffFromMult effort i n 
        z = zero i

        effComp = intrealeff_eComp sampleE effort
        effOrd = intrealeff_intordeff sampleE effort
        
        effPowerEndpt = ArithUpDn.fldEffortPow sampleE effField
        effField = ArithUpDn.rrEffortField sampleE effE
        effE = intrealeff_eRoundedReal effort 
        sampleE = l

instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e)) 
    =>
    RoundedDivideEffort (Interval e)
    where
    type DivEffortIndicator (Interval e) =
        IntervalRealEffort e
    divDefaultEffort i =
        defaultIntervalRealEffort i 

instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     NumOrd.HasExtrema e)
    => 
    RoundedDivide (Interval e)
    where
    divOutEff effort i1@(Interval sampleE _) i2 =
        multOutEff effort i1 $ 
            recipInterval 
                (pPosNonnegNegNonposEff effComp) 
                (ArithUpDn.divDnEff effDiv)
                (ArithUpDn.divUpEff effDiv)
                (RefOrd.bottom i1)
                i2
        where
        effComp = intrealeff_eComp sampleE effort
        
        effDiv = ArithUpDn.fldEffortDiv sampleE effField
        effField = ArithUpDn.rrEffortField sampleE effE
        effE = intrealeff_eRoundedReal effort 
        
    divInEff effort i1@(Interval sampleE _) i2 =
        multInEff effort i1 $ 
            recipInterval 
                (pPosNonnegNegNonposEff effComp) 
                (ArithUpDn.divUpEff effDiv)
                (ArithUpDn.divDnEff effDiv)
                (RefOrd.top i1)
                i2
        where
        effComp = intrealeff_eComp sampleE effort
        
        effDiv = ArithUpDn.fldEffortDiv sampleE effField
        effField = ArithUpDn.rrEffortField sampleE effE
        effE = intrealeff_eRoundedReal effort 



recipInterval :: 
   (NumOrd.HasExtrema e, HasOne e) 
   =>
   (e -> (Maybe Bool, Maybe Bool, Maybe Bool, Maybe Bool))
   -> (e -> e -> e)
   -> (e -> e -> e)
   -> Interval e
   -> Interval e
   -> Interval e
recipInterval pPosNonnegNegNonpos divL divR fallback (Interval l r) =
    case (pPosNonnegNegNonpos l, pPosNonnegNegNonpos r) of
        -- positive:
        ((Just True, _, _, _), (Just True, _, _, _)) ->  
             Interval (divL o r) (divR o l)
        -- negative:
        ((_, _, Just True, _), (_, _, Just True, _)) ->  
             Interval (divL o r) (divR o l)
        -- consistent around zero:
        ((_, _, _, Just True), (_, Just True, _, _)) ->
             (RefOrd.bottom fallback)
        -- anti-consistent around zero:
        ((_, Just True, _, _), (_,_,_, Just True)) ->  
             (RefOrd.top fallback)
        -- unknown:
        _ ->  
             fallback
    where
    o = one sampleE
    sampleE = l 

instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     NumOrd.HasExtrema e)
    =>
    RoundedRingEffort (Interval e)
    where
    type RingOpsEffortIndicator (Interval e) =
        IntervalRealEffort e
    ringOpsDefaultEffort i =
        defaultIntervalRealEffort i
    ringEffortAdd (Interval sampleE _) effort =
        ArithUpDn.fldEffortAdd sampleE effortField
        where
        effortField = ArithUpDn.rrEffortField sampleE $ intrealeff_eRoundedReal effort
    ringEffortMult _ effort = effort
    ringEffortPow _ effort = effort

instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     NumOrd.HasExtrema e)
    =>
    RoundedRing (Interval e)


instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     NumOrd.HasExtrema e)
    =>
    RoundedFieldEffort (Interval e)
    where
    type FieldOpsEffortIndicator (Interval e) =
        IntervalRealEffort e
    fieldOpsDefaultEffort i =
        defaultIntervalRealEffort i
    fldEffortAdd (Interval sampleE _) effort =
        ArithUpDn.fldEffortAdd sampleE effortField
        where
        effortField = ArithUpDn.rrEffortField sampleE $ intrealeff_eRoundedReal effort
    fldEffortMult _ effort = effort
    fldEffortPow _ effort = effort
    fldEffortDiv _  effort = effort
         
        
instance 
    (ArithUpDn.RoundedReal e, 
     RoundedFieldEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     NumOrd.HasExtrema e)
    =>
    RoundedField (Interval e)
        