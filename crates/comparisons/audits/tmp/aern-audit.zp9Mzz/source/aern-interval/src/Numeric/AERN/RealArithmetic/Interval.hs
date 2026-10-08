{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE CPP #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval
    Description :  instances of arithmetic classes for Intervals  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Instances of arithmetic classes for Intervals.
-}

module Numeric.AERN.RealArithmetic.Interval
(
    -- This module re-exports mainly type class instances;
    -- The following modules have some auxiliary functions:
    module Numeric.AERN.RealArithmetic.Interval.Effort,
    module Numeric.AERN.RealArithmetic.Interval.MixedFieldOps,
    module Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
)
where

import Numeric.AERN.RealArithmetic.Interval.ExactOps ()
import Numeric.AERN.RealArithmetic.Interval.Effort
import Numeric.AERN.RealArithmetic.Interval.Measures ()
import Numeric.AERN.RealArithmetic.Interval.UpDnConversion ()
import Numeric.AERN.RealArithmetic.Interval.Conversion ()
import Numeric.AERN.RealArithmetic.Interval.FieldOps ()
import Numeric.AERN.RealArithmetic.Interval.MixedFieldOps
import Numeric.AERN.RealArithmetic.Interval.SpecialConst ()
import Numeric.AERN.RealArithmetic.Interval.Floating ()
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified 
       Numeric.AERN.NumericOrder as NumOrd
import qualified 
       Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.ShowInternals

instance (HasLegalValues e) => HasLegalValues (Interval e) where
    maybeGetProblem (Interval l r) = 
        case (maybeGetProblem l, maybeGetProblem r) of
            (Nothing, Nothing) -> Nothing
            (Just problemDescription, _) -> 
                Just $ "in the left endpoint of an interval: " ++ problemDescription
            (_, Just problemDescription) ->  
                Just $ "in the right endpoint of an interval: " ++ problemDescription

instance
    (ShowInternals e, 
     ArithUpDn.RoundedReal e, NumOrd.HasExtrema e, 
     ArithInOut.RoundedField (Distance e), 
     Neg (Distance e),
     RefOrd.RoundedLattice (Distance e)
    ) => 
    ArithInOut.RoundedReal (Interval e)
    where
    type RoundedRealEffortIndicator (Interval e) =
        IntervalRealEffort e 
    roundedRealDefaultEffort i =
        defaultIntervalRealEffort i 
    rrEffortNumComp (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortMinmaxInOut (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortRefComp (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortPartialJoin (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortJoinMeet (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortDistance (Interval _ _) eff = eff 
--    rrEffortDistanceComp (Interval _ _) eff = intrealeff_distComp eff
--    rrEffortDistanceField (Interval _ _) eff = intrealeff_distField eff
    rrEffortImprecision (Interval _ _) eff = eff
--    rrEffortImprecisionComp (Interval _ _) eff = intrealeff_distComp eff
--    rrEffortImprecisionField (Interval _ _) eff = intrealeff_distField eff
    rrEffortToSelf _ _ = ()
    rrEffortToInt (Interval l _) eff = ArithUpDn.rrEffortToInt l $ intrealeff_eRoundedReal eff
    rrEffortFromInt (Interval l _) eff = ArithUpDn.rrEffortFromInt l $ intrealeff_eRoundedReal eff
    rrEffortToInteger (Interval l _) eff = ArithUpDn.rrEffortToInteger l  $ intrealeff_eRoundedReal eff
    rrEffortFromInteger (Interval l _) eff = ArithUpDn.rrEffortFromInteger l  $ intrealeff_eRoundedReal eff
    rrEffortToDouble (Interval l _) eff = ArithUpDn.rrEffortToDouble l  $ intrealeff_eRoundedReal eff
    rrEffortFromDouble (Interval l _) eff = ArithUpDn.rrEffortFromDouble l  $ intrealeff_eRoundedReal eff
    rrEffortToRational (Interval l _) eff = ArithUpDn.rrEffortToRational l  $ intrealeff_eRoundedReal eff
    rrEffortFromRational (Interval l _) eff = ArithUpDn.rrEffortFromRational l  $ intrealeff_eRoundedReal eff
    rrEffortAbs (Interval l _) eff = intrealeff_intordeff l eff
    rrEffortField (Interval _ _) eff = eff 
    rrEffortSelfMixedField (Interval l _) eff = (effMixed, eff)
        where
        effMixed =
            IntervalRealMixedEffort
            {
                intrealmxeff_eEffort = eff,
                intrealmxeff_mixedField = ArithUpDn.rrEffortSelfMixedField l effR,
                intrealmxeff_tnComp = ArithUpDn.rrEffortComp l effR
            }
        effR = intrealeff_eRoundedReal eff
    rrEffortIntMixedField (Interval l _) eff =
        IntervalRealMixedEffort
        {
            intrealmxeff_eEffort = eff,
            intrealmxeff_mixedField = ArithUpDn.rrEffortIntMixedField l effR,
            intrealmxeff_tnComp = ()
        }
        where
        effR = intrealeff_eRoundedReal eff
    rrEffortIntegerMixedField (Interval l _) eff =
        IntervalRealMixedEffort
        {
            intrealmxeff_eEffort = eff,
            intrealmxeff_mixedField = ArithUpDn.rrEffortIntegerMixedField l effR,
            intrealmxeff_tnComp = ()
        }
        where
        effR = intrealeff_eRoundedReal eff
    rrEffortDoubleMixedField (Interval l _) eff =
        IntervalRealMixedEffort
        {
            intrealmxeff_eEffort = eff,
            intrealmxeff_mixedField = ArithUpDn.rrEffortDoubleMixedField l effR,
            intrealmxeff_tnComp = ()
        }
        where
        effR = intrealeff_eRoundedReal eff
    rrEffortRationalMixedField (Interval l _) eff =
        IntervalRealMixedEffort
        {
            intrealmxeff_eEffort = eff,
            intrealmxeff_mixedField = ArithUpDn.rrEffortRationalMixedField l effR,
            intrealmxeff_tnComp = ()
        }
        where
        effR = intrealeff_eRoundedReal eff

