{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE StandaloneDeriving #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.Effort
    Description :  composite effort indicator for basic interval real operations 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Composite effort indicator for basic interval real operations.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.Effort 
where

import Numeric.AERN.Basics.Interval

import qualified 
       Numeric.AERN.NumericOrder as NumOrd 
--import qualified 
--       Numeric.AERN.RefinementOrder as RefOrd

--import Numeric.AERN.RealArithmetic.Measures (Distance, distanceBetween)

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
       (RoundedReal, RoundedRealEffortIndicator(..), roundedRealDefaultEffort, 
        RoundedMixedField, RoundedMixedFieldEffort, MixedFieldOpsEffortIndicator(..), mixedFieldOpsDefaultEffort)
--import qualified 
--       Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--       (RoundedFieldEffort, FieldOpsEffortIndicator, fieldOpsDefaultEffort)

import Numeric.AERN.Basics.Effort

import Test.QuickCheck (Arbitrary(..)) --, arbitrary, vectorOf)

import Control.Applicative

data IntervalRealEffort e =
    IntervalRealEffort
    {
        intrealeff_eRoundedReal :: ArithUpDn.RoundedRealEffortIndicator e
    }

instance 
    (
        Arbitrary (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    Arbitrary (IntervalRealEffort e)
    where
    arbitrary =
        IntervalRealEffort <$> arbitrary -- <*> arbitrary <*> arbitrary <*> arbitrary
        
deriving instance
    (
        Show (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    Show (IntervalRealEffort e)

instance
    (
        EffortIndicator (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    EffortIndicator (IntervalRealEffort e)
    where
    effortIncrementVariants (IntervalRealEffort e1O) =
        [IntervalRealEffort e1 | 
            (e1) <- effortIncrementVariants (e1O) ]
    effortRepeatIncrement (IntervalRealEffort i1, IntervalRealEffort j1) = 
        IntervalRealEffort 
            (effortRepeatIncrement (i1, j1)) 
    effortIncrementSequence (IntervalRealEffort e1O) =
        [IntervalRealEffort e1 | 
            (e1) <- effortIncrementSequence (e1O) ]
    effortCombine (IntervalRealEffort i1) (IntervalRealEffort j1) =
        IntervalRealEffort (effortCombine i1 j1) 

intrealeff_intordeff ::
    (ArithUpDn.RoundedReal e)
    => 
    e -> IntervalRealEffort e -> IntervalOrderEffort e
intrealeff_intordeff sampleE intrealeff =
    IntervalOrderEffort
    {
        intordeff_eComp = ArithUpDn.rrEffortComp sampleE effE,
        intordeff_eMinmax = ArithUpDn.rrEffortMinmax sampleE effE 
    }
    where
    effE = intrealeff_eRoundedReal intrealeff

intrealeff_eComp ::
    (ArithUpDn.RoundedReal e)
    => 
    e -> IntervalRealEffort e -> NumOrd.PartialCompareEffortIndicator e
intrealeff_eComp sampleE intrealeff =
    intordeff_eComp $
        intrealeff_intordeff sampleE intrealeff

intrealeff_eMinmax ::
    (ArithUpDn.RoundedReal e)
    => 
    e -> IntervalRealEffort e -> NumOrd.MinmaxEffortIndicator e
intrealeff_eMinmax sampleE intrealeff =
    intordeff_eMinmax $
        intrealeff_intordeff sampleE intrealeff

defaultIntervalRealEffort :: 
   (ArithUpDn.RoundedReal e) 
   =>
   Interval e -> IntervalRealEffort e
defaultIntervalRealEffort (Interval sampleE _) =
    IntervalRealEffort
    {
        intrealeff_eRoundedReal = ArithUpDn.roundedRealDefaultEffort sampleE
--        ,
--        intrealeff_distField = ArithInOut.fieldOpsDefaultEffort sampleDist,         
--        intrealeff_distComp = NumOrd.pCompareDefaultEffort sampleDist,
--        intrealeff_distJoinMeet = RefOrd.joinmeetDefaultEffort sampleDist
    }
--    where
--    sampleDist = distanceBetween sampleE sampleE
    
data IntervalRealMixedEffort e tn =
    IntervalRealMixedEffort
    {
        intrealmxeff_eEffort :: IntervalRealEffort e
    ,
        intrealmxeff_mixedField :: ArithUpDn.MixedFieldOpsEffortIndicator e tn
    ,
        intrealmxeff_tnComp :: NumOrd.PartialCompareEffortIndicator tn
    }

defaultIntervalRealMixedEffort :: 
   (ArithUpDn.RoundedReal e1,
    ArithUpDn.RoundedMixedFieldEffort e1 e2,
    NumOrd.PartialComparison e2) 
   =>
   Interval e1 -> e2 -> IntervalRealMixedEffort e1 e2
defaultIntervalRealMixedEffort i@(Interval sampleE1 _) sampleE2 =
    IntervalRealMixedEffort
    {
        intrealmxeff_eEffort = defaultIntervalRealEffort i
    ,
        intrealmxeff_mixedField = ArithUpDn.mixedFieldOpsDefaultEffort sampleE1 sampleE2
    ,
        intrealmxeff_tnComp = NumOrd.pCompareDefaultEffort sampleE2
    }

instance 
    (
        Arbitrary (ArithUpDn.RoundedRealEffortIndicator e),
        Arbitrary (ArithUpDn.MixedFieldOpsEffortIndicator e tn),
        Arbitrary (NumOrd.PartialCompareEffortIndicator tn)
    )
    =>
    Arbitrary (IntervalRealMixedEffort e tn)
    where
    arbitrary =
        IntervalRealMixedEffort <$> arbitrary <*> arbitrary <*> arbitrary -- <*> arbitrary
        
deriving instance
    (
        Show (ArithUpDn.RoundedRealEffortIndicator e),
        Show (ArithUpDn.MixedFieldOpsEffortIndicator e tn),
        Show (NumOrd.PartialCompareEffortIndicator tn)
    )
    =>
    Show (IntervalRealMixedEffort e tn)

instance
    (
        EffortIndicator (ArithUpDn.RoundedRealEffortIndicator e),
        EffortIndicator (ArithUpDn.MixedFieldOpsEffortIndicator e tn),
        EffortIndicator (NumOrd.PartialCompareEffortIndicator tn)
    )
    =>
    EffortIndicator (IntervalRealMixedEffort e tn)
    where
    effortIncrementVariants (IntervalRealMixedEffort e1O e2O e3O) =
        [IntervalRealMixedEffort e1 e2 e3 | 
            (e1,e2,e3) <- effortIncrementVariants (e1O,e2O,e3O) ]
    effortRepeatIncrement (IntervalRealMixedEffort i1 i2 i3, IntervalRealMixedEffort j1 j2 j3) = 
        IntervalRealMixedEffort 
            (effortRepeatIncrement (i1, j1)) 
            (effortRepeatIncrement (i2, j2)) 
            (effortRepeatIncrement (i3, j3)) 
    effortIncrementSequence (IntervalRealMixedEffort e1O e2O e3O) =
        [IntervalRealMixedEffort e1 e2 e3 | 
            (e1,e2,e3) <- effortIncrementSequence (e1O,e2O,e3O) ]
    effortCombine (IntervalRealMixedEffort i1 i2 i3) (IntervalRealMixedEffort j1 j2 j3) =
        IntervalRealMixedEffort 
            (effortCombine i1 j1) 
            (effortCombine i2 j2) 
            (effortCombine i3 j3) 

    
