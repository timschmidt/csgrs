{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.Measures
    Description :  distance and imprecision for intervals
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Distance and imprecision for intervals.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.Measures 
()
where

import Numeric.AERN.RealArithmetic.Interval.Effort

import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Interval
--import Numeric.AERN.Basics.Consistency

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators ((<+>))

import qualified 
       Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators ((</\>))

import qualified 
       Numeric.AERN.NumericOrder as NumOrd

import qualified 
       Numeric.AERN.Basics.PartialOrdering as POrd

instance 
    (ArithUpDn.RoundedReal e,
     ArithInOut.RoundedAdd (Distance e))
    => 
    HasDistance (Interval e) 
    where
    type Distance (Interval e) = Distance e
    type DistanceEffortIndicator (Interval e) =
        IntervalRealEffort e
    distanceDefaultEffort i =
        defaultIntervalRealEffort i
    distanceBetweenEff effort (Interval l1 r1) (Interval l2 r2) =
        distL <+> distR -- addition with default effort
        where
        distL = distanceBetweenEff effGetDist l1 l2
        distR = distanceBetweenEff effGetDist r1 r2
        effGetDist = ArithUpDn.rrEffortDistance sampleE effortE
        sampleE = l1
        effortE = intrealeff_eRoundedReal effort
    
instance
    (ArithUpDn.RoundedReal e,
     RefOrd.RoundedLattice (Distance e), Neg (Distance e))
    => 
    HasImprecision (Interval e) 
    where
    type Imprecision (Interval e) = Distance e
    type ImprecisionEffortIndicator (Interval e) = 
        IntervalRealEffort e
    imprecisionDefaultEffort i =
        defaultIntervalRealEffort i 
    imprecisionOfEff effort (Interval l r) =
        case (NumOrd.pCompareEff effComp l r) of
            Just POrd.LT -> dist
            Just POrd.GT -> neg dist
            _ -> dist </\> (neg dist)
        where
        dist = distanceBetweenEff effGetDist l r
        effGetDist = ArithUpDn.rrEffortDistance sampleE effortE
        effComp = ArithUpDn.rrEffortComp sampleE effortE
        sampleE = l
        effortE = intrealeff_eRoundedReal effort
        
        
    
