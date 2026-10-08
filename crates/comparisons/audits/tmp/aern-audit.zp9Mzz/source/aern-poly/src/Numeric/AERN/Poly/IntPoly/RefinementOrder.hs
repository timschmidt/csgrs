{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.RefinementOrder
    Description :  interval polynomial as a polynomial interval
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Interval polynomial as a polynomial interval.
-}

module Numeric.AERN.Poly.IntPoly.RefinementOrder
where

import Prelude hiding (EQ,LT,GT)
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.Evaluation ()
--import Numeric.AERN.Poly.IntPoly.Addition 
import Numeric.AERN.Poly.IntPoly.Multiplication ()
import Numeric.AERN.Poly.IntPoly.NumericOrder ()

--import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.Evaluation

--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Comparison
--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Arbitrary
--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Minmax

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Basics.Interval (refordPCompareInFullIntervalsEff)

import Numeric.AERN.Basics.PartialOrdering
--import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

--import Numeric.AERN.Misc.Debug

import Test.QuickCheck (Arbitrary)

instance
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     Arbitrary cf,
     NumOrd.PartialComparison (Imprecision cf), 
     RefOrd.IntervalLike cf) 
    => 
    RefOrd.PartialComparison (IntPoly var cf) 
    where
    type PartialCompareEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    pCompareDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg
    pCompareEff eff p1 p2 =
        case partialInfo2PartialOrdering $ RefOrd.pCompareInFullEff eff p1 p2 of
            [rel] -> Just rel
            _ -> Nothing
    pCompareInFullEff eff p1 p2 = 
        refordPCompareInFullIntervalsEff eff p1Endpoints p2Endpoints  
        where
        p1Endpoints = polyGetEndpointsOutEff effGetE p1
        p2Endpoints = polyGetEndpointsOutEff effGetE p2
        effGetE = ipolyeff_cfGetEndpointsEffort eff
        


instance 
    (Ord var, Show var, 
     Show cf, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf, HasZero cf,
     HasConsistency cf, 
     NumOrd.PartialComparison (IntPoly var cf)
    )
    => 
    (HasConsistency (IntPoly var cf))
    where
    type ConsistencyEffortIndicator (IntPoly var cf) =
        (RefOrd.GetEndpointsEffortIndicator cf,
         NumOrd.PartialCompareEffortIndicator (IntPoly var cf))
    consistencyDefaultEffort p =
        (RefOrd.getEndpointsDefaultEffort sampleCf,
         NumOrd.pCompareDefaultEffort p)
        where
        sampleCf = getSampleDomValue p
    getConsistencyEff (effGetE, effComp) p =
-- TODO: add detection of inconsistency, needs using pCompareInFullEff instead of pCompareEff
        case NumOrd.pCompareEff effComp pL pR of
            Just EQ -> Just Exact
            Just LT -> Just Consistent
            Just GT -> Just Anticonsistent
            _ -> Nothing
        where
        (pL, pR) = RefOrd.getEndpointsOutEff effGetE p
    
instance 
    (Ord var, Show var, 
     Show cf, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf, HasZero cf, 
     NumOrd.PartialComparison (IntPoly var cf),
     HasAntiConsistency cf
    )
    => 
    (HasAntiConsistency (IntPoly var cf))
    where
    flipConsistency = flipConsistencyPoly
    