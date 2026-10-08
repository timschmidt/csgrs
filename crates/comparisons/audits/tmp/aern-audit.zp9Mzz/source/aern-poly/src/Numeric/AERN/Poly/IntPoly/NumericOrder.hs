{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.NumericOrder
    Description :  pointwise up/down comparison
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Pointwise up/down comparison of interval polynomials.
-}

module Numeric.AERN.Poly.IntPoly.NumericOrder
where
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.Evaluation ()
--import Numeric.AERN.Poly.IntPoly.Addition 
import Numeric.AERN.Poly.IntPoly.Multiplication ()

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Comparison
import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Arbitrary
--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Minmax

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

--import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder 
    (PartialCompareEffortIndicator) -- needed for ghc 6.12
import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

--import Numeric.AERN.Basics.Interval (refordPCompareInFullIntervalsEff)

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary

import Test.QuickCheck

import Debug.Trace
_ = trace

instance
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf), 
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     Arbitrary cf,
     NumOrd.PartialComparison (Imprecision cf), 
     RefOrd.IntervalLike cf) 
    => 
    NumOrd.PartialComparison (IntPoly var cf) 
    where
    type PartialCompareEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    pCompareDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg
    pCompareEff eff p1 p2 =
        case partialInfo2PartialOrdering $ NumOrd.pCompareInFullEff eff p1 p2 of
            [rel] -> Just rel
            _ -> Nothing
    pCompareInFullEff eff p1 p2 = 
        pCompareFunFromRingOps (n, eff, effCompDom, effEval) p1 p2 
        where
        n = ipolyeff_counterExampleSearchSampleCount eff
        effEval = eff
        effDom = ipolyeff_cfRoundedRealEffort eff
        effCompDom = ArithInOut.rrEffortNumComp sampleDom effDom
        sampleDom = getSampleDomValue p1

instance
    (Ord var, Show var, Show cf,
     NumOrd.HasGreatest cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf
    )
    =>
    NumOrd.HasGreatest (IntPoly var cf)
    where
    greatest sampleF = 
        newConstFnFromSample sampleF (NumOrd.greatest sampleDom)
        where
        sampleDom = getSampleDomValue sampleF
        
instance
    (Ord var, Show var, Show cf,
     NumOrd.HasLeast cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf
    )
    =>
    NumOrd.HasLeast (IntPoly var cf)
    where
    least sampleF = 
        newConstFnFromSample sampleF (NumOrd.least sampleDom)
        where
        sampleDom = getSampleDomValue sampleF
        
instance
    (Ord var, Show var, Show cf,
     NumOrd.HasExtrema cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf
    )
    =>
    NumOrd.HasExtrema (IntPoly var cf)

        
instance
    (Show var, Ord var,
     Show (Interval e), Show (SizeLimits (Interval e)), 
     RefOrd.ArbitraryOrderedTuple (Interval e), RefOrd.IntervalLike (Interval e),
     ArithInOut.RoundedReal (Interval e),
     HasAntiConsistency (Interval e),
     ArithInOut.RoundedMixedMultiply (Interval e) (Interval e),
     ArithInOut.RoundedMixedAdd (Interval e) (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     Arbitrary (Interval e),
     Arbitrary (RefOrd.GetEndpointsEffortIndicator (Interval e)),
     Arbitrary (ArithInOut.MixedMultEffortIndicator (Interval e) (Interval e)),
     Arbitrary (ArithInOut.MixedAddEffortIndicator (Interval e) (Interval e)),
     Arbitrary (ArithInOut.RoundedRealEffortIndicator (Interval e)),
     Arbitrary (ArithInOut.AddEffortIndicator (Interval e))
    )
    =>
    ArbitraryWithArea (IntPoly var (Interval e))
    where
    type Area (IntPoly var (Interval e)) = Area4FunFromRingOps (IntPoly var (Interval e))
    areaWhole sampleF = areaWhole4FunFromRingOps sampleF
    arbitraryInArea area@(sampleFn,_) =
        do
        eff <- arbitrary
        poly <- withEff eff
        isThin <- arbitrary
        case isThin of
            True -> return $ fst $ RefOrd.getEndpointsOut poly
            False -> return poly
        where
        withEff eff =
            arbitraryInArea4FunFromRingOps eff
                (fnSequence fixedRandSeqQuantityOfSize sampleFn) 
                fixedRandSeqQuantityOfSize 
                area
        fixedRandSeqQuantityOfSize :: Int -> Int
        fixedRandSeqQuantityOfSize size =
--            trace
--            (
--                "fixedRandSeqQuantityOfSize " ++ show size ++ " = " ++ show result
--            )
            result
            where
            result =
                10 + ((size*(size+100)) `div` 10)  
--                10 + (3*size)

instance
    AreaHasForbiddenValues (IntPoly var (Interval e))
    where
    areaGetForbiddenValues _ = []
    areaAddForbiddenValues _ _ = error "Adding forbidden values to Area (IntPoly var (Interval e)) currently not supported."

instance
    (Show var, Ord var,
     Show (Interval e), Show (SizeLimits (Interval e)),
     RefOrd.ArbitraryOrderedTuple (Interval e), RefOrd.IntervalLike (Interval e),
     ArithInOut.RoundedReal (Interval e),
     HasAntiConsistency (Interval e),
     ArithInOut.RoundedMixedMultiply (Interval e) (Interval e),
     ArithInOut.RoundedMixedAdd (Interval e) (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     Arbitrary (Interval e),
     Arbitrary (RefOrd.GetEndpointsEffortIndicator (Interval e)),
     Arbitrary (ArithInOut.MixedMultEffortIndicator (Interval e) (Interval e)),
     Arbitrary (ArithInOut.MixedAddEffortIndicator (Interval e) (Interval e)),
     Arbitrary (ArithInOut.RoundedRealEffortIndicator (Interval e)),
     Arbitrary (ArithInOut.AddEffortIndicator (Interval e))
    )
    =>
    NumOrd.ArbitraryOrderedTuple (IntPoly var (Interval e))
    where
    arbitraryTupleRelatedBy = 
        error "AERN internal error: NumOrd.arbitraryTupleRelatedBy not defined for IntPoly"
    arbitraryTupleInAreaRelatedBy area@(sampleFn,_) indices rels =
        case 
            arbitraryTupleInAreaRelatedBy4FunFromRingOps 
                dummyEff 
                (fnSequence fixedRandSeqQuantityOfSize sampleFn) 
                fixedRandSeqQuantityOfSize 
                area indices rels of
            Nothing -> Nothing
            Just _ -> Just $
                do
                eff <- arbitrary
                case
                    arbitraryTupleInAreaRelatedBy4FunFromRingOps 
                        eff 
                        (fnSequence fixedRandSeqQuantityOfSize sampleFn) 
                        fixedRandSeqQuantityOfSize 
                        area indices rels of
                    Just gen -> gen
                    _ -> error "IntPoly.NumericOrder: arbitraryTupleInAreaRelatedBy: internal error"
        where
        fixedRandSeqQuantityOfSize :: Int -> Int
        fixedRandSeqQuantityOfSize size
            = 10 + ((size*(size+100)) `div` 10)  
--            = 10 + (3*size)
        dummyEff = 
            ((e,e,e),
             (e,e),
             (e,e)
            )
        e :: t
        e = error "AERN internal error: arbitraryTupleInAreaRelatedBy4FunFromRingOps: dummyEff should not be used"

        