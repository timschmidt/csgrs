{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Comparison
    Description :  partial comparison of functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Partial comparison of functions.
-}

module Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Comparison 
(
    pCompareFunFromRingOps
)
where

import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.InPlace.OpsImplicitEffort

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort
----import Numeric.AERN.RefinementOrder.InPlace.OpsImplicitEffort

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort
--
--import Numeric.AERN.Basics.Effort
--import Numeric.AERN.Basics.Mutable
--import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.Basics.PartialOrdering

type PartialCompareEffortIndicatorFromRingOps f =
    (Int, -- number of samples to evaluate in the search for negative information
     ArithInOut.AddEffortIndicator f,
     NumOrd.PartialCompareEffortIndicator (Domain f),
     EvaluationEffortIndicator f) 

pCompareFunFromRingOps ::
    (Show (Domain f),
     NumOrd.PartialComparison (Domain f),
     HasZero (Domain f),
     CanEvaluate f,
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSubtr f) 
    =>
    PartialCompareEffortIndicatorFromRingOps f ->
    f ->
    f ->
    PartialOrderingPartialInfo
pCompareFunFromRingOps (n, effAdd, effCompDom, effEval) f1 f2 =
    addEQ $ addGEQ $ addGT $ addLEQ $ addLT $ addNC $ 
        infoRanges
    where
    sampleF = f1
    dombox = getDomainBox sampleF 
    sampleDom = getSampleDomValue sampleF

    infoRanges = compareOver dombox
    compareOver db =
        NumOrd.pCompareInFullEff effCompDom diffRange (zero sampleDom)
        where
        diffRange = evalAtPointOutEff effEval db diff
        diff = ArithInOut.subtrOutEff effAdd f1 f2
    addEQ pInfo
        | (pOrdInfEQ pInfo == Nothing) && neqD = pInfo { pOrdInfEQ = Just False }
        | otherwise = pInfo
    addNC pInfo
        | nleqD && ngeqD = pInfo { pOrdInfNC = Just True }
        | (pOrdInfLEQ pInfo /= Just True) && (pOrdInfGEQ pInfo /= Just True) = pInfo { pOrdInfNC = Nothing }
        | otherwise = pInfo
    addLT pInfo
        | (pOrdInfLT pInfo == Nothing) && nltD = pInfo { pOrdInfLT = Just False }
        | otherwise = pInfo
    addLEQ pInfo
        | (pOrdInfLEQ pInfo == Nothing) && nleqD = pInfo { pOrdInfLEQ = Just False }
        | otherwise = pInfo
    addGT pInfo
        | (pOrdInfGT pInfo == Nothing) && ngtD = pInfo { pOrdInfGT = Just False }
        | otherwise = pInfo
    addGEQ pInfo
        | (pOrdInfGEQ pInfo == Nothing) && ngeqD = pInfo { pOrdInfGEQ = Just False }
        | otherwise = pInfo
    neqD = hasFalse $ map pOrdInfEQ domPointsInfos
    nleqD = hasFalse $ map pOrdInfLEQ domPointsInfos
    nltD = hasFalse $ map pOrdInfLT domPointsInfos
    ngeqD = hasFalse $ map pOrdInfGEQ domPointsInfos
    ngtD = hasFalse $ map pOrdInfGT domPointsInfos
    domPointsInfos =
        map compareOver domPoints
    domPoints = getNSamplesFromInsideDomainBox sampleF dombox n

hasFalse :: [(Maybe Bool)] -> Bool
hasFalse (Just False : _) = True
hasFalse (_ : rest) = hasFalse rest
hasFalse [] = False
