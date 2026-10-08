{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
--{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Bisection
    Description :  adaptive splitting solver parametrised by a single-step solver  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Adaptive splitting solver parametrised by a single-step solver.
    
    Typically one uses solveODEIVPByBisectingT0 and
    its subsolver is defined using solveODEIVPByBisectingAtT0End
    and its subsolver for the right-hand side is 
    defined using solveODEIVPByBisectingT:
    
    solveODEIVPByBisectingT0
    |
    |
    solveODEIVPByBisectingAtT0End
    |       \
    |        \
    solve-VT  solveODEIVPByBisectingT
              |
              |
              solve-Vt
-}

module Numeric.AERN.IVP.Solver.Bisection
(
    solveHybridIVPByBisectingT,
    solveODEIVPByBisectingAtT0End,
    solveODEIVPByBisectingT,
    solveODEIVPByBisectingT0,
    BisectionInfo(..),
    showBisectionInfo,
    bisectionInfoMapSegments,
    bisectionInfoCountLeafs,
    bisectionInfoGetLeafSegInfoSequence,
    bisectionInfoCheckCondition,
    bisectionInfoEvalFn,
    bisectionInfoTrimAt
)
where

import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Specification.Hybrid

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Consistency

--import Numeric.AERN.Basics.Exception
--import Control.Exception (throw)

--import qualified Data.Map as Map


import Data.Maybe (isJust)

import Debug.Trace
_ = trace
        
solveHybridIVPByBisectingT ::
    (CanAddVariables f,
     CanEvaluate f,
     CanCompose f,
     HasProjections f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike(Domain f),
     HasAntiConsistency (Domain f),
     Domain f ~ Imprecision (Domain f),
     Show f, Show (Domain f))
    =>
    (Int -> HybridIVP f -> (Maybe (HybridSystemUncertainState (Domain f)), solvingInfo)) -- ^ solver to use for segments  
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    Imprecision (Domain f) -- ^ splitting improvement threshold
    ->
    Domain f -- ^ minimum segment length  
    ->
    Domain f -- ^ maximum segment length  
    ->
    (HybridIVP f)  -- ^ problem to solve
    ->
    (
        Maybe (HybridSystemUncertainState (Domain f))
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe (Imprecision (Domain f)))
        )
    )
solveHybridIVPByBisectingT
        solver
            effDom splitImprovementThreshold minStepSize maxStepSize 
                hybivpG 
    =
    result
    where
    result = splitSolve 0 hybivpG
    
    splitSolve depth hybivp =
--        trace
--        (
--            "solveHybridIVPByBisectingT: splitSolve: "
--            ++ "tStart = " ++ show tStart
--            ++ "tEnd = " ++ show tEnd
--        ) $
        result2
        where
        result2
            | onMinStepSize = directComputation
            | aboveMaxStepSize = splitComputation
            | directComputationFailed = splitComputation
            | otherwise = 
                case maybeSplitImprovement of
                    Just improvementBy 
                        | (improvementBy >? splitImprovementThreshold) /= Just True -> 
                            directComputation -- split once computations succeeded but brought no noticeable improvement
                    _
                        | splitComputationFailed -> directComputation
                        | otherwise -> splitComputation -- splitting either brought noticeable improvement or some computation failed 
        tStart = hybivp_tStart hybivp
        tEnd = hybivp_tEnd hybivp
        
        onMinStepSize =
            ((tEnd .<->. tStart) >? minStepSize) /= Just True
        aboveMaxStepSize =
            ((tEnd .<->. tStart) <=? maxStepSize) /= Just True

        directComputation =
--            trace
--            (
--                "solveHybridIVPByBisectingT: completed time " ++ show tEnd
--            ) $
            case maybeDirectResult of
                Just resultOut 
                    | otherwise -> (Just resultOut, BisectionNoSplit directInfo)
                _ -> (Nothing, BisectionNoSplit directInfo) 
        (maybeDirectResult, directInfo) = solver depth hybivp
        directComputationFailed =
            case maybeDirectResult of Just _ -> False; _ -> True
        
        splitOnceComputation = -- needed only to decide whether splitting is benefitial, the result is then discarded
            case solver (depth + 1) hybivpL of
                (Just midState, _) ->
                    case solver (depth + 1) hybivpR of
                        (Just endStateOut, _) -> Just endStateOut 
                        _ -> Nothing
                    where
                    hybivpR =
                        hybivp
                        {
                            hybivp_tStart = tMid,
                            hybivp_initialStateEnclosure = midState
                        }
                _ -> Nothing
                
        (splitComputation, splitComputationFailed) =
            (
                (maybeState, BisectionSplit (directInfo, maybeSplitImprovement) infoL maybeInfoR)
            , 
                case maybeState of Just _ -> False; _ -> True
            )
            where
            (maybeMidState, infoL) =
                splitSolve (depth + 1) hybivpL
            (maybeState, maybeInfoR) =
                case maybeMidState of
                    Just midState ->
                        case splitSolve (depth + 1) hybivpR of
                            (maybeState2, infoR) -> (maybeState2, Just infoR)
                        where
                        hybivpR =
                            hybivp
                            {
                                hybivp_tStart = tMid,
                                hybivp_initialStateEnclosure = midState
                            }
                    Nothing -> (Nothing, Nothing)
        hybivpL =
            hybivp
            {
                hybivp_tEnd = tMid
            }
        tMid =
            getMidPoint effAddDom effDivDomInt tStart tEnd 
        
        maybeSplitImprovement =
            case (maybeDirectResult, splitOnceComputation) of
                (Just directResult, Just splitOnceResult) -> 
                    Just $ measureImprovementState sampleDom effDom directResult splitOnceResult
                _ -> Nothing

    (.<->.) = ArithInOut.subtrOutEff effAddDom

    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
--    effRefComp = ArithInOut.rrEffortRefComp sampleDom effDom
    sampleDom = hybivp_tStart hybivpG
--    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDom effDom
    
--    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
--    sampleImpr = imprecisionOfEff effImpr sampleDom
--    effAddImpr = ArithInOut.fldEffortAdd sampleImpr $ ArithInOut.rrEffortImprecisionField sampleDom effDom
        
solveODEIVPByBisectingAtT0End ::
    (HasAntiConsistency (Domain f), 
     HasAntiConsistency f,
     Show f, Show (Domain f))
    =>
    (ODEIVP f -> (Maybe [f], solvingInfoL)) -- ^ uncertain time solver; giving parametrised results
    -> 
    ([f] -> ODEInitialValues f) -- ^ make ODE IVP initial value specification from parametrised initial values
    -> 
    (ODEIVP f -> (Maybe [Domain f], solvingInfoR)) -- ^ exact time solver; giving wrapped results
    -> 
    ODEIVP f
    -> 
    (
        Maybe [Domain f]
    , 
        (solvingInfoL, Maybe solvingInfoR)
    )
solveODEIVPByBisectingAtT0End
        solverVT makeMakeParamInitValFnVec solverVt 
            odeivpG 
    =
    (maybeResult, (infoL, maybeInfoR))
    where
    (maybeResult, maybeInfoR) =
        case maybeResultL of
            Just fnVecLOut ->
                case maybeResultROut of
                    Just resultOut ->
--                        trace
--                        (
--                            "solveODEIVPByBisectingAtT0End:"
--                            ++ "\n fnVecLOut = " ++ show fnVecLOut
--                        )
                        (Just result, Just infoROut)
                        where
                        result = resultOut 
                    Nothing -> (Nothing, Just infoROut)
                where
                (maybeResultROut, infoROut) = solverVt odeivpROut
                odeivpROut = odeivpR fnVecLOut
                odeivpR fnVecL =
                    odeivpG
                        {
                            odeivp_tStart = t0End
                        ,
                            odeivp_makeInitialValueFnVec =
                                makeMakeParamInitValFnVec fnVecL
                        }
            _ -> 
                (Nothing, Nothing)
    (maybeResultL, infoL) = solverVT odeivpL
    odeivpL =
        odeivpG
            {
                odeivp_tEnd = t0End
            }
    t0End = odeivp_t0End odeivpG
    
solveODEIVPByBisectingT ::
    (CanAddVariables f,
     CanEvaluate f,
     CanCompose f,
     HasProjections f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike(Domain f),
     HasAntiConsistency (Domain f),
     Domain f ~ Imprecision (Domain f),
     Show f, Show (Domain f), Show result,
     solvingInfo ~ (Maybe result, additionalInfo))
    =>
    (ODEIVP f -> solvingInfo) -- ^ solver to use for segments  
    ->
    (result -> Domain f) -- ^ measure imprecision
    ->
    (result -> ODEInitialValues f) -- ^ how to change initial conditions
    ->
    (result -> Maybe result)
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    (Domain f) -- ^ splitting improvement threshold
    ->
    Domain f -- ^ minimum segment length  
    ->
    Domain f -- ^ maximum segment length  
    ->
    (ODEIVP f)  -- ^ problem to solve
    ->
    (
        (Maybe result,
         Bool -- if the result is Nothing, is it because the solution strayed outside the domain?
        )
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe (Imprecision (Domain f)))
        )
    )
solveODEIVPByBisectingT
        solver measureResultImprecision makeMakeInitValFnVec intersectDomain
            effDom splitImprovementThreshold minStepSize maxStepSize
                odeivpG 
    =
    result
    where
    result = splitSolve odeivpG Nothing

    (.<->.) = ArithInOut.subtrOutEff effAddDom
    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    sampleDom = odeivp_tStart odeivpG

--    effRefComp = ArithInOut.rrEffortRefComp sampleDom effDom
--    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDom effDom
--    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
--    sampleImpr = imprecisionOfEff effImpr sampleDom
--    effAddImpr = ArithInOut.fldEffortAdd sampleImpr $ ArithInOut.rrEffortImprecisionField sampleDom effDom
    
    splitSolve odeivp maybeDirectInfoPre =
--        trace
--        (
--            "solveODEIVPByBisectingT: splitSolve: "
--            ++ "\n tStart = " ++ show tStart
--            ++ "\n tEnd = " ++ show tEnd
----            ++ "\n maybeSplitImprovement = " ++ show maybeSplitImprovement
--            ++ "\n maybeDirectResult = " ++ show maybeDirectResult
--        ) $
        result2
        where
        result2
            | onMinStepSize = directComputation
            | aboveMaxStepSize = splitComputation
            | directComputationFailed = splitComputation
            | otherwise = 
                case maybeSplitImprovement of
                    Just improvementBy 
                        | (improvementBy >? splitImprovementThreshold) /= Just True -> 
                            directComputation -- split once computations succeeded but brought no noticeable improvement
                    _
                        | midStateIsOutsideDomain -> splitComputation
                        | splitComputationFailed -> directComputation
                        | otherwise -> splitComputation -- splitting either brought noticeable improvement or some computation failed 
        tStart = odeivp_tStart odeivp
        tEnd = odeivp_tEnd odeivp
        
        onMinStepSize =
            ((tEnd .<->. tStart) >? minStepSize) /= Just True
        aboveMaxStepSize =
            ((tEnd .<->. tStart) <=? maxStepSize) /= Just True

        (directComputation, directInfo, directComputationFailed) =
            case maybeDirectResult of
                Just resultOut -> (((Just resultOut, False), BisectionNoSplit directInfo2), directInfo2, False)
                _ -> ((((Nothing, False), BisectionNoSplit directInfo2)), directInfo2, True)
            where 
            (maybeDirectResult, _) = directInfo2
            directInfo2 =
                case maybeDirectInfoPre of
                    Just directInfoPre -> directInfoPre
                    _ -> solver odeivp -- no pre-computed results, we need to run the solver now
        
        (splitOnceComputation, infoL1, maybeInfoR1) = 
            -- This is needed mainly to decide whether splitting is benefitial, 
            -- the result is discarded if splitting does not seem benefitial
            -- or if more than one level of splitting is eventually used.
            case solver odeivpL of
                infoL@(Just midValuesOut, _) ->
                    case solver odeivpR of
                        infoR@(Just endValuesOut, _) -> (Just endValuesOut, infoL, Just infoR) 
                        infoR -> (Nothing, infoL, Just infoR)
                    where
                    midValues = midValuesOut
                    odeivpR =
                        odeivp
                        {
                            odeivp_tStart = tMid,
                            odeivp_t0End = tMid, -- exact initial time
                            odeivp_makeInitialValueFnVec =
                                makeMakeInitValFnVec midValues 
                        }
                infoL -> (Nothing, infoL, Nothing)
                
        splitComputationFailed =
            case splitComputation of
                ((Just _, _), _) -> False
                _ -> True
        ((_, midStateIsOutsideDomain), _) = splitComputation 
        splitComputation =
            ((maybeState, midStateIsOutsideDomain), BisectionSplit (directInfo, maybeSplitImprovement) infoL maybeInfoR)
            where
            ((maybeMidState, midStateIsOutsideDomainL), infoL) =
                splitSolve odeivpL (Just infoL1) -- avoid re-running the solver for the left half-segment
            ((maybeState, maybeInfoR), midStateIsOutsideDomain) =
                case maybeMidState of
                    Just midState ->
                        case intersectDomain midState of
                            Just midState2 ->
                                case splitSolve odeivpR ifLisL1maybeInfoR1 of
                                    ((maybeState2, midStateIsOutsideDomain2), infoR) -> 
                                        ((maybeState2, Just infoR), midStateIsOutsideDomain2)
                                where
                                ifLisL1maybeInfoR1 =
                                    case infoL of
                                        (BisectionNoSplit _) -> maybeInfoR1 
                                            -- We can reuse a pre-computed solution for the right half-segment because
                                            -- the chosen computation on the left segment did not split any further
                                            -- and is thus equivalent to the preliminary split-once computation above.  
                                        _ -> Nothing
                                odeivpR =
                                    odeivp
                                    {
                                        odeivp_tStart = tMid,
                                        odeivp_t0End = tMid, -- exact initial time
                                        odeivp_makeInitialValueFnVec =
                                            makeMakeInitValFnVec  midState2
                                    }
                            Nothing -> ((Nothing, Nothing), True)
                    Nothing -> ((Nothing, Nothing), midStateIsOutsideDomainL)
        odeivpL =
            odeivp
            {
                odeivp_tEnd = tMid
            }
        tMid =
            getMidPoint effAddDom effDivDomInt tStart tEnd 
        
        maybeSplitImprovement =
            case (directComputation, splitOnceComputation) of
                (((Just directResult, _), _), Just splitOnceResult) -> 
                        measureImprovementVec directResult splitOnceResult
                _ -> Nothing

        measureImprovementVec res1 res2 =
--            trace
--            (
--                "solveODEIVPByBisectingT: measureImprovementVec: "
--                ++ "\n  imprecision1 = " ++ show imprecision1 
--                ++ "\n  imprecision2 = " ++ show imprecision2 
--            ) $
            Just $ imprecision1 .<->. imprecision2
            where
            imprecision1 = measureResultImprecision res1
            imprecision2 = measureResultImprecision res2
--        measureImprovementVec vec1 vec2 = 
--            do
--            improvements <- sequence $ 
--                                map measureImprovement $ 
--                                    zip vec1 vec2
--            Just $ foldl1 (NumOrd.minOutEff effMinmax) improvements
--        measureImprovement (encl1, encl2) =
--            let ?addInOutEffort = effAddDom in
--            let ?pCompareEffort = effRefComp in
--            do
----            refines <- encl1 |<=? encl2
----            case refines of
----                True -> 
--                    Just $ (imprecisionOfEff effImpr encl1) <-> (imprecisionOfEff effImpr encl2)
----                False -> Nothing 
                
solveODEIVPByBisectingT0 ::
    (CanAddVariables f,
     CanEvaluate f,
     CanCompose f,
     HasProjections f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike(Domain f),
     Domain f ~ Imprecision (Domain f),
     solvingInfoSub ~ ((Domain f, Maybe [Domain f]), solvingInfo),
     Show f, Show (Domain f))
    =>
    (ODEIVP f -> (Maybe [Domain f], solvingInfo)) -- ^ solver to use for segments  
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    Imprecision (Domain f) -- ^ splitting improvement threshold
    ->
    Domain f -- ^ minimum segment length  
    ->
    (ODEIVP f)  -- ^ problem to solve
    ->
    (
        Maybe [Domain f]
    , 
        BisectionInfo solvingInfoSub (solvingInfoSub, Maybe (Imprecision (Domain f)))
    )
solveODEIVPByBisectingT0
        solver
            effDom splitImprovementThreshold minStepSize 
                odeivpG 
    =
    splitSolve odeivpG
    where
    (</\>) = RefOrd.meetOutEff effJoinDom
    (.<->.) = ArithInOut.subtrOutEff effAddDom

    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    effRefComp = ArithInOut.rrEffortRefComp sampleDom effDom
    sampleDom = odeivp_tStart odeivpG
    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDom effDom
    effJoinDom = ArithInOut.rrEffortJoinMeet sampleDom effDom
    
    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
--    sampleImpr = imprecisionOfEff effImpr sampleDom
--    effAddImpr = ArithInOut.fldEffortAdd sampleImpr $ ArithInOut.rrEffortImprecisionField sampleDom effDom
    
    splitSolve odeivp
        | onMinStepSize = directComputation
        | directComputationFailed = splitComputation
        | otherwise = 
            case maybeSplitImprovement of
                Just improvementBy 
                    | (improvementBy >? splitImprovementThreshold) == Just True -> 
                        splitComputation
                _ -> directComputation
        where
        tStart = odeivp_tStart odeivp
        t0End = odeivp_t0End odeivp
        tEnd = odeivp_tEnd odeivp
        
        onMinStepSize =
            ((t0End .<->. tStart) >? minStepSize) /= Just True

        directComputation =
            (maybeDirectResult, BisectionNoSplit ((tEnd, maybeDirectResult), directInfo))
        (maybeDirectResult, directInfo) = solver odeivp
        directComputationFailed =
            case maybeDirectResult of Just _ -> False; _ -> True
        
        splitOnceComputation = -- needed only to decide whether splitting is benefitial, the result is then discarded
            case solver odeivpL of
                (Just endValuesLOut, _) -> 
                    case solver odeivpR of
                        (Just endValuesROut, _) ->
                            Just endValuesOut
                            where
                            endValuesOut =
                                zipWith (</\>) endValuesLOut endValuesROut
                        _ -> Nothing
                    where
                    odeivpR =
                        odeivp
                        {
                            odeivp_tStart = t0Mid
                        }
                _ -> Nothing

        splitComputation =
            case splitSolve odeivpL of
                (Just endValuesLOut, infoL) -> 
                    case splitSolve odeivpR of
                        (Just endValuesROut, infoR) ->
                            (Just endValuesOut, BisectionSplit (((tEnd, maybeDirectResult), directInfo), maybeSplitImprovement) infoL (Just infoR))
                            where
                            endValuesOut =
                                zipWith (</\>) endValuesLOut endValuesROut
                        (Nothing, infoR) ->
                            (Nothing, BisectionSplit (((tEnd, maybeDirectResult), directInfo), maybeSplitImprovement) infoL (Just infoR))
                    where
                    odeivpR =
                        odeivp
                        {
                            odeivp_tStart = t0Mid 
                        }
                failedLeftComputation -> failedLeftComputation

        odeivpL =
            odeivp
            {
                odeivp_t0End = t0Mid
            }
        t0Mid =
            getMidPoint effAddDom effDivDomInt tStart t0End 
        
        maybeSplitImprovement =
            case (directComputation, splitOnceComputation) of
                ((Just directResult, _), Just splitOnceResult) ->
--            case (directComputation, splitComputation) of
--                ((Just (directResult, _), _), (Just (splitOnceResult, _), _)) ->
                    measureImprovementVec directResult splitOnceResult
                _ -> Nothing
        measureImprovementVec vec1 vec2 =
            do
            improvements <- sequence $ zipWith measureImprovement vec1 vec2
            Just $ foldl1 (NumOrd.minOutEff effMinmax) improvements
        measureImprovement encl1 encl2 =
            do
--            refines <- encl1 |<=? encl2
--            case refines of
--                True -> 
                    Just $ (imprecisionOfEff effImpr encl1) .<->. (imprecisionOfEff effImpr encl2)
--                False -> Nothing 

data BisectionInfo segInfo splitReason
    = BisectionNoSplit segInfo
    | BisectionSplit 
        splitReason 
        (BisectionInfo segInfo splitReason) 
        (Maybe (BisectionInfo segInfo splitReason))

showBisectionInfo :: 
    (String -> segInfo -> String) 
    -> 
    (String -> splitReason -> String) 
    -> 
    String -- ^ indentation to add to all lines 
    -> 
    BisectionInfo segInfo splitReason 
    -> 
    String
showBisectionInfo showSegInfo showSplitReason indentG bisectionInfoG =
    shLevel indentG bisectionInfoG
    where
    shLevel indent bisectionInfo =
        case bisectionInfo of
            BisectionNoSplit segInfo -> showSegInfo indent segInfo
            BisectionSplit reason infoL Nothing ->
                (showSplitReason indent reason)
                ++ "\n" ++
                (shLevel (indent ++ "| ") infoL)
            BisectionSplit reason infoL (Just infoR) ->
                (showSplitReason indent reason)
                ++ "\n" ++
                (shLevel (indent ++ "| ") infoL)
                ++ "\n" ++
                (shLevel (indent ++ "  ") infoR)
             
bisectionInfoMapSegments :: 
    (segInfo1 -> segInfo2) ->
    BisectionInfo segInfo1 splitReason ->
    BisectionInfo segInfo2 splitReason
bisectionInfoMapSegments segFn bisectionInfo =
    aux bisectionInfo
    where
    aux (BisectionNoSplit segInfo) = BisectionNoSplit $ segFn segInfo
    aux (BisectionSplit splitReason left maybeRight) =
        BisectionSplit splitReason (aux left) (fmap aux maybeRight)

bisectionInfoCountLeafs ::
    BisectionInfo segInfo splitReason -> Int
bisectionInfoCountLeafs (BisectionNoSplit _) = 1
bisectionInfoCountLeafs (BisectionSplit _ left Nothing) =
    bisectionInfoCountLeafs left
bisectionInfoCountLeafs (BisectionSplit _ left (Just right)) =
    bisectionInfoCountLeafs left + bisectionInfoCountLeafs right
    
    
bisectionInfoGetLeafSegInfoSequence ::
    BisectionInfo segInfo splitReason -> [segInfo]
bisectionInfoGetLeafSegInfoSequence (BisectionNoSplit info) = [info]
bisectionInfoGetLeafSegInfoSequence (BisectionSplit _ left Nothing) =
    bisectionInfoGetLeafSegInfoSequence left
bisectionInfoGetLeafSegInfoSequence (BisectionSplit _ left (Just right)) =
    bisectionInfoGetLeafSegInfoSequence left
    ++
    bisectionInfoGetLeafSegInfoSequence right

bisectionInfoCheckCondition ::
    (ArithInOut.RoundedReal dom,
     RefOrd.IntervalLike dom) 
    => 
    ArithInOut.RoundedRealEffortIndicator dom ->
    (segInfo -> Maybe Bool) {-^ the condition to check -} -> 
    BisectionInfo segInfo (segInfo, otherInfo) {-^ bisected function  -} -> 
    (dom, dom) {-^ the domain of the function encoded by the above bisection -} -> 
    dom -> 
    Maybe Bool
bisectionInfoCheckCondition effDom condition bisectionInfo bisectionDom dom =
    aux bisectionDom bisectionInfo
    where
    aux _ (BisectionNoSplit info) = condition info
    aux (dL, dR) (BisectionSplit (info, _) left maybeRight) 
        | isJust resultUsingInfo = resultUsingInfo
        | domNotInR = auxLeft
        | domNotInL = auxRight
        | domInsideBothLR = 
            case auxLeft of
                Just _ -> auxLeft
                _ -> auxRight
        | otherwise = -- domSplitBetweenLR =
            case (auxLeft, auxRight) of
                (Just False, Just False) -> Just False
                (Just True , Just True ) -> Just True
                _ -> Nothing
        where
        resultUsingInfo = condition info
        domNotInL =
            (dM <? dom) == Just True
        domNotInR =
            (dom <? dM) == Just True
        domInsideBothLR = 
            (dom ==? dM) == Just True
        dM = 
            getMidPoint effAddDom effDivDomInt dL dR 
        auxLeft = aux (dL, dM) left
        auxRight = 
            case maybeRight of 
                Nothing -> Nothing
                Just right-> aux (dM, dR) right
    effComp = ArithInOut.rrEffortNumComp sampleDom effDom
--    effJoinMeet = ArithInOut.rrEffortJoinMeet sampleDom effDom
    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    sampleDom = dom

bisectionInfoEvalFn ::
    (ArithInOut.RoundedReal dom,
     RefOrd.IntervalLike dom)
    => 
    ArithInOut.RoundedRealEffortIndicator dom ->
    (segInfo -> a) {-^ the evaluation function -} -> 
    BisectionInfo segInfo otherInfo {-^ bisected function  -} -> 
    (dom, dom) {-^ the domain of the function encoded by the above bisection -} -> 
    dom {-^ @dom@ - domain to evaluate the function on -} -> 
    [[a]] {-^ evaluation on various sub-segments of @dom@, possibly in multiple ways -}
bisectionInfoEvalFn effDom evalFn bisectionInfo bisectionDom domG =
    aux bisectionDom bisectionInfo domG
    where
    aux _ (BisectionNoSplit info) _ = [[evalFn info]]
    aux (dL, dR) (BisectionSplit _ left maybeRight) dom 
        | domNotInR = auxLeft
        | domNotInL = auxRight
        | domInsideBothLR =
            zipWith (++) auxLeft auxRight 
        | otherwise = -- domSplitBetweenLR =
            auxLeft ++ auxRight
        where
        domNotInL =
            (dM <? dom) == Just True
        domNotInR =
            (dom <? dM) == Just True
        domInsideBothLR = 
            (dom ==? dM) == Just True
        dM = 
            getMidPoint effAddDom effDivDomInt dL dR 
        auxLeft = aux (dL, dM) left domL
        auxRight = 
            case maybeRight of 
                Nothing -> [[]]
                Just right-> aux (dM, dR) right domR
        domL = 
            dom <\/> (dL </\> dM)
        domR =
            dom <\/> (dM </\> dR)
    (</\>) = RefOrd.meetOutEff effJoinMeet
    (<\/>) = RefOrd.joinOutEff effJoinMeet
    effComp = ArithInOut.rrEffortNumComp sampleDom effDom
    effJoinMeet = ArithInOut.rrEffortJoinMeet sampleDom effDom
    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    sampleDom = domG
    
bisectionInfoTrimAt :: 
    (ArithInOut.RoundedReal dom,
     RefOrd.IntervalLike dom)
    => 
    ArithInOut.RoundedRealEffortIndicator dom 
    ->
    (segInfo1 -> segInfo1)
    -> 
    (segInfo1 -> segInfo1)
    -> 
    BisectionInfo segInfo1 splitReason
    -> 
    (dom, dom)
    -> 
    dom
    -> 
    BisectionInfo segInfo1 splitReason
bisectionInfoTrimAt effDom trimResult removeResult bisectionInfoG bisectionDom cutOffPoint =
    aux bisectionDom bisectionInfoG
    where
    aux (dL, dR) bisectionInfo =
        case (dR <=? cutOffPoint, cutOffPoint <=? dL) of
            (Just True, _) -> bisectionInfo -- entirely to the left of the point
            (_, Just True) -> bisectionInfoRemoveSegInfo bisectionInfo
            _ ->
                case bisectionInfo of
                    BisectionNoSplit info -> BisectionNoSplit $ trimResult info
                    BisectionSplit splitReason left maybeRight ->
                        BisectionSplit splitReason newLeft maybeNewRight
                        where
                        newLeft = aux (dL, dM) left
                        maybeNewRight =
                            fmap (aux (dM, dR)) maybeRight
                        dM = getMidPoint effAddDom effDivDomInt dL dR 
    
    bisectionInfoRemoveSegInfo (BisectionNoSplit info) = BisectionNoSplit $ removeResult info
    bisectionInfoRemoveSegInfo (BisectionSplit splitReason left _maybeRight) =
        BisectionSplit splitReason (bisectionInfoRemoveSegInfo left) Nothing
    
    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    sampleDom = cutOffPoint
            
getMidPoint :: 
    (RefOrd.IntervalLike dom, 
     ArithInOut.RoundedAdd dom,
     ArithInOut.RoundedMixedDivide dom Int) 
    =>
    ArithInOut.AddEffortIndicator dom 
    -> 
    ArithInOut.MixedDivEffortIndicator dom Int 
    -> 
    dom -> dom -> dom
getMidPoint effAddDom effDivDomInt l r =             
    fst $ RefOrd.getEndpointsOut $
    (l <+> r) </>| (2 :: Int)
    where
    (<+>) = ArithInOut.addOutEff effAddDom
    (</>|) = ArithInOut.mixedDivOutEff effDivDomInt 
    
        