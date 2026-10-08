{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Events.PWL
    Description :  hybrid system simulation step using passed and waiting lists
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Hybrid system simulation step using passed and waiting lists.
-}

module Numeric.AERN.IVP.Solver.Events.PWL
--(
--)
where

import Numeric.AERN.IVP.Solver.Events.Locate

import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Solver.Picard.UncertainValue
--import Numeric.AERN.IVP.Solver.Picard.UncertainTime

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsDefaultEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

--import Data.Maybe (catMaybes)
import qualified Data.Map as Map
import qualified Data.Set as Set
--import qualified Data.List as List

import Control.Monad.Error

import Numeric.AERN.Misc.Debug
_ = unsafePrint

solveHybridIVP_UsingPicardAndPWL ::
    (CanAddVariables f,
     CanRenameVariables f,
     CanEvaluate f,
     CanCompose f,
     CanPartiallyEvaluate f,
     HasProjections f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     NumOrd.RefinementRoundedLattice f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f), 
     HasAntiConsistency (Domain f), 
     Show f, Show (Domain f), Show (Var f), Eq (Var f)
     )
    =>
    SizeLimits f 
    ->
    PartialEvaluationEffortIndicator f 
    ->
    CompositionEffortIndicator f 
    ->
    EvaluationEffortIndicator f 
    ->
    IntegrationEffortIndicator f 
    ->
    RefOrd.PartialCompareEffortIndicator f 
    ->
    ArithInOut.AddEffortIndicator f 
    ->
    ArithInOut.MultEffortIndicator f 
    ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) 
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f)
    ->
    Int {-^ maximum number of potential events to consider -}
    ->
    Domain f {-^ initial widening @delta@ -}
    ->
    Int  {-^ @m@ - limit on the number of Picard iterations after stabilisation -}
    ->
    Var f {-^ @t0@ - the initial time variable -} 
    ->
    HybridIVP f 
    ->
    (Maybe (HybridSystemUncertainState (Domain f), HybridSystemUncertainState f))
solveHybridIVP_UsingPicardAndPWL
        sizeLimits effPEval effCompose effEval effInteg effInclFn effAddFn _effMultFn effAddFnDom effDom
            maxNodes
                delta m
                    _t0Var
                        (hybivp :: HybridIVP f)
    =
    case result of
        Left errorMessage -> Nothing
        Right (finalState, stateOverT) -> Just (finalState, stateOverT) 
    where
    effJoinDom = ArithInOut.rrEffortJoinMeet sampleDom effDom
    effRefCompDom = ArithInOut.rrEffortRefComp sampleDom effDom
    sampleDom = tStart
    tStart = hybivp_tStart hybivp
    tEnd = hybivp_tEnd hybivp
    tDom = RefOrd.fromEndpointsOut (tStart, tEnd)
    tVar = hybivp_tVar hybivp
    hybsys = hybivp_system hybivp
    componentNames = hybsys_componentNames hybsys
    modeInvariants = hybsys_modeInvariants hybsys
    eventSpecification = hybsys_eventSpecification hybsys
    initState = hybivp_initialStateEnclosure hybivp
    
    result =
        do
        (s0_FnVec, sampleFn) <- solveODEsForState initState
        let final0_DomVec = Map.map (evalFnVecAt effEval tVar tEnd) s0_FnVec
        let s0_DomVec = Map.map (getRangesOfFnVec effEval) s0_FnVec
        sEvents_DomVec <- reachableStatesPWL maxNodes Map.empty s0_DomVec
        let sEvents_FnVec = Map.map (map $ newConstFnFromSample sampleFn) sEvents_DomVec
        return $ 
            (mergeHybridStates effJoinDom 
                (sEvents_DomVec :: HybridSystemUncertainState (Domain f)) 
                (final0_DomVec :: HybridSystemUncertainState (Domain f))
            , 
             (sEvents_FnVec :: HybridSystemUncertainState f) 
                `mergeHybridStatesFns` 
             (s0_FnVec :: HybridSystemUncertainState f)
            )
        where
        mergeHybridStatesFns :: 
            HybridSystemUncertainState f ->
            HybridSystemUncertainState f ->
            HybridSystemUncertainState f
        mergeHybridStatesFns s1 s2 =
            Map.unionWith (zipWith fnUnionByBox) s1 s2
            where
            fnUnionByBox :: f -> f -> f
            fnUnionByBox f1 f2 = f1
--                newConstFnFromSample f1 (range1 </\> range2)
--                where
--                [range1, range2] = getRangesOfFnVec effEval [f1, f2]
    
    solveODEsForState :: 
        HybridSystemUncertainState (Domain f) ->
        Either String (HybridSystemUncertainState f, f) 
    solveODEsForState initState =
        do
        resultList <- mapM solveODEForMode initStateList
        let ((_,(sampleFn:_)): _) = resultList  
        return (Map.fromAscList resultList, sampleFn)
        where
        initStateList = Map.toAscList initState
    
    solveODEForMode initState@(initialMode, initialValues) =
        case maybeIterations of
            Nothing -> 
                Left $ "ODE solver gave up for initial state " ++ show initState
            Just iterations -> 
                Right $ (initialMode, map removeAllVarsButT $ iterations !! m)
        where
        maybeIterations =
            solveODEIVPUncertainValueExactTime_PicardIterations
                sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
                delta
                (odeivp tStart initialMode $ 
                    makeFnVecFromInitialValues componentNames initialValues)
        removeAllVarsButT fn =
            -- substitute domains for all variables except t:
            pEvalAtPointOutEff effPEval domboxNoT fn
            where
            domboxNoT =
                fromAscList $ filter ((/= tVar) . fst) varDoms
            varDoms = 
                toAscList $ getDomainBox fn
        
    reachableStatesPWL ::
        Int ->
        HybridSystemUncertainState (Domain f) ->
        HybridSystemUncertainState (Domain f) ->
        Either String (HybridSystemUncertainState (Domain f))
    reachableStatesPWL maxIters passed_in waiting_in
        | Map.null waiting_in = Right passed_in
        | maxIters == 0 = Left "reachableStatesPWL exceeded maxIters"
        | otherwise =
            do
            (s_evolved_fnVec, _sampleFn) <- solveODEsForState $ encloseOneEvent waiting_in
            let s_evolved  = Map.map (getRangesOfFnVec effEval) s_evolved_fnVec 
            let waiting_out = differenceHybridStates effRefCompDom s_evolved passed_in
            let passed_out = mergeHybridStates effJoinDom s_evolved passed_in 
            reachableStatesPWL (maxIters - 1) passed_out waiting_out
                
    encloseOneEvent state =
        error "PWL not fully implemented yet"
--        intersectInv modeInvariants $
--        applyAllEventsInParallel eventSpecification $
--        state

    odeivp t0End mode makeInitValueFnVec =
        ODEIVP
        {
            odeivp_description = "ODE for " ++ show mode,
            odeivp_field = field,
            odeivp_componentNames = componentNames,
            odeivp_tVar = tVar,
            odeivp_tStart = tStart,
            odeivp_tEnd = tEnd,
            odeivp_makeInitialValueFnVec = makeInitValueFnVec,
            odeivp_t0End = t0End,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
        where
        Just field = Map.lookup mode modeFields
        modeFields = hybsys_modeFields hybsys
    
wrapFnVecAsBox ::
    (CanEvaluate f, HasConstFns f) 
    =>
    EvaluationEffortIndicator f -> 
    ([Domain f] -> Maybe [Domain f]) -> 
    [f] -> 
    Maybe [f]
wrapFnVecAsBox effEval transformVec fnVec =
    do
    resultVec <- transformVec $ getRangesOfFnVec effEval fnVec
    return $ map (newConstFnFromSample sampleFn) $ resultVec 
    where
    (sampleFn : _) = fnVec

getRangesOfFnVec ::
    (CanEvaluate f) 
    =>
    EvaluationEffortIndicator f -> 
    [f] -> 
    [Domain f]
getRangesOfFnVec effEval fnVec =
    result
    where
    result = map (evalAtPointOutEff effEval dombox) fnVec         
    dombox = getDomainBox sampleFn
    (sampleFn : _) = fnVec

evalFnVecAt ::
    (CanEvaluate f) 
    =>
    EvaluationEffortIndicator f ->
    Var f ->
    Domain f -> 
    [f] ->
    [Domain f]
evalFnVecAt effEval tVar tVal fnVec =
    result
    where
    result = map (evalAtPointOutEff effEval domboxT) fnVec
    domboxT = insertVar tVar tVal dombox
    dombox = getDomainBox sampleFn
    (sampleFn : _) = fnVec
    
