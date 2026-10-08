{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Picard.UncertainValue
    Description :  uncertain initial value ODE solvers
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Uncertain initial value ODE solvers using interval Picard methods.
-}

module Numeric.AERN.IVP.Solver.Picard.UncertainValue
(
    solveODEIVPUncertainValueExactTime_PicardIterations,
    solveODEIVPUncertainValueExactTime_UsingPicard,
    solveODEIVPUncertainValueExactTime_UsingPicard_Bisect
)
where

import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Solver.Bisection
import Numeric.AERN.IVP.Solver.ShrinkWrap

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Exception

import Debug.Trace
_ = trace
        

solveODEIVPUncertainValueExactTime_UsingPicard_Bisect ::
    (CanAddVariables f,
     CanEvaluate f,
     CanPartiallyEvaluate f,
     CanCompose f,
     CanChangeSizeLimits f,
     HasProjections f,
     HasConstFns f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     RoundedFakeDerivative f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedAbs f,
     NumOrd.RefinementRoundedLattice f,
     ArithInOut.RoundedMixedDivide f Int,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f),
     Eq (Var f),
     Domain f ~ Imprecision (Domain f),
     solvingInfo ~ (Maybe ([f],[f]), (Domain f, Maybe [Domain f])), 
     Show f, Show (Domain f), Show (Var f))
    =>
    Bool {-^ should wrap intermediate values ? -}
    -> 
    Bool {-^ if not wrapping, should try to shrink wrap parameterised intermediate values ? -}
    ->
    SizeLimits f {-^ size limits for all function -} ->
    SizeLimitsChangeEffort f ->
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    IntegrationEffortIndicator f ->
    FakeDerivativeEffortIndicator f ->
    RefOrd.PartialCompareEffortIndicator f ->
    ArithInOut.AddEffortIndicator f ->
    ArithInOut.MultEffortIndicator f ->
    ArithInOut.AbsEffortIndicator f ->
    NumOrd.MinmaxInOutEffortIndicator f ->
    ArithInOut.MixedDivEffortIndicator f Int ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    Domain f {-^ initial widening @delta@ -}  ->
    Int {-^ @m@ -} -> 
    Domain f {-^ minimum step size @s@ -} -> 
    Domain f {-^ maximum step size -} -> 
    Imprecision (Domain f) {-^ split improvement threshold @eps@ -} ->
    ODEIVP f
    ->
    (
        (Maybe [Domain f])
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe (Imprecision (Domain f)))
        )
    )
solveODEIVPUncertainValueExactTime_UsingPicard_Bisect
        shouldWrap shouldShrinkWrap
            sizeLimits effSizeLims effCompose effEval effInteg effDeriv effInclFn 
            effAddFn effMultFn effAbsFn effMinmaxFn 
            effDivFnInt effAddFnDom effMultFnDom effDom
                delta m minStepSize maxStepSize splitImprovementThreshold
                    (odeivpG :: ODEIVP f)
    | (tStart <? t0End) == Just True =
        error "aern-ivp: solveUncertainValueExactTime called with an uncertain time IVP"
    | otherwise =
        case solve odeivpG of
            ((Nothing, _), bisectionInfo) ->
                (Nothing, bisectionInfo)
            ((Just (_, parameterisedInitialValues), _), bisectionInfo) ->
                (Just $ map (getRangeFullEval effEval) parameterisedInitialValues, bisectionInfo)
    where
--    tVar = odeivp_tVar odeivpG
    tStart, t0End :: Domain f
    tStart = odeivp_tStart odeivpG
    t0End = odeivp_t0End odeivpG
    componentNames = odeivp_componentNames odeivpG
    imprecisionLimit = odeivp_enclosureRangeWidthLimit odeivpG

    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
--    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
    sampleDom :: Domain f
    sampleDom = tStart

    
    solve odeivp =
        solveODEIVPByBisectingT
            solveODEIVPNoSplitting 
                (measureImpr) 
                (makeFnVec)
                (intersectDomain)
                effDom splitImprovementThreshold minStepSize maxStepSize
                    odeivp
    measureImpr (_, parameterisedInitialValues) =
        measureResultImprecision effAddDom ranges
        where
        ranges = map (getRange effEval) parameterisedInitialValues
    intersectDomain r@(_, parameterisedInitialValues) =
        case odeivp_intersectDomain odeivpG ranges of
            Just _ -> Just r
            Nothing -> Nothing
        where
        ranges = map (getRange effEval) parameterisedInitialValues
    makeFnVec (_, parameterisedInitialValues) =
        makeFnVecFromParamInitialValuesOut effAddFn effMultFn effSizeLims componentNames parameterisedInitialValues
    solveODEIVPNoSplitting odeivp = 
--        trace 
--        (
--            "solveODEIVPNoSplitting:"
--            ++ "\n  tStart = " ++ show (odeivp_tStart odeivp) 
--            ++ "\n  tEnd = " ++ show (odeivp_tEnd odeivp) 
--            ++ "\n  resultImprecision = " ++ show imprec 
--        )
--        $
        case imprecisionOverLimit  of
            True -> (Nothing, (odeivp_tEnd odeivp, Nothing))
            False -> result
        where
        (imprec, imprecisionOverLimit) =
            case maybeParamValues of
                Just (_, parameterisedInitialValues) ->
                    (Just imprecisions, or $ map overLimit imprecisions)
                    where
                    imprecisions = measureResultImprecisions effAddDom ranges
                    ranges = map (getRange effEval) parameterisedInitialValues
                    overLimit imprec2 =
                        (imprec2 >? imprecisionLimit) == Just True
                _ -> 
                    (Nothing, False)
        result@(maybeParamValues, _) =
            solveODEIVPUncertainValueExactTime_UsingPicard 
                shouldWrap shouldShrinkWrap
                    sizeLimits effCompose effEval effInteg effDeriv effInclFn 
                    effAddFn effAbsFn effMinmaxFn 
                    effDivFnInt effAddFnDom effMultFnDom effDom
                        delta m splitImprovementThreshold
                            odeivp 


measureResultImprecision :: 
    (RefOrd.IntervalLike dom, 
     ArithInOut.RoundedSubtr dom, 
     Show dom
    )
    =>
    ArithInOut.AddEffortIndicator dom
    -> 
    [dom]
    ->
    dom
measureResultImprecision effAddDom resVec =
    result
    where
    result =
        snd $ RefOrd.getEndpointsOut $
        foldl1 (<+>) imprVec
    imprVec = measureResultImprecisions effAddDom resVec
    (<+>) = ArithInOut.addOutEff effAddDom

measureResultImprecisions :: 
    (RefOrd.IntervalLike dom, 
     ArithInOut.RoundedSubtr dom, 
     Show dom
    )
    =>
    ArithInOut.AddEffortIndicator dom
    -> 
    [dom]
    ->
    [dom]
measureResultImprecisions effAddDom resVec =
    imprVec
    where
    imprVec = map perComp resVec
    perComp res =
        snd $ RefOrd.getEndpointsOut $
        resR <-> resL
        where
        (resL, resR) = RefOrd.getEndpointsOut res
        (<->) = ArithInOut.subtrOutEff effAddDom


measureFunctionImprecision :: 
    (RefOrd.IntervalLike (Domain f), 
     RefOrd.IntervalLike f,
     ArithInOut.RoundedReal (Domain f), 
     ArithInOut.RoundedSubtr f,
     CanEvaluate f,
     Eq (Var f),
     Show (Domain f), Show f
    )
    =>
    EvaluationEffortIndicator f
    -> 
    ArithInOut.AddEffortIndicator f
    -> 
    ArithInOut.AddEffortIndicator (Domain f)
    -> 
    [f]
    -> 
    Domain f
measureFunctionImprecision effEval effAddFn effAddDom fnVec =
--    trace
--    (
--        "measureFunctionImprecision:"
--        ++ "\n fnVec = " ++ show fnVec
--        ++ "\n imprVec = " ++ show imprVec
--        ++ "\n result = " ++ show result
--    ) $
    result
    where
    result =
        snd $ RefOrd.getEndpointsOut $
        foldl1 (.<+>.) imprVec
    imprVec = map perComp fnVec
    perComp fn =
        getRange effEval $ fnR ~<->~ fnL
        where
        (fnL, fnR) = RefOrd.getEndpointsOut fn

    (.<+>.) = ArithInOut.addOutEff effAddDom
    (~<->~) = ArithInOut.subtrOutEff effAddFn

getRange ::
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Eq (Var f)) 
    =>
    EvaluationEffortIndicator f -> 
    f -> Domain f
getRange 
    | True = getRangeBySamples -- comment out to switch
    | otherwise = getRangeFullEval 

getRangeFullEval ::
    (CanEvaluate f) 
    =>
    EvaluationEffortIndicator f -> 
    f -> Domain f
getRangeFullEval effEval fn =
    evalAtPointOutEff effEval (getDomainBox fn) fn

getRangeBySamples ::
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Eq (Var f)) 
    =>
    EvaluationEffortIndicator f -> 
    f -> Domain f
getRangeBySamples effEval fn =
    foldl1 (RefOrd.</\>) values 
    where
    values = map head $ evalSamplesAlongEdgesEff effEval 1 (getDomainBox fn) allVars [fn]
    allVars = getVars $ getDomainBox fn

solveODEIVPUncertainValueExactTime_UsingPicard ::
    (CanAddVariables f,
     CanEvaluate f,
     CanPartiallyEvaluate f,
     CanCompose f,
     HasProjections f,
     HasConstFns f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     RoundedFakeDerivative f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedAbs f,
     NumOrd.RefinementRoundedLattice f,
     ArithInOut.RoundedMixedDivide f Int,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f),
     Eq (Var f),
     Domain f ~ Imprecision (Domain f),
     solvingInfo ~ (Domain f, Maybe [Domain f]), 
     Show f, Show (Domain f), Show (Var f))
    =>
    Bool {-^ should wrap intermediate values ? -}
    -> 
    Bool {-^ if not wrapping, should try to shrink wrap parameterised intermediate values ? -}
    ->
    SizeLimits f {-^ size limits for all function -} ->
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    IntegrationEffortIndicator f ->
    FakeDerivativeEffortIndicator f ->
    RefOrd.PartialCompareEffortIndicator f ->
    ArithInOut.AddEffortIndicator f ->
    ArithInOut.AbsEffortIndicator f ->
    NumOrd.MinmaxInOutEffortIndicator f ->
    ArithInOut.MixedDivEffortIndicator f Int ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    Domain f {-^ initial widening @delta@ -}  ->
    Int {-^ @m@ -} -> 
    Domain f {-^ step size @s@ -} -> 
    ODEIVP f
    ->
    (Maybe ([f],[f]), (Domain f, Maybe [Domain f]))
solveODEIVPUncertainValueExactTime_UsingPicard 
        shouldWrap shouldShrinkWrap
            sizeLimits effCompose effEval effInteg effDeriv effInclFn 
            effAddFn effAbsFn effMinmaxFn 
            effDivFnInt effAddFnDom effMultFnDom effDom
                delta m splitImprovementThreshold
                    (odeivp :: ODEIVP f) 
    =
    case maybeResult of
        Just (result, resultAtTEnd) ->
            let valuesAtEnd = resultAtTEnd in
            let paramValuesAtTEnd = evalAtTEnd valuesAtEnd $ result in
            (Just (result, paramValuesAtTEnd), (tEnd, Just valuesAtEnd))
        Nothing -> (Nothing, (tEnd, Nothing))
    where
    tVar = odeivp_tVar odeivp
    tStart :: Domain f
    tStart = odeivp_tStart odeivp
--    t0End = odeivp_t0End odeivp
    componentNames = odeivp_componentNames odeivp

    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
--    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
    sampleDom :: Domain f
    sampleDom = tStart

    evalAtTEnd valuesAtEnd fns
        | shouldWrap = wrappedParamValuesAtTEnd
        | shouldShrinkWrap = 
            case shrinkWrappedParamValuesAtTEnd of
                Just fns2 ->
--                    unsafePrint ("solveUncertainValueExactTimeBisect: shrink wrapping succeeded") $ 
                    fns2
                _ | haveAnExactDomain -> 
--                    unsafePrint ("solveUncertainValueExactTimeBisect: shrink wrapping failed, wrapping") $ 
                    wrappedParamValuesAtTEnd
                _ -> 
--                    unsafePrint ("solveUncertainValueExactTimeBisect: shrink wrapping failed, leaving it thick") $ 
                    thickParamValuesAtTEnd 
        | otherwise = thickParamValuesAtTEnd
        where
        wrappedParamValuesAtTEnd =
            parameteriseInitialValues sizeLimits componentNames valuesAtEnd
        haveAnExactDomain =
            or $ map isExactDom $ domains
            where
            isExactDom dom =
                isExact dom == Just True
--            isExact dom =
--                (domL ==? domR) == Just True
--                where
--                (domL, domR) = RefOrd.getEndpointsOut dom
            domains = map snd $ toAscList $ getDomainBox fn
            (fn : _) = fns 
        shrinkWrappedParamValuesAtTEnd =
            shrinkWrap 
                effCompose effEval effDeriv effAddFn effAbsFn effMinmaxFn 
                    effDivFnInt effAddFnDom effMultFnDom effDom $
                        thickParamValuesAtTEnd
        thickParamValuesAtTEnd =
            fst $ partiallyEvalAtEndTimeOutInVec tVar tEnd (fns, fns)
    
    maybeResult =
        fmap (untilStableButNotMoreThan m) maybeIterations
    untilStableButNotMoreThan maxIters (hInit : tInit) =
        aux 1 hInitImprecision tInit
        where
        hInitImprecision = measureFunctionImprecision effEval effAddFn effAddDom hInit
        aux iterNo prevImprecision (h : t)
            | iterNo >= maxIters = (h, hAtTEnd)
            | notMuchImprovement =
--                 unsafePrint 
--                    ("solveUncertainValueExactTimeBisect:"
--                        ++ " finished after " ++ show iterNo 
--                        ++ " iterations (max = " ++ show maxIters ++ ")") $
                (h, hAtTEnd)
            | otherwise =
--                unsafePrint 
--                    ("solveUncertainValueExactTimeBisect: "
--                        ++ "hImprecision = " ++ show hImprecision
--                    ) $
                aux (iterNo + 1) hImprecision t
            where
            notMuchImprovement = 
                (prevImprecision .<->. hImprecision <=? splitImprovementThreshold) == Just True  
            hImprecision = measureFunctionImprecision effEval effAddFn effAddDom h
            hAtTEnd = fst $ evalAtEndTimeVec effEval tVar tEnd h
    maybeIterations :: Maybe [[f]]
    maybeIterations =
        solveODEIVPUncertainValueExactTime_PicardIterations
            sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
                delta
                    odeivp
    tEnd = odeivp_tEnd odeivp
    (.<->.) = ArithInOut.subtrOutEff effAddDom
                
solveODEIVPUncertainValueExactTime_PicardIterations ::
    (CanAddVariables f,
     CanEvaluate f,
     CanCompose f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike(Domain f), 
     Show f, Show (Domain f))
    =>
    SizeLimits f ->
    CompositionEffortIndicator f ->
    IntegrationEffortIndicator f ->
    RefOrd.PartialCompareEffortIndicator f ->
    ArithInOut.AddEffortIndicator f ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    Domain f {-^ initial widening @delta@ -}  ->
    ODEIVP f ->
    Maybe [[f]] {-^ sequence of enclosures with domain @T x D@ produced by the Picard operator -}
solveODEIVPUncertainValueExactTime_PicardIterations
        sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
        delta
        odeivp
    | (tStart <? t0End) == Just True = 
        error "aern-ivp: solveUncertainValueExactTime called with an uncertain time IVP"
    | otherwise =
        case evalCatchAERNExceptions "solveUncertainValueExactTime" result of
            Right res -> res
            _ -> Nothing
    where
    result =
        case findEnclosure (50 :: Int) $ iterate picard initialAttemptFns of
            Just firstEnclosure ->
                Just $ iterate picard firstEnclosure
            Nothing -> Nothing
    field = odeivp_field odeivp
    tVar = odeivp_tVar odeivp
    tStart = odeivp_tStart odeivp
    tEnd = odeivp_tEnd odeivp
    t0End = odeivp_t0End odeivp
    initialValuesFnVec =
        map (composeVarOutEff effCompose tVar tStartFn) initialValuesFnVecWithT
        where
        tStartFn = newConstFnFromSample sampleFn tStart
        (sampleFn : _) = initialValuesFnVecWithT
    initialValuesFnVecWithT =
        odeivp_makeInitialValueFnVec odeivp sizeLimits tVar timeDomain
    
    timeDomain =
        RefOrd.fromEndpointsOut (tStart, tEnd)

    effJoinDom = ArithInOut.rrEffortJoinMeet sampleDom effDom
    sampleDom = tStart
    
        
    initialAttemptFns =
        map widenFn initialValuesFnVec
        where
        widenFn fn = fn ~<+>. ((neg delta) </\> delta) 

        (~<+>.) = ArithInOut.mixedAddOutEff effAddFnDom
        (</\>) = RefOrd.meetOutEff effJoinDom
    findEnclosure maxIter = aux 0 
        where
        aux iterNo (fn1Vec : fn2Vec : rest)
            | iterNo <= maxIter =
                case fn2RefinesFn1 of
                    True ->
--                        unsafePrint 
--                            ("solveUncertainValueExactTime:" 
--                                ++ " enclosure found after " ++ show iterNo ++ " iterations"
--                                ++ " (max = " ++ show maxIter ++ ")") $ 
                        Just fn2Vec
                    _ ->
--                        unsafePrint
--                        (
--                            "solveUncertainValueExactTime: findEnclosure: not yet enclosing:"
--                            ++ "\n fn1Vec = " ++ (show $ evalAtEndTimeVec fn1Vec)
--                            ++ "\n fn2Vec = " ++ (show $ evalAtEndTimeVec fn2Vec)
--                            ++ "\n fn1Vec at end time = " ++ (show $ evalAtEndTimeVec fn1Vec)
--                            ++ "\n fn2Vec at end time = " ++ (show $ evalAtEndTimeVec fn2Vec)
--                        ) $
                        aux (iterNo + 1) (fn2Vec : rest)
            where
            fn2RefinesFn1 =
                and $ map (== (Just True)) $ zipWith (|<=?) fn1Vec fn2Vec
                where
                (|<=?) = RefOrd.pLeqEff effInclFn
        aux _ _  =
            Nothing
--            error "aern-picard: solveUncertainValueExactTime failed to find enclosure"
    picard xvec = 
--        unsafePrint
--        (
--            "solveUncertainValueExactTime: picard:"
--            ++ "\n tEnd = " ++ (show tEnd)
--            ++ "\n initialValuesFnVec = " ++ (show initialValuesFnVec)
--            ++ "\n xvec = " ++ (show xvec)
--            ++ "\n xdvec = " ++ (show xdvec)
--            ++ "\n result = " ++ (show result)
----            ++ "\n xvec at end time = " ++ (show $ evalAtEndTimeVec tVar tEnd xvec)
----            ++ "\n xdvec at end time = " ++ (show $ evalAtEndTimeVec tVar tEnd xdvec)
----            ++ "\n result at end time = " ++ (show $ evalAtEndTimeVec tVar tEnd result)
--        ) $
        resultPic
        where
        resultPic =
            zipWith (~<+>~) primitFn initialValuesFnVec
        primitFn = map picardFn xdvec
        xdvec = field xvec
        picardFn xdi =
            primitiveFunctionOutEff effInteg xdi tVar
        (~<+>~) = ArithInOut.addOutEff effAddFn

