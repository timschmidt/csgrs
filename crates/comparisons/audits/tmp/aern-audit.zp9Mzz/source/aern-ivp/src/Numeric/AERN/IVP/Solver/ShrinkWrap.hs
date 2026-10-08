--{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.ShrinkWrap
    Description :  shrink wrapping an interval function
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Shrink wrapping an interval function @f@, ie finding a function @g@
    whose range is almost the same as that of @f@ but is exact.  Moreover,
    the range of @g@ must fully cover the range of @f@.
-}

module Numeric.AERN.IVP.Solver.ShrinkWrap 
(
    shrinkWrap
)
where

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
--import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Measures

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Misc.Debug
import Data.List (intercalate)
_ = unsafePrint -- stop the unused warning
_ = intercalate

shrinkWrap ::
    (
     HasProjections f,
     HasConstFns f,
     CanEvaluate f,
     CanCompose f,
     RoundedFakeDerivative f,
     RefOrd.IntervalLike f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedAbs f,
     NumOrd.RefinementRoundedLattice f,
     ArithInOut.RoundedMixedDivide f Int,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Domain f)
    )
    =>
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    FakeDerivativeEffortIndicator f ->
    ArithInOut.AddEffortIndicator f ->
    ArithInOut.AbsEffortIndicator f ->
    NumOrd.MinmaxInOutEffortIndicator f ->
    ArithInOut.MixedDivEffortIndicator f Int ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    [f] ->
    Maybe [f]
shrinkWrap effComp effEval effDeriv effAddFn effAbsFn effMinmaxFn effDivFnInt effAddFnDom effMultFnDom effDom fns =
--    unsafePrint
--    (
--        "shrinkWrap:"
----        ++ "\n fns = " ++ (show fns)
--        ++ "\n ws = " ++ (show ws)
--        ++ "\n delta = " ++ (show delta)
--    )$
    result
    where
    result 
        | (delta >? threshold) == Just True =
            Nothing
        | otherwise =
            Just $ map zoomDomains $ map snd wAndfnMs
    threshold =
        maxW .<+>. maxW -- <+> maxW <+> maxW
    maxW = foldl1 (NumOrd.maxOutEff effMinmax) ws
    ws = map fst wAndfnMs
    
    zoomDomains =
        zoomDomainsInterpretationBy effComp effAddFnDom effMultFnDom effDom delta
    delta = 
        getDomainDelta1 
            effComp effEval effDeriv effAbsFn effMinmaxFn effAddFnDom effMultFnDom effDom wAndfnMs
    wAndfnMs =
        map getWfnM fns

    getWfnM fn = 
--        unsafePrint
--        (
--            "getWfnM:"
--            ++ "\n fn = " ++ show fn
--            ++ "\n fnL = " ++ show fnL
--            ++ "\n fnR = " ++ show fnR
--            ++ "\n wDbl = " ++ show wDbl
--            ++ "\n w = " ++ show w
--        ) $ 
        (w, fnM)
        where 
        w =
            wTimesTwo .</>| (2 :: Int)
--            let ?addInOutEffort = effAddFn in
--            (evalAtPointOutEff effEval dombox $ fnR <-> fnL) </>| (2 :: Int) 
            
        wTimesTwo =
            evalAtPointOutEff effEval dombox $ fnR ~<->~ fnL 
        fnM =
            (fnL ~<+>~ fnR) ~</>| (2 :: Int)
        (fnL, fnR) = 
            RefOrd.getEndpointsOut fn
        dombox = getDomainBox fn 

    (.<+>.) = ArithInOut.addOutEff effAdd
    (.</>|) = ArithInOut.mixedDivOutEff effDivDomInt

    (~<+>~) = ArithInOut.addOutEff effAddFn
    (~<->~) = ArithInOut.subtrOutEff effAddFn
    (~</>|) = ArithInOut.mixedDivOutEff effDivFnInt
    
    effMinmax = 
        ArithInOut.rrEffortMinmaxInOut sampleDom effDom
    effAdd = 
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDomInt = 
        ArithInOut.mxfldEffortDiv sampleDom (0::Int) $ ArithInOut.rrEffortIntMixedField sampleDom effDom
        -- TODO report GHC BUG: if changing Div -> Add above, GHC compiles it but gives Seg Fault when computing w
        -- also need to remove wDbl and replace it with the commented out expression above for the bug to demonstrate
    sampleDom = 
        case fns of 
            (fn : _) -> getSampleDomValue fn
            _ -> error "aern-ivp: shrinkWrap: internal error"
    
_getDomainDelta2 ::
    (
     HasProjections f,
     HasConstFns f,
     CanEvaluate f,
     CanCompose f,
     RoundedFakeDerivative f,
     RefOrd.IntervalLike f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedAbs f,
     NumOrd.RefinementRoundedLattice f,
     ArithInOut.RoundedMixedDivide f Int,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Domain f)
    )
    =>
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    FakeDerivativeEffortIndicator f ->
    ArithInOut.AbsEffortIndicator f ->
    NumOrd.MinmaxInOutEffortIndicator f ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    [(Domain f, f)] ->
    Domain f
_getDomainDelta2 effComp effEval effDeriv effAbsFn effMinmaxFn effAddFnDom effMultFnDom effDom wAndfns =
    {- 
        The constraint for delta depends on delta.  We try to obtain
        a delta that satisfies the constraint by iterating the
        formula in the constraint.
    -}
    getPeak (100 :: Int) $ iterate computeDelta initDelta
    where
    initDelta = zero sampleDom
    getPeak maxTrials (delta1 : rest@(delta2 : _)) 
        | maxTrials <= 0 = 
            error $ "aern-ivp: getDomainDelta: failed to find delta for wAndfns = " ++ show wAndfns
        | detectedNotIncreasing =
            delta1
        | otherwise =
            getPeak (maxTrials - 1) rest
        where
        detectedNotIncreasing = 
--            unsafePrint
--            (
--                "ShrinkWrap: getDomainDelta: getPeak: delta1 = " ++ show delta1
--            ) $
            ((delta1 <? plusInfinity delta1) /= Just True)
            ||
            ((delta2 <=? delta1) == Just True)
    computeDelta oldDelta =
        {-
            max_{i=1..n} w_i / ( max_{j=1..n} |fn_i/dx_j|)
            
            The norm of the derivative is taken over domains extended
            by oldDelta.
        -}
        foldl1 (NumOrd.maxOutEff effMinmax) $  map (getFnDelta oldDelta) wAndfns
    getFnDelta oldDelta (w, fn) =
--        unsafePrint
--        (
--            "ShrinkWrap: getDomainDelta: getFnDelta:" 
--                ++ "\n fnWithOldDelta = " ++ show fnWithOldDelta
--                ++ "\n slopes = \n   " ++ (intercalate "\n   " $ map show slopes)
--                ++ "\n maxSlopeFn = " ++ show maxSlopeFn
--                ++ "\n maxSlope = " ++ show maxSlope
--                ++ "\n fnDelta = " ++ show fnDelta
--        ) $
        snd $ RefOrd.getEndpointsOut $
        fnDelta
        where
        fnDelta = 
            w .</>. maxSlope 
        maxSlope =
            evalAtPointOutEff effEval dombox maxSlopeFn
        maxSlopeFn =
            foldl1 maxDn slopes
            where
            maxDn slope1 slope2 =
                fst $ RefOrd.getEndpointsOut $
                    NumOrd.maxOutEff effMinmaxFn slope1 slope2
        slopes = map getSlope vars
        getSlope var =
            fst $ RefOrd.getEndpointsOut $
                ArithInOut.absOutEff effAbsFn $
                    fakePartialDerivativeOutEff effDeriv fnWithOldDelta var
        fnWithOldDelta =
            zoomDomainsInterpretationBy effComp effAddFnDom effMultFnDom effDom oldDelta fn
    
    (.</>.) = ArithInOut.divOutEff effDiv

    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDom effDom
--    effAbs = ArithInOut.rrEffortAbs sampleDom effDom
    effDiv = ArithInOut.fldEffortDiv sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    sampleDom = getSampleDomValue sampleFn
    ((_, sampleFn) : _) = wAndfns
    dombox = getDomainBox sampleFn
    vars = getVars dombox 

getDomainDelta1 ::
    (
     HasProjections f,
     HasConstFns f,
     CanEvaluate f,
     CanCompose f,
     RoundedFakeDerivative f,
     RefOrd.IntervalLike f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedDivide f Int,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Domain f)
    )
    =>
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    FakeDerivativeEffortIndicator f ->
    ArithInOut.AbsEffortIndicator f ->
    NumOrd.MinmaxInOutEffortIndicator f ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    [(Domain f, f)] ->
    Domain f
getDomainDelta1 effComp effEval effDeriv  _effAbsFn _effMinmaxFn effAddFnDom effMultFnDom effDom wAndfns =
    {- 
        The constraint for delta depends on delta.  We try to obtain
        a delta that satisfies the constraint by iterating the
    -}
    getPeak (100 :: Int) $ iterate computeDelta initDelta
    where
    initDelta = zero sampleDom
    getPeak maxTrials (delta1 : rest@(delta2 : _)) 
        | maxTrials <= 0 = 
            error $ "aern-ivp: getDomainDelta: failed to find delta for wAndfns = " ++ show wAndfns
        | detectedNotIncreasing =
            delta1
        | otherwise =
            getPeak (maxTrials - 1) rest
        where
        detectedNotIncreasing = 
--            unsafePrint
--            (
--                "ShrinkWrap: getDomainDelta: getPeak: delta1 = " ++ show delta1
--            ) $
            (delta2 <=? delta1) == Just True
    computeDelta oldDelta =
        {-
            max_{i=1..n} w_i / ( max_{j=1..n} |fn_i/dx_j|)
            
            The norm of the derivative is taken over domains extended
            by oldDelta.
        -}
        foldl1 (NumOrd.maxOutEff effMinmax) $  map (getFnDelta oldDelta) wAndfns
    getFnDelta oldDelta (w, fn) =
--        unsafePrint
--        (
--            "ShrinkWrap: getDomainDelta1: getFnDelta:" 
--                ++ "\n fn = " ++ show fn
--                ++ "\n fnWithOldDelta = " ++ show fnWithOldDelta
--                ++ "\n derivatives = " ++ show derivatives
--                ++ "\n effEval = " ++ show effEval
--                ++ "\n slopes = " ++ show slopes
--                ++ "\n maxSlope = " ++ show maxSlope
--                ++ "\n fnDelta = " ++ show fnDelta
--        ) $
        snd $ RefOrd.getEndpointsOut $
        fnDelta
        where
        fnDelta = w .</>. maxSlope 
        maxSlope = foldl1 (NumOrd.maxOutEff effMinmax) slopes 
        slopes = map getSlope derivatives
        fnWithOldDelta =
            zoomDomainsInterpretationBy effComp effAddFnDom effMultFnDom effDom oldDelta fn
        getSlope derivative =
            ArithInOut.absOutEff effAbs $
                evalAtPointOutEff effEval dombox $
                    derivative
        derivatives = map getDeriv vars
        getDeriv var =
            fakePartialDerivativeOutEff effDeriv fnWithOldDelta var
    
    (.</>.) = ArithInOut.divOutEff effDiv

    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDom effDom
    effAbs = ArithInOut.rrEffortAbs sampleDom effDom
    effDiv = ArithInOut.fldEffortDiv sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    sampleDom = getSampleDomValue sampleFn
    ((_, sampleFn) : _) = wAndfns
    dombox = getDomainBox sampleFn
    vars = getVars dombox 

zoomDomainsInterpretationBy ::
    (
     HasProjections f,
     HasConstFns f,
     CanEvaluate f,
     CanCompose f,
     RoundedFakeDerivative f,
     RefOrd.IntervalLike f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Domain f)
    )
    =>
    CompositionEffortIndicator f ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.MixedMultEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) 
    ->
    Domain f 
    ->
    f
    ->
    f
zoomDomainsInterpretationBy effComp effAddFnDom effMultFnDom effDom delta fn =
    composeVarsOutEff effComp expansionBox fn
    where
    expansionBox =
        fromAscList $ map makeExpansion $ toAscList dombox
    makeExpansion (var, dom) =
        (var, (varFn ~<*>. slope) ~<+>. shift)
        where
        varFn = newProjectionFromSample fn var
        slope = (one sampleDom) .<+>. ((delta .<+>. delta) .</>. w)
        shift = (neg delta) .<*>. ((l .<+>. r) .</>. w)
        w = r .<->. l
        (l,r) = RefOrd.getEndpointsOut dom
    dombox = getDomainBox fn
    
    (.<+>.) = ArithInOut.addOutEff effAdd
    (.<->.) = ArithInOut.subtrOutEff effAdd
    (.<*>.) = ArithInOut.multOutEff effMult
    (.</>.) = ArithInOut.divOutEff effDiv

    (~<+>.) = ArithInOut.mixedAddOutEff effAddFnDom
    (~<*>.) = ArithInOut.mixedMultOutEff effMultFnDom
    
    effAdd = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effMult = ArithInOut.fldEffortMult sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDiv = ArithInOut.fldEffortDiv sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    sampleDom = getSampleDomValue fn
    