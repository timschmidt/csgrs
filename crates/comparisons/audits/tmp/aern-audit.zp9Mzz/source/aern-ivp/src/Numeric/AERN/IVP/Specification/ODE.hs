{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Specification.ODE
    Description :  specification of ODE IVP with uncertainty
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Specification of ODE IVP with uncertainty.
-}

module Numeric.AERN.IVP.Specification.ODE
--(
--)
where

import Numeric.AERN.RmToRn
--
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.ExactOps
--
--import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.Operators
--
import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.Operators

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.SizeLimits

import Numeric.AERN.Misc.Debug
_ = unsafePrint

data ODEIVP f =
    ODEIVP
    {
        odeivp_description :: String
    ,
        odeivp_componentNames :: [Var f]
    , 
        odeivp_field :: [f] -> [f]
    ,
        odeivp_tVar :: Var f -- ^ @tVar@
    ,
        odeivp_tStart :: Domain f -- ^ @tStart@
    ,
        odeivp_tEnd :: Domain f -- ^ @tEnd@
    ,
        odeivp_makeInitialValueFnVec :: ODEInitialValues f 
        {-^ the parameters are:
            * size limits for the function
            * initial time variable @t0@ 
            * time domain @T0@ 
            the result are functions whose first variable is @t0 : T0@ 
            - the initial value may depend on initial time
        -} 
    ,
        odeivp_t0End :: Domain f 
        {-^ 
            @t0End@ -- initial time uncertainty; 
            must satisfy @tStart <= t0End <= tEnd@;
            with exact initial time, we have @t0End = tStart@
        -}
    ,
        odeivp_intersectDomain :: [Domain f] -> Maybe [Domain f]
    ,
        odeivp_valuePlotExtents :: [(Domain f, Domain f)]
    ,
        odeivp_enclosureRangeWidthLimit :: Domain f
    ,
        odeivp_maybeExactValuesAtTEnd :: Maybe [Domain f]
    }

type ODEInitialValues f =
    SizeLimits f -> Var f -> Domain f -> [f]

makeFnVecFromInitialValues ::
     (Show f, Show (Domain f), Show (Var f),
      HasSizeLimits f,
      HasProjections f) 
      =>
     [Var f] {-^ names for solution vector components -} -> 
     [Domain f] {-^ initial values, one for each component -}
     ->
     SizeLimits f {-^ size limits to use in new function -} ->
     Var f {-^ @tVar@ - time variable -} -> 
     Domain f {-^ time domain  -} 
     -> 
     [f]
makeFnVecFromInitialValues componentNames initialValues sizeLimits t0Var t0Domain =
--    unsafePrint
--    (
--        "makeFnVecFromInitialValues:"
--        ++ "\n componentNames = " ++ show componentNames
--        ++ "\n initialValues = " ++ show initialValues
----        ++ "\n sizeLimits = " ++ show sizeLimits
--        ++ "\n tVar = " ++ show tVar
--        ++ "\n timeDomain = " ++ show timeDomain
--        ++ "\n result = " ++ show initialValuesFnVec
--    ) $
    initialValuesFnVec
    where    
    initialValuesFnVec =
        map initialValueFn componentNames
    initialValueFn = 
        newProjection sizeLimits varDoms
    varDoms =
        (t0Var, t0Domain) : zip componentNames initialValues

parameteriseInitialValues ::
     (Show f, Show (Domain f), Show (Var f),
      HasProjections f) 
      =>
     SizeLimits f -> 
     [Var f] {-^ names for solution vector components -} -> 
     [Domain f] {-^ initial values, one for each component -}
     ->
     [f]
parameteriseInitialValues sizeLimits componentNames initialValues =
--    unsafePrint
--    (
--        "makeFnVecFromInitialValues:"
--        ++ "\n componentNames = " ++ show componentNames
--        ++ "\n initialValues = " ++ show initialValues
----        ++ "\n sizeLimits = " ++ show sizeLimits
--        ++ "\n tVar = " ++ show tVar
--        ++ "\n timeDomain = " ++ show timeDomain
--        ++ "\n result = " ++ show initialValuesFnVec
--    ) $
    initialValuesFnVec
    where    
    initialValuesFnVec =
        map initialValueFn componentNames
    initialValueFn = 
        newProjection sizeLimits varDoms
    varDoms =
        zip componentNames initialValues


makeFnVecFromParamInitialValuesOut ::
     (Show f, Show (Domain f), Show (Var f),
      ArithInOut.RoundedReal (Domain f),
      ArithInOut.RoundedAdd f,
      ArithInOut.RoundedSubtr f,
      ArithInOut.RoundedMultiply f,
      RefOrd.IntervalLike f,
      RefOrd.IntervalLike (Domain f),
      HasConstFns f,  HasProjections f,
      CanAddVariables f, CanChangeSizeLimits f) 
      =>
     (ArithInOut.AddEffortIndicator f) ->
     (ArithInOut.MultEffortIndicator f) ->
     (SizeLimitsChangeEffort f)
     ->
     [Var f] {-^ names for solution vector components -} -> 
     [f] {-^ parametrised initial values, one for each component -}
     ->
     SizeLimits f {-^ size limits to use in new function - ignored here -} ->
     Var f {-^ @tVar@ - time variable -} -> 
     Domain f {-^ time domain  -} 
     -> 
     [f]
makeFnVecFromParamInitialValuesOut effAddFn effMultFn effSizeLims componentNames initialValueFnVec sizeLimits t0Var t0Domain =
--    unsafePrint
--    (
--        "makeFnVecFromParamInitialValuesOut:"
--        ++ "\n initialValueFnVec = " ++ show initialValueFnVec
--        ++ "\n result = " ++ show result
--    )
    result
    where
    result =
        map updateFn initialValueFnVec
    updateFn initValFn =
        initValFnWithNewVars -- just adding a time variable
        where
        initValFnWithNewVars =
            changeSizeLimitsOutEff effSizeLims sizeLimits $
                addVariablesFront newVarDoms initValFn
    newVarDoms =
        [(t0Var, t0Domain)] -- ++ (zip componentNames $ repeat unitDom)
    -- get rid of unused  warnings but keep the params in case thinning is needed in future:
    _ = componentNames
    _ = effAddFn
    _ = effMultFn

parametriseThickFunctions ::
    (RefOrd.IntervalLike f,
     RefOrd.IntervalLike (Domain f),
     HasProjections f,
     HasConstFns f,
     CanAddVariables f,
     HasOne (Domain f),
     HasZero (Domain f),
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f) 
    =>
    ArithInOut.AddEffortIndicator f -> 
    ArithInOut.MultEffortIndicator f 
    -> 
    [Var f] -> [f] -> [f]
parametriseThickFunctions effAddFn effMultFn componentNames fnVec =
    map parametriseFn $ zip componentNames fnVec
    where
    parametriseFn (compName, fn) =
        (fnL ~<*>~ compVar) ~<+>~ (fnR ~<*>~ (c1 ~<->~ compVar)) -- thinning
        where
        compVar = makeVar compName
        (fnL, fnR) = RefOrd.getEndpointsOut fnWithNewVars
        fnWithNewVars =
            addVariablesFront newVarDoms fn
        
    c1 = 
        newConstFnFromSample sampleFnOutgoing (one sampleDom) 
    makeVar =
        newProjectionFromSample sampleFnOutgoing

    sampleFnOutgoing =
        addVariablesFront newVarDoms sampleFnIncoming 

    newVarDoms =
        (zip componentNames $ repeat unitDom)
        where
        unitDom = RefOrd.fromEndpointsOut (zero sampleDom, one sampleDom)
    
    (~<+>~) = ArithInOut.addOutEff effAddFn
    (~<->~) = ArithInOut.subtrOutEff effAddFn
    (~<*>~) = ArithInOut.multOutEff effMultFn

    sampleFnIncoming : _ = fnVec
    sampleDom = getSampleDomValue sampleFnIncoming

evalAtEndTimeOutInVec ::
    (Show (Domain f), Show f,
     CanEvaluate f,
     RefOrd.IntervalLike f,
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f))
    =>
    EvaluationEffortIndicator f ->
    Var f -> 
    Domain f -> 
    [(f,f)] 
    -> 
    ([Domain f], [Domain f])
evalAtEndTimeOutInVec effEval tVar tEnd fnVec =
    unzip $ map (evalAtEndTimeOutInFn effEval tVar tEnd) fnVec
    
evalAtEndTimeOutInFn ::
    (Show (Domain f), Show f,
     CanEvaluate f,
     RefOrd.IntervalLike f,
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f))
    =>
    EvaluationEffortIndicator f ->
    Var f -> 
    Domain f -> 
    (f, f)
    -> 
    (Domain f, Domain f)
evalAtEndTimeOutInFn effEval tVar tEnd (fnOut, fnIn) =
--    unsafePrint
--    (
--        "evalAtEndTimeOutInFn:"
--        ++ "\n fnOut = " ++ show fnOut
--        ++ "\n fnIn = " ++ show fnIn
--        ++ "\n at tEnd = " ++ show tEnd
--        ++ "\n valueOut = " ++ show valueOut
--        ++ "\n valueIn = " ++ show valueIn
--        ++ "\n valueLR = " ++ show valueLR
--        ++ "\n valueRL = " ++ show valueRL
--    ) $
    (valueOut, valueIn)
    where
    valueOut =
        evalAtPointOutEff effEval endTimeArea fnOut
    valueIn =
        RefOrd.fromEndpointsOut (valueRL, valueLR)
    (_, valueLR) =
        RefOrd.getEndpointsOut valueBelowR
    (valueRL, _) =
        RefOrd.getEndpointsOut valueAboveL
    valueBelowR =
        evalAtPointInEff effEval endTimeArea fnBelowR
    valueAboveL =
        evalAtPointInEff effEval endTimeArea fnAboveL
    (fnAboveL, fnBelowR) = RefOrd.getEndpointsOut fnIn
--    endTimeArea :: DomainBox f
    endTimeArea = insertVar tVar tEnd $ getDomainBox fnOut

        
evalAtEndTimeVec ::
    (Show (Domain f), Show f,
     CanEvaluate f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f))
    =>
    EvaluationEffortIndicator f ->
    Var f -> 
    Domain f -> 
    [f] 
    -> 
    ([Domain f], [Domain f])
evalAtEndTimeVec effEval tVar tEnd fnVec =
    unzip $ map (evalAtEndTimeFn effEval tVar tEnd) fnVec
    
evalAtEndTimeFn ::
    (Show (Domain f), Show f,
     CanEvaluate f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f))
    =>
    EvaluationEffortIndicator f ->
    Var f -> 
    Domain f -> 
    f
    -> 
    (Domain f, Domain f)
evalAtEndTimeFn effEval tVar tEnd fn =
    evalAtEndTimeOutInFn effEval tVar tEnd (fn, flipConsistency fn)

partiallyEvalAtEndTimeOutInVec ::
    (Show (Domain f), Show f,
     CanPartiallyEvaluate f)
    =>
    Var f -> 
    Domain f -> 
    ([f],[f])
    -> 
    ([f],[f])
partiallyEvalAtEndTimeOutInVec tVar tEnd fnVec =
    unzip $ map (partiallyEvalAtEndTimeOutInFn tVar tEnd) $ uncurry zip fnVec

partiallyEvalAtEndTimeOutInFn ::
    (Show (Domain f), Show f,
     CanPartiallyEvaluate f)
    =>
    Var f -> 
    Domain f -> 
    (f, f)
    -> 
    (f, f)
partiallyEvalAtEndTimeOutInFn tVar tEnd (fnOut, fnIn) =
--    unsafePrint
--    (
--        "partiallyEvalAtEndTimeOutInFn:"
--        ++ "\n fnOut = " ++ show fnOut
--        ++ "\n fnIn = " ++ show fnIn
--        ++ "\n at tEnd = " ++ show tEnd
--        ++ "\n paramValueOut = " ++ show paramValueOut
--        ++ "\n paramValueIn = " ++ show paramValueIn
--    ) $
    (paramValueOut, paramValueIn)
    where
    paramValueOut =
        pEvalAtPointOutEff (partialEvaluationDefaultEffort fnOut) endTimeArea fnOut
    paramValueIn =
        pEvalAtPointInEff (partialEvaluationDefaultEffort fnIn) endTimeArea fnIn
    endTimeArea = fromList [(tVar, tEnd)]
