{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-  
    Module      :  Numeric.AERN.RmToRn.Interval
    Description :  interval of functions as a function approximation
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Interval of functions as a function approximation.
    Assuming that both endpoint functions have the same domain and size limits.
-}
module Numeric.AERN.RmToRn.Interval where

import Numeric.AERN.Basics.Interval

import Numeric.AERN.RmToRn

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.NumericOrder as NumOrd

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.Operators

import Numeric.AERN.Basics.ShowInternals
--import Numeric.AERN.Basics.SizeLimits

instance
    (HasDomainBox f)
    =>
    (HasDomainBox (Interval f))
    where
    type (Var (Interval f)) = Var f
    type (Domain (Interval f)) = Domain f
    type (VarBox (Interval f)) = VarBox f
    getSampleDomValue (Interval l _) = getSampleDomValue l
    defaultDomSplit (Interval l _) = defaultDomSplit l
    getDomainBox (Interval l _) = getDomainBox l
    getVarDoms (Interval l _) = getVarDoms l
    getNSamplesFromInsideDomainBox (Interval l _) = getNSamplesFromInsideDomainBox l
    getSampleFromInsideDomainBox (Interval l _) = getSampleFromInsideDomainBox l
    
instance
    (HasDomainBox f)
    =>
    (HasDomainBox (IntervalApprox f))
    where
    type (Var (IntervalApprox f)) = Var f
    type (Domain (IntervalApprox f)) = Domain f
    type (VarBox (IntervalApprox f)) = VarBox f
    getSampleDomValue (IntervalApprox o _) = getSampleDomValue o
    defaultDomSplit (IntervalApprox o _) = defaultDomSplit o
    getDomainBox (IntervalApprox o _) = getDomainBox o
    getVarDoms (IntervalApprox o _) = getVarDoms o
    getNSamplesFromInsideDomainBox (IntervalApprox o _) = getNSamplesFromInsideDomainBox o
    getSampleFromInsideDomainBox (IntervalApprox o _) = getSampleFromInsideDomainBox o
    
    
instance
    CanAdjustDomains f
    => 
    CanAdjustDomains (Interval f)
    where
    adjustDomain (Interval l r) var dom = Interval lN rN
         where
         lN = adjustDomain l var dom
         rN = adjustDomain r var dom
    
instance
    CanAdjustDomains f
    => 
    CanAdjustDomains (IntervalApprox f)
    where
    adjustDomain (IntervalApprox o i) var dom = IntervalApprox oN iN
         where
         oN = adjustDomain o var dom
         iN = adjustDomain i var dom
    
instance 
    (HasProjections f,
     RefOrd.IntervalLike f)
    => 
    HasProjections (Interval f)
    where
    newProjection sizeLims dombox vars = Interval l r
        where
        (l,r) = RefOrd.getEndpointsOut p
        p = newProjection sizeLims dombox vars

instance 
    (HasProjections f,
     RefOrd.IntervalLike f)
    => 
    HasProjections (IntervalApprox f)
    where
    newProjection sizeLims dombox vars = IntervalApprox o i
        where
        i = o -- TODO: this is wrong with degree 0, introduce newProjectionOut and newProjectionIn?
        o = newProjection sizeLims dombox vars

instance
    (HasConstFns f,
     RefOrd.IntervalLike f) 
    => 
    HasConstFns (Interval f) 
    where
    newConstFn sizeLims dombox value = Interval l r
        where
        (l,r) = RefOrd.getEndpointsOut p
        p = newConstFn sizeLims dombox value

instance
    (HasConstFns f,
     RefOrd.IntervalLike f) 
    => 
    HasConstFns (IntervalApprox f) 
    where
    newConstFn sizeLims dombox value = IntervalApprox o i
        where
        i = o
        o = newConstFn sizeLims dombox value

instance 
    CanAddVariables f 
    => 
    CanAddVariables (Interval f) 
    where
    addVariablesFront varDoms (Interval l r) = Interval lN rN
        where
        lN = addVariablesFront varDoms l
        rN = addVariablesFront varDoms r
    addVariablesBack varDoms (Interval l r) = Interval lN rN
        where
        lN = addVariablesBack varDoms l
        rN = addVariablesBack varDoms r

instance 
    CanRenameVariables f
    => 
    CanRenameVariables (Interval f) 
    where
    renameVar old new (Interval l r) = Interval lN rN
        where
        lN = renameVar old new l
        rN = renameVar old new r
    renameVars renVar (Interval l r) = Interval lN rN
        where
        lN = renameVars renVar l
        rN = renameVars renVar r
    

instance
    (CanEvaluate f, 
     RefOrd.IntervalLike (Domain f)) 
    =>
    (CanEvaluate (Interval f))
    where
    type EvaluationEffortIndicator (Interval f) =
        (EvaluationEffortIndicator f, RefOrd.FromEndpointsEffortIndicator (Domain f))
    evaluationDefaultEffort (Interval l _) = 
        (evaluationDefaultEffort l, RefOrd.fromEndpointsDefaultEffort sampleDom)
        where
        sampleDom = getSampleDomValue l 
    evalAtPointOutEff (effEval, effFromE) dombox (Interval l r) =
        RefOrd.fromEndpointsOutEff effFromE (lVal, rVal)
        where
        lVal = evalAtPointOutEff effEval dombox l
        rVal = evalAtPointOutEff effEval dombox r
    evalAtPointInEff (effEval, effFromE) dombox (Interval l r) =
        RefOrd.fromEndpointsInEff effFromE (lVal, rVal)
        where
        lVal = evalAtPointOutEff effEval dombox l
        rVal = evalAtPointOutEff effEval dombox r

instance
    (CanEvaluate f, 
     RefOrd.IntervalLike (Domain f)) 
    =>
    (CanEvaluate (IntervalApprox f))
    where
    type EvaluationEffortIndicator (IntervalApprox f) =
        (EvaluationEffortIndicator f, RefOrd.FromEndpointsEffortIndicator (Domain f))
    evaluationDefaultEffort (IntervalApprox o _) =
        evaluationDefaultEffort o 
    evalAtPointOutEff eff dombox (IntervalApprox o i) =
        evalAtPointOutEff eff dombox o 
    evalAtPointInEff =
        error $ "AERN: inner rounded evalAtPointInEff not defined for IntervalApprox"

instance
    (CanPartiallyEvaluate f, 
     RefOrd.IntervalLike f) 
    =>
    (CanPartiallyEvaluate (Interval f))
    where
    type PartialEvaluationEffortIndicator (Interval f) =
        (PartialEvaluationEffortIndicator f, 
         RefOrd.GetEndpointsEffortIndicator f)
    partialEvaluationDefaultEffort (Interval l _) = 
        (partialEvaluationDefaultEffort l, RefOrd.getEndpointsDefaultEffort l)
    pEvalAtPointOutEff (effEval, effGetE) dombox (Interval l r) =
        Interval lVal rVal
        where
        (lVal, _) = RefOrd.getEndpointsOutEff effGetE $ pEvalAtPointOutEff effEval dombox l
        (_, rVal) = RefOrd.getEndpointsOutEff effGetE $ pEvalAtPointOutEff effEval dombox r
    pEvalAtPointInEff (effEval, effGetE) dombox (Interval l r) =
        Interval lVal rVal
        where
        (_, lVal) = RefOrd.getEndpointsOutEff effGetE $ pEvalAtPointInEff effEval dombox l
        (rVal, _) = RefOrd.getEndpointsOutEff effGetE $ pEvalAtPointInEff effEval dombox r



instance
    CanEvaluateOtherType f
    => 
    CanEvaluateOtherType (Interval f)
    where
    type EvalOps (Interval f) = EvalOps f
    evalOtherType evalOps valuesBox (Interval l r) =
        RefOrd.fromEndpointsOut $ (lN, rN)
        where
        lN = evalOtherType evalOps valuesBox l
        rN = evalOtherType evalOps valuesBox r

instance
    CanEvaluateOtherTypeInner f
    => 
    CanEvaluateOtherTypeInner (Interval f)
    where
    evalOtherTypeInner evalOps valuesBox (Interval l r) =
        RefOrd.fromEndpointsIn $ (lN, rN)
        where
        lN = evalOtherTypeInner evalOps valuesBox l
        rN = evalOtherTypeInner evalOps valuesBox r

instance 
    HasEvalOps f t
    => 
    HasEvalOps (Interval f) t
    where
    type EvalOpsEffortIndicator (Interval f) t =
        EvalOpsEffortIndicator f t
    evalOpsDefaultEffort (Interval l _) sampleT = 
        evalOpsDefaultEffort l sampleT
    evalOpsEff eff (Interval l _) sampleT =
        evalOpsEff eff l sampleT 


instance
    (CanEvaluateOtherTypeInner f,
     RefOrd.IntervalLike f, HasProjections f,
     HasEvalOps f (Interval f),
     HasVarValue (VarBox f (Interval f)) (Var f) (Interval f),
     NumOrd.PartialComparison f,
     NumOrd.RoundedLatticeEffort f,
     ShowInternals f
    )
    => 
    CanCompose (Interval f)
    where
    type CompositionEffortIndicator (Interval f) =
        (EvalOpsEffortIndicator (Interval f) (Interval f))
    compositionDefaultEffort i =
        (evalOpsDefaultEffort i i)
    composeVarsOutEff effOps valueBox i =
        evalOtherType (evalOpsEff effOps i i) valueBox i
    composeVarsInEff effOps valueBox i =
        evalOtherTypeInner (evalOpsEff effOps i i) valueBox i


instance 
    (RoundedIntegration f,
     RefOrd.IntervalLike f) 
    => 
    RoundedIntegration (Interval f) 
    where
    type IntegrationEffortIndicator (Interval f) =
        (IntegrationEffortIndicator f,
         RefOrd.GetEndpointsEffortIndicator f)
    integrationDefaultEffort (Interval l _) =
        (integrationDefaultEffort l,
         RefOrd.getEndpointsDefaultEffort l)
    primitiveFunctionOutEff (effInteg, effGetE) (Interval l r) var = Interval lN rN
        where
        (lN, _) = RefOrd.getEndpointsOutEff effGetE $ primitiveFunctionOutEff effInteg l var 
        (_, rN) = RefOrd.getEndpointsOutEff effGetE $ primitiveFunctionOutEff effInteg r var 
    primitiveFunctionInEff (effInteg, effGetE) (Interval l r) var = Interval lN rN
        where
        (_, lN) = RefOrd.getEndpointsOutEff effGetE $ primitiveFunctionInEff effInteg l var 
        (rN, _) = RefOrd.getEndpointsOutEff effGetE $ primitiveFunctionInEff effInteg r var 

instance 
    (RoundedFakeDerivative f,
     RefOrd.IntervalLike f)
    =>
    RoundedFakeDerivative (Interval f)
    where
    type FakeDerivativeEffortIndicator (Interval f) = 
        (FakeDerivativeEffortIndicator f,
         RefOrd.GetEndpointsEffortIndicator f)
    fakeDerivativeDefaultEffort (Interval l _) =
        (fakeDerivativeDefaultEffort l, RefOrd.getEndpointsDefaultEffort l)
    fakePartialDerivativeOutEff (effDeriv, effGetE) (Interval l r) var = Interval lN rN
        where
        (lN, _) = RefOrd.getEndpointsOutEff effGetE $ fakePartialDerivativeOutEff effDeriv l var 
        (_, rN) = RefOrd.getEndpointsOutEff effGetE $ fakePartialDerivativeOutEff effDeriv r var
    fakePartialDerivativeInEff (effDeriv, effGetE) (Interval l r) var = Interval lN rN
        where
        (_, lN) = RefOrd.getEndpointsOutEff effGetE $ fakePartialDerivativeInEff effDeriv l var 
        (rN, _) = RefOrd.getEndpointsOutEff effGetE $ fakePartialDerivativeInEff effDeriv r var
        
instance
    (
        ArithInOut.RoundedReal (Interval f),
        HasEvalOps f (Interval f),
        CanEvaluate f,
        CanEvaluateOtherTypeInner f,
        RefOrd.IntervalLike f, 
        HasProjections f,
        HasConstFns f,
        NumOrd.PartialComparison f,
        RoundedIntegration f,
        ArithInOut.RoundedReal (Domain f),
        RefOrd.IntervalLike (Domain f),
        HasVarValue (VarBox f (Interval f)) (Var f) (Interval f),
        GeneratableVariables (Var f),
        ShowInternals f   
    )
    =>
    RoundedRealFn (Interval f)
    where
    -- TODO
    
    