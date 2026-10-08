{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Events.Bisection
    Description :  hybrid system simulation  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Hybrid system simulation with time bisection.
-}

module Numeric.AERN.IVP.Solver.Events.Bisection
(
    solveHybridIVP_UsingPicardAndEventTree_Bisect
)
where

import Numeric.AERN.IVP.Solver.Events.EventTree

import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Solver.Bisection

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import Numeric.AERN.RealArithmetic.Measures
--import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsDefaultEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

import qualified Data.Map as Map
--import qualified Data.Set as Set
--import qualified Data.List as List

import Numeric.AERN.Misc.Debug
_ = unsafePrint

solveHybridIVP_UsingPicardAndEventTree_Bisect ::
    (CanAddVariables f,
     CanRenameVariables f,
     CanEvaluate f,
     CanCompose f,
     CanPartiallyEvaluate f,
     HasProjections f,
     HasConstFns f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     NumOrd.RefinementRoundedLattice f,
     RefOrd.PartialComparison f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f),
     Domain f ~ Imprecision (Domain f),
     solvingInfo ~ (Domain f, Maybe (HybridSystemUncertainState (Domain f)), [(HybSysMode, EventInfo f)]),
     Show f, Show (Domain f), Show (Var f), Eq (Var f))
    =>
    SizeLimits f {-^ size limits for all function -} ->
    PartialEvaluationEffortIndicator f ->
    CompositionEffortIndicator f ->
    EvaluationEffortIndicator f ->
    IntegrationEffortIndicator f ->
    RefOrd.PartialCompareEffortIndicator f ->
    ArithInOut.AddEffortIndicator f ->
    ArithInOut.MultEffortIndicator f ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    Domain f {-^ initial widening @delta@ -}  ->
    Int {-^ @m@ -} -> 
    Var f {-^ @t0@ - the initial time variable -} ->
    Domain f {-^ min step size @s@ -} -> 
    Domain f {-^ max step size @s@ -} -> 
    Imprecision (Domain f) {-^ split improvement threshold @eps@ -} ->
    HybridIVP f
    ->
    (
        Maybe (HybridSystemUncertainState (Domain f))
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe (Imprecision (Domain f)))
        )
    )
solveHybridIVP_UsingPicardAndEventTree_Bisect
        sizeLimits effPEval effCompose effEval effInteg effInclFn effAddFn effMultFn effAddFnDom effDom
            delta m t0Var minStepSize maxStepSize splitImprovementThreshold
                hybivpG
    = solve hybivpG
    where
    solve hybivp =
        solveHybridIVPByBisectingT
            directSolver
                effDom splitImprovementThreshold minStepSize maxStepSize
                    hybivp

    directSolver depth hybivp =
        (maybeFinalStateWithInvariants, (tEnd, maybeFinalStateWithInvariants, modeEventInfoList))
        where
        tEnd = hybivp_tEnd hybivp
        maybeFinalStateWithInvariants
            = fmap filterInvariants maybeFinalState
            where
            filterInvariants st =
                Map.mapWithKey filterInvariantsVec st
                where
                filterInvariantsVec mode vec =
                    case invariant vec of
                        Just res -> res
                        _ -> error $ 
                                "mode invariant failed on a value passed between two segments:"
                                ++ "\n mode = " ++ show mode
                                ++ "\n vec = " ++ show vec
                    where
                    Just invariant =
                        Map.lookup mode modeInvariants
        modeInvariants = hybsys_modeInvariants $ hybivp_system hybivp
        (maybeFinalState, modeEventInfoList) = 
            solveHybridIVP_UsingPicardAndEventTree
                sizeLimits effPEval effCompose effEval effInteg effInclFn effAddFn effMultFn effAddFnDom effDom
                    (10 + 2 * depth)
                        delta m
                            t0Var
                                hybivp

