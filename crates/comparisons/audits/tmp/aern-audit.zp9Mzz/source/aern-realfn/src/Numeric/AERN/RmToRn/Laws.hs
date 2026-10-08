{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Laws
    Description :  auxiliary functions for testable properties  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Auxiliary functions for testable properties.
-}

module Numeric.AERN.RmToRn.Laws where

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.ShowInternals

--import Numeric.AERN.Misc.QuickCheck
import Numeric.AERN.Misc.Debug
--import Numeric.AERN.Misc.Bool
--import Numeric.AERN.Misc.Maybe

import Test.Framework (Test, testGroup)
import Test.Framework.Providers.QuickCheck2 (testProperty)

{-|
    Property declaring that the given full-function binary operator
    is compatible with the given point binary operator.
-}
roundedAgreeWithPointwiseOp2 :: 
    (CanEvaluate f,
     RefOrd.PartialComparison (Domain f),
     ShowInternals f, Show f, Show (Domain f),
     Show (DomainBox f)) 
    =>
    f 
        {-^ sample function (only for type checking) -} -> 
    String 
        {-^ context description to use in reporting -} -> 
    (effFn -> f -> f -> f) 
        {-^ the binary op - a whole function version - inner-rounded -} -> 
    (effFn -> f -> f -> f) 
        {-^ the binary op - a whole function version - outer-rounded -} -> 
    (effPt -> Domain f -> Domain f -> Domain f)
        {-^ the binary op - a point version - inner-rounded -} -> 
    (effPt -> Domain f -> Domain f -> Domain f)
        {-^ the binary op - a point version - outer-rounded -} -> 
    (f, f)
        {-^ pair of functions to test the property on -} -> 
    (SingletonInArea (DomainBox f)) 
        {-^ a domain point to test the operation on -} -> 
    effFn 
        {-^ effort indicator for the predicate - function version -}  ->
    effPt 
        {-^ effort indicator for the predicate - point version -}  ->
    EvaluationEffortIndicator f 
        {-^ function evaluation effort parameter -} ->
    (RefOrd.PartialCompareEffortIndicator (Domain f)) 
        {-^ effort indicator for containment checking of results -} ->
    Bool
roundedAgreeWithPointwiseOp2 _ contextDescription 
        fnOpInEff fnOpOutEff 
        ptOpInEff ptOpOutEff 
        (fn1, fn2) 
        (SingletonInArea box) -- TODO: replace the single point by a list of points to get better coverage 
        effFnOp effPtOp effEval effContain
    | passed = passed
    | otherwise =
            unsafePrint
            (
                contextDescription ++ ": pointwise check failed for: " 
                ++ "\n fn1 = " ++ showUsingShowInternals fn1
                ++ "\n fn2 = " ++ showUsingShowInternals fn2
                ++ "\n box = " ++ show box
                ++ "\n pt1In = " ++ show pt1In
                ++ "\n pt1Out = " ++ show pt1Out
                ++ "\n pt2In = " ++ show pt2In
                ++ "\n pt2Out = " ++ show pt2Out
                ++ "\n resPtIn = " ++ show resPtIn
                ++ "\n resPtOut = " ++ show resPtOut
                ++ "\n resFnAtPtIn = " ++ show resFnAtPtIn
                ++ "\n resFnAtPtOut = " ++ show resFnAtPtOut
                ++ "\n resFnIn = " ++ show resFnIn
                ++ "\n resFnOut = " ++ show resFnOut
            )
            False
    where
    passed = 
        case (resPtOut `contains` resFnAtPtIn, resFnAtPtOut `contains` resPtIn) of
            (Just False, _) -> False
            (_, Just False) -> False
            _ -> True
        where
        contains = RefOrd.pLeqEff effContain 
        
    -- evaluate operand functions on box and then execute operation:
    pt1Out = evalAtPointOutEff effEval box fn1
    pt2Out = evalAtPointOutEff effEval box fn2
    resPtOut = ptOpOutEff effPtOp pt1Out pt2Out
    
    pt1In = evalAtPointInEff effEval box fn1
    pt2In = evalAtPointInEff effEval box fn2
    resPtIn = ptOpInEff effPtOp pt1In pt2In

    -- execute operation on whole functions and then evaluate them on box:
    resFnOut = fnOpOutEff effFnOp fn1 fn2
    resFnAtPtOut = evalAtPointOutEff effEval box resFnOut
    
    resFnIn = fnOpInEff effFnOp fn1 fn2
    resFnAtPtIn = evalAtPointInEff effEval box resFnIn


{-|
    Property declaring that if the given binary predicate holds on the whole domain,
    it holds also on each point of the domain.
-}
roundedAgreeWithPointwisePred2All :: 
    (CanEvaluate f,
     Show f,
     Show (DomainBox f)) 
    =>
    f 
        {-^ sample function (only for type checking) -} -> 
    String 
        {-^ context description to use in reporting -} -> 
    (effFn -> f -> f -> Maybe Bool) 
        {-^ the binary predicate - a whole function version -} -> 
    (effPt -> Domain f -> Domain f -> Maybe Bool)
        {-^ the binary predicate - a domain point version -} -> 
    (f, f)
        {-^ pair of functions to test the property on -} -> 
    (SingletonInArea (DomainBox f)) 
        {-^ a domain point to test the property on -} -> 
    effFn 
        {-^ effort indicator for the predicate - function version -}  ->
    effPt 
        {-^ effort indicator for the predicate - point version -}  ->
    EvaluationEffortIndicator f 
        {-^ function evaluation effort parameter -} ->
    Bool
roundedAgreeWithPointwisePred2All _ contextDescription fnOpEff ptOpEff (fn1, fn2) (SingletonInArea box) effFnOp effPtOp effEval =
    case (mresultFn, mresultPt) of
        (Just True, Just False) ->
            -- if it is true on all points, it cannot be false on this box
--        (Just resFn, Just resPt) | resFn /= resPt ->
            unsafePrint
            (
                contextDescription ++ ": pointwise check failed for: " 
                ++ "\n fn1 = " ++ show fn1
                ++ "\n fn2 = " ++ show fn2
                ++ "\n box = " ++ show box
            )
            False
        _ -> 
--            unsafePrint
--            (
--                contextDescription ++ ": pointwise check succeeded for: " 
--                ++ "\n box = " ++ show box
--                ++ "\n resultFn = " ++ show resultFn
--                ++ "\n resultPt = " ++ show resultPt
--            )
            True
    where
    mresultFn = fnOpEff effFnOp fn1 fn2
    mresultPt = ptOpEff effPtOp pt1 pt2
    pt1 = evalAtPointOutEff effEval box fn1
    pt2 = evalAtPointOutEff effEval box fn2

roundedRefinementIsotoneDom ::
    (RefOrd.PartialComparison t, 
     RefOrd.ArbitraryOrderedTuple (DomainBox f),
     Show (DomainBox f),
     Show (Domain f),
     Show t)
    =>
    f 
        {-^ sample function (only for type checking) -} -> 
    String 
        {-^ context description to use in reporting -} -> 
    (ei -> (DomainBox f) -> t) 
        {-^ operation -} ->
    (ei -> (DomainBox f) -> t) ->
    (RefOrd.LEPair (DomainBox f)) -> 
    ei -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
roundedRefinementIsotoneDom _ contextDescription exprUp exprDn (RefOrd.LEPair (domDn, domUp)) effort effortComp =
--    unsafePrint
--    (
--        "roundedRefinementIsotoneDom: "
--        ++ "\n domUp = " ++ show domUp
--        ++ "\n domDn = " ++ show domDn
--        ++ "\n resUp = " ++ show resUp
--        ++ "\n resDn = " ++ show resDn
--    ) $
    case RefOrd.pLeqEff effortComp resDn resUp of
        Just True -> True
        Just False -> 
            unsafePrint
            (
                contextDescription ++ ": roundedRefinementIsotoneDom failed for:"
                ++ "\n domUp = " ++ show domUp
                ++ "\n domDn = " ++ show domDn
                ++ "\n resUp = " ++ show resUp
                ++ "\n resDn = " ++ show resDn
            )
            False
        _ -> True
    where
    resUp = check $ exprUp effort domUp
    resDn = check $ exprDn effort domDn
    check = id -- detectIllegalValues $ contextDescription ++ " refinement isotone"
    
    
{-  -}    
propEvalRefIsotone ::
    (CanEvaluate f,
     RefOrd.PartialComparison (Domain f),
     RefOrd.ArbitraryOrderedTuple (VarBox f (Domain f)), 
     Area (VarBox f (Domain f)) ~ VarBox f (Domain  f),
     Show f,
     Show (Domain f),
     Show (DomainBox f))
    =>
    f ->
    (SingletonInArea f) -> 
    (Area (DomainBox f),
        (RefOrd.LEPair (DomainBox f)) -> 
        EvaluationEffortIndicator f ->
        (RefOrd.PartialCompareEffortIndicator (Domain f)) ->
        Bool)
propEvalRefIsotone _
        (SingletonInArea fn)
    =
--    unsafePrint
--    (
--       "propEvalRefIsotone: "
--       ++ "\n fn = " ++ show fn
--    ) $
    (domBox, roundedRefinementIsotoneDom fn "function evaluation" evalIn evalOut) 
    where
    domBox = getDomainBox fn
    evalIn eff d = evalAtPointInEff eff d fn
    evalOut eff d = evalAtPointOutEff eff d fn

testsEval ::
    (Show f,
     Show (Domain f),
     Show (VarBox f (Domain f)),
     ArbitraryWithArea f,
     RefOrd.ArbitraryOrderedTuple (VarBox f (Domain f)),
     RefOrd.PartialComparison (Domain f),
     CanEvaluate f,
     Area (DomainBox f) ~ (DomainBox f))
     =>
    ([Char], f) -> Area f -> Test
testsEval (name, sample) area =
    testGroup (name ++ " evaluation") $
        [
            testProperty "refinement isotone" (area, propEvalRefIsotone sample)
        ]
    
propNumCompareAgreesWithPointwise :: 
    (Show f,
     Show (DomainBox f),
     NumOrd.PartialComparison f,
     NumOrd.PartialComparison (Domain f),
     CanEvaluate f)
    =>
    f -> 
    PairInArea f ->
    (DomainBox f,
        (SingletonInArea (DomainBox f)) -> 
        NumOrd.PartialCompareEffortIndicator f -> 
        NumOrd.PartialCompareEffortIndicator (Domain f) -> 
        EvaluationEffortIndicator f -> 
        Bool)
propNumCompareAgreesWithPointwise sampleFn (PairInArea fns) =
    (domBox, roundedAgreeWithPointwisePred2All sampleFn  "function numerical comparison" NumOrd.pLeqEff NumOrd.pLeqEff fns)
    where
    domBox = getDomainBox sampleFn
    
testsFnNumCompare :: 
    (Show f,
     Show (DomainBox f),
     ArbitraryWithArea (DomainBox f),
     NumOrd.PartialComparison f,
     NumOrd.PartialComparison (Domain f),
     NumOrd.ArbitraryOrderedTuple f,
     CanEvaluate f,
     Area (DomainBox f) ~ (DomainBox f))
    =>
    ([Char], f) -> Area f -> Test
testsFnNumCompare (name, sample) area =
    testGroup (name ++ " numerical comparison") $
        [
            testProperty "pointwise" (area, propNumCompareAgreesWithPointwise sample)
        ]


propAddPointwise :: 
  (Show f, Show (VarBox f (Domain f)), Show (Domain f),
   ShowInternals f, 
   ArithInOut.RoundedAdd f,
   ArithInOut.RoundedAdd (Domain f),
   RefOrd.PartialComparison (Domain f), CanEvaluate f) 
  =>
  f
  -> PairInArea f
  -> (DomainBox f,
      SingletonInArea (DomainBox f)
      -> ArithInOut.AddEffortIndicator f
      -> ArithInOut.AddEffortIndicator (Domain f)
      -> EvaluationEffortIndicator f
      -> RefOrd.PartialCompareEffortIndicator (Domain f)
      -> Bool)
propAddPointwise sampleFn (PairInArea fns) =
    (domBox, 
        roundedAgreeWithPointwiseOp2 
            sampleFn "addition" 
            ArithInOut.addInEff ArithInOut.addOutEff 
            ArithInOut.addInEff ArithInOut.addOutEff 
            fns)
    where
    domBox = getDomainBox sampleFn

testsFieldPointwise ::
    (Show f, ShowInternals f,
     Show (Domain f),
     Show (VarBox f (Domain f)),
     ArbitraryWithArea f,
     ArithInOut.RoundedReal (Domain f), 
     ArithInOut.RoundedAdd f, 
     RefOrd.ArbitraryOrderedTuple (VarBox f (Domain f)),
     RefOrd.PartialComparison (Domain f),
     CanEvaluate f,
     Area (DomainBox f) ~ (DomainBox f))
     =>
    ([Char], f) -> Area f -> Test
testsFieldPointwise (name, sample) area =
    testGroup (name ++ " field ops") $
        [
            testProperty "pointwise (+)" (area, propAddPointwise sample)
        ]
    
    