{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Minmax
    Description :  approximation of min and max using only ring operations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Approximation of min and max using only ring operations.
    .
    The motivating use case for this module is where we compute min or max for a 
    /function/ pointwise over its domain.
-}

module Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Minmax where

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.RefinementOrderRounding.BernsteinPoly

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort
----import Numeric.AERN.RefinementOrder.InPlace.OpsImplicitEffort

import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.Exception
import Control.Exception
--import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.SizeLimits
--import Numeric.AERN.Basics.Mutable
--import Numeric.AERN.RealArithmetic.ExactOps

--import Test.QuickCheck

--import qualified Data.List as List

--import Control.Monad.ST (ST)

import Numeric.AERN.Misc.Debug
_ = unsafePrint

data MinmaxEffortIndicatorFromRingOps f t =
    MinmaxEffortIndicatorFromRingOps
    {
        minmaxFromRO_eff_convertTDF :: ArithUpDn.ConvertEffortIndicator t (Domain f),
            -- finding the range of a function of type t
        minmaxFromRO_eff_roundedRealD :: ArithInOut.RoundedRealEffortIndicator (Domain f),
        minmaxFromRO_eff_getEndpointsD :: RefOrd.GetEndpointsEffortIndicator (Domain f),
        minmaxFromRO_eff_evalFT :: EvalOpsEffortIndicator f t,
        minmaxFromRO_eff_evalFDF :: EvaluationEffortIndicator f,
        minmaxFromRO_eff_ringOpsF :: ArithInOut.RingOpsEffortIndicator f,
        minmaxFromRO_eff_mixmultFDF :: ArithInOut.MixedMultEffortIndicator f (Domain f),
        minmaxFromRO_eff_sizelimitsF :: SizeLimits f,
        minmaxFromRO_eff_ringOpsT :: ArithInOut.RingOpsEffortIndicator t,
        minmaxFromRO_eff_mixedFldTDF :: ArithInOut.MixedFieldOpsEffortIndicator t (Domain f)
    }


--deriving instance
--    (ArithUpDn.Convertible t (Domain f),
--     ArithInOut.RoundedReal (Domain f),
--     HasEvalOps f t, CanEvaluate f,
--     ArithInOut.RoundedRing t,
--     ArithInOut.RoundedRing f,
--     ArithInOut.RoundedMixedField f Int,
--     ArithInOut.RoundedMixedField t (Domain f),
--     Show (SizeLimits f)) 
--    =>
--    Show (MinmaxEffortIndicatorFromRingOps f t)

    
--minmaxEffortIndicatorFromRingOpsAdjustDegree ::
--    MinmaxEffortIndicatorFromRingOps f t ->
--    (Int -> Int) ->
--    MinmaxEffortIndicatorFromRingOps f t
--minmaxEffortIndicatorFromRingOpsAdjustDegree effMinmax@(a,b,(c1,c2,c3,Int1To10 degree),d) adjDegree =
--    (a,b,(c1,c2,c3, Int1To10 $ adjDegree degree),d)

defaultMinmaxEffortIndicatorFromRingOps :: 
    (ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasSizeLimits f,
     HasDomainBox f,
     HasEvalOps f t,
     CanEvaluate f,
     ArithInOut.RoundedRingEffort f,
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     ArithInOut.RoundedRingEffort t
    )
    =>
    f {-^ an arbitrary sample value of a function type used to model internal Bernstein approximations -} -> 
    t {-^ an arbitrary sample value of the main type -} -> 
    MinmaxEffortIndicatorFromRingOps f t
defaultMinmaxEffortIndicatorFromRingOps sampleF sampleT =
    MinmaxEffortIndicatorFromRingOps
    {
        minmaxFromRO_eff_convertTDF = ArithUpDn.convertDefaultEffort sampleT sampleDF,
        minmaxFromRO_eff_roundedRealD = ArithInOut.roundedRealDefaultEffort sampleDF,
        minmaxFromRO_eff_getEndpointsD = RefOrd.getEndpointsDefaultEffort sampleDF,
        minmaxFromRO_eff_evalFT = evalOpsDefaultEffort sampleF sampleT,
        minmaxFromRO_eff_evalFDF = evaluationDefaultEffort sampleF,
        minmaxFromRO_eff_ringOpsF = ArithInOut.ringOpsDefaultEffort sampleF,
        minmaxFromRO_eff_mixmultFDF = ArithInOut.mixedMultDefaultEffort sampleF sampleDF,
        minmaxFromRO_eff_sizelimitsF = getSizeLimits sampleF,
        minmaxFromRO_eff_ringOpsT = ArithInOut.ringOpsDefaultEffort sampleT,
        minmaxFromRO_eff_mixedFldTDF = ArithInOut.mixedFieldOpsDefaultEffort sampleT sampleDF
    }
    where
    sampleDF = getSampleDomValue sampleF

maxUpEffFromRingOps :: 
    (
     Show t, Show f,
     HasZero t, 
     ArithInOut.RoundedRing t,
     ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     HasEvalOps f t, 
     RefOrd.IntervalLike t,
     RefOrd.IntervalLike f,
     HasVarValue (VarBox f t) (Var f) t,
     CanEvaluate f,
     HasVarValue (VarBox f (Domain f)) (Var f) (Domain f),
     HasProjections f, HasConstFns f, HasOne f, -- HasZero f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f)) 
    =>
    f ->
    (f -> f) {- ^ Given a sample function, return the function \x:[0,1] -> x of the same type. -} ->
    MinmaxEffortIndicatorFromRingOps f t ->
    Int {- ^ degree of Bernstein approximations; must be > 1 -} ->
    t -> t -> (t, t)
maxUpEffFromRingOps sampleF getX eff degree a b =
    (upper, upperShiftedBelow)
    where
    upperShiftedBelow =
        upper #<+>. (neg errUp)
    upper =
        a #<+># maxBMinusAUp 
    ((_, maxBMinusAUp), errUp) = 
        maxZeroDnUp sampleF getX eff degree $ b #<-># a

    (#<+>#) = ArithInOut.addOutEff effAddT
    (#<->#) = ArithInOut.subtrOutEff effAddT
    (#<+>.) = ArithInOut.mixedAddOutEff effAddTDF

    effAddT = ArithInOut.ringEffortAdd sampleT $ effRingT
    effAddTDF = ArithInOut.mxfldEffortAdd sampleT sampleDF $ effFieldTDF
    
    effFieldTDF = minmaxFromRO_eff_mixedFldTDF eff
    effRingT = minmaxFromRO_eff_ringOpsT eff
    
    sampleT = a
    sampleDF = errUp

maxDnEffFromRingOps :: 
    (
     Show t, Show f,
     HasZero t, 
     ArithInOut.RoundedRing t,
     ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     HasEvalOps f t, 
     RefOrd.IntervalLike t,
     RefOrd.IntervalLike f,
     HasVarValue (VarBox f t) (Var f) t,
     CanEvaluate f,
     HasVarValue (VarBox f (Domain f)) (Var f) (Domain f),
     HasProjections f, HasConstFns f, HasOne f, -- HasZero f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f)) 
    =>
    f ->
    (f -> f) {- ^ Given a sample function, return the function \x:[0,1] -> x of the same type. -} ->
    MinmaxEffortIndicatorFromRingOps f t -> 
    Int {- ^ degree of Bernstein approximations (must be > 1) -} ->
    t -> t -> t
maxDnEffFromRingOps sampleF getX eff degree a b =
    a #<+># (fst $ fst $ maxZeroDnUp sampleF getX eff degree $ b #<-># a)
    where
    (#<+>#) = ArithInOut.addOutEff effAddT
    (#<->#) = ArithInOut.subtrOutEff effAddT
    effAddT = ArithInOut.ringEffortAdd sampleT effRingT
    effRingT = minmaxFromRO_eff_ringOpsT eff
    sampleT = a

minUpEffFromRingOps ::
    (
     Show t, Show f,
     HasZero t, 
     ArithInOut.RoundedRing t,
     ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     HasEvalOps f t, 
     RefOrd.IntervalLike t,
     RefOrd.IntervalLike f,
     HasVarValue (VarBox f t) (Var f) t,
     CanEvaluate f,
     HasVarValue (VarBox f (Domain f)) (Var f) (Domain f),
     HasProjections f, HasConstFns f, HasOne f, -- HasZero f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f)) 
    =>
    f ->
    (f -> f) {- ^ Given a sample function, return the function \x:[0,1] -> x of the same type. -} ->
    MinmaxEffortIndicatorFromRingOps f t -> 
    Int {- ^ degree of Bernstein approximations (must be > 1) -} ->
    t -> t -> t
minUpEffFromRingOps sampleF getX eff degree a b =
    neg $ maxDnEffFromRingOps sampleF getX eff degree (neg a) (neg b)

minDnEffFromRingOps ::
    (
     Show t, Show f,
     HasZero t, 
     ArithInOut.RoundedRing t,
     ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     HasEvalOps f t, 
     RefOrd.IntervalLike t,
     RefOrd.IntervalLike f,
     HasVarValue (VarBox f t) (Var f) t,
     CanEvaluate f,
     HasVarValue (VarBox f (Domain f)) (Var f) (Domain f),
     HasProjections f, HasConstFns f, HasOne f, -- HasZero f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f)) 
    =>
    f ->
    (f -> f) {- ^ Given a sample function, return the function \x:[0,1] -> x of the same type. -} ->
    MinmaxEffortIndicatorFromRingOps f t -> 
    Int {- ^ degree of Bernstein approximations (must be > 1) -} ->
    t -> t -> (t, t)
minDnEffFromRingOps sampleF getX eff degree a b =
    (neg resNeg, neg resShiftedNeg)
    where
    (resNeg, resShiftedNeg) =
        maxUpEffFromRingOps sampleF getX eff degree (neg a) (neg b) 

maxZeroDnUp ::    
    (
     Show t, Show f,
     HasZero t, 
     ArithInOut.RoundedRing t,
     ArithUpDn.Convertible t (Domain f), 
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f),
     ArithInOut.RoundedMixedField t (Domain f),
     HasEvalOps f t, 
     RefOrd.IntervalLike f,
     RefOrd.IntervalLike t,
     HasVarValue (VarBox f t) (Var f) t,
     CanEvaluate f,
     HasVarValue (VarBox f (Domain f)) (Var f) (Domain f),
     HasProjections f, HasConstFns f, HasOne f, -- HasZero f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f)) 
    =>
    f ->
    (f -> f) {- ^ Given a sample function, return the function \x:[0,1] -> x of the same type. -} ->
    MinmaxEffortIndicatorFromRingOps f t ->
    Int {- ^ degree of Bernstein approximations (must be > 1) -} ->
    t -> 
    ((t,t), Domain f)
{-
    overview of the algorithm:
    
    * find bounds of type (Domain f) for a (a is of type t)
    * if a can be shown to be positive or negative, finish
    * affinely transform a to a' so that it fits inside [0,1] 
      and note the point 0 < c < 1 where the original crossed zero
    * compute a Bernstein approximation of the given degree to the function \y -> \max(0,y-c)
    * compute a reliable estimate e of the approximation error
    * r' = evaluate this polynomial with a' for y and add [-e,0] 
    * transform r' back to the range of a to get the result r 
-}
maxZeroDnUp
        sampleF getX
        eff
        degree
        a =
    let (<=?) = NumOrd.pLeqEff effCompDF in
    case (bounded, maybeaDn, c0 <=? aDn, maybeaUp, aUp <=? c0) of
        (_,Nothing, _,_,_) -> 
            throw $ AERNException $ "maxZeroDnUp called for an unbounded value: " ++ show a
        (_,_,_,Nothing,_) -> 
            throw $ AERNException $ "maxZeroDnUp called for an unbounded value: " ++ show a
        (False,_,_,_,_) -> 
            throw $ AERNException $ "maxZeroDnUp called for an unbounded value: " ++ show a
        (_,_, Just True, _, _) ->
--            unsafePrint ("maxZeroDnUp: positive") $ 
            ((a, a), zero sampleDF)
        (_,_,_,_, Just True) -> 
--            unsafePrint ("maxZeroDnUp: negative") $ 
            ((zero a, zero a), zero sampleDF)
        _ -> 
--            unsafePrint ("maxZeroDnUp: mixed"
--                ++ "\n a = " ++ show a
--                ++ "\n aUp = " ++ show aUp
--                ++ "\n aDn = " ++ show aDn
--                ++ "\n translateToUnit a = " ++ show (translateToUnit a)
--                ++ "\n maxCUp = " ++ show maxCUp
--                ++ "\n maxCDn = " ++ show maxCDn
--                ++ "\n viaBernsteinUp = " ++ show viaBernsteinUp
--                ++ "\n viaBernsteinDn = " ++ show viaBernsteinDn
--            ) $ 
            ((viaBernsteinDn, viaBernsteinUp), errUp)
    where
    sampleT = a
    x = getX sampleF
    sampleDF = getSampleDomValue x
    maybeaUp = ArithUpDn.convertUpEff effTToDom sampleDF a
    maybeaDn = ArithUpDn.convertDnEff effTToDom sampleDF a
    Just aUp = maybeaUp
    Just aDn = maybeaDn
    bounded = excludesInfinity aWidth
    
    viaBernsteinUp = doSubst maxCUp
    viaBernsteinDn = doSubst maxCDn
    doSubst p =   
        translateFromUnit $ 
            evalOtherType (evalOpsEff effEvalOpsT sampleF sampleT) varA p
        where
        varA = fromAscList [(var, translateToUnit a)]
    (var:_) = getVars $ getDomainBox $ x
    errUp = 
        errUpUnit .<*>. aWidth
    maxCUp = 
        snd $ RefOrd.getEndpointsOut maxCUpPre
    (maxCUpPre, errUpUnit) = 
        hillbaseApproxUp effCompDF effRingF effMultFDF effRealDF effEval x c degree
    maxCDn = 
        fst $ RefOrd.getEndpointsOut maxCDnPre
    maxCDnPre =
        hillbaseApproxDn effGetEDF effCompDF effRingF effMultFDF effRealDF effEval x c dInit degree
        where
        dInit = 
            (maxCUpAtC .<->. c)
                  .<*>| (2 :: Int)
--                  .</>| (2 :: Int)
        maxCUpAtC =
            evalAtPointOutEff effEval varC maxCUp
        varC = fromAscList [(var, c)]
    c = 
        (neg aDn) .</>. aWidth
    aWidth = 
        aUp .<->. aDn
    translateToUnit b =
        (b #<+>. (neg aDn)) #</>. aWidth
    translateFromUnit b =
        (b #<*>. aWidth) #<+>. aDn
    c0 = zero sampleDF
    _ = [c,c0]
    
--    (.<+>.) = ArithInOut.addOutEff effAddDF
    (.<->.) = ArithInOut.subtrOutEff effAddDF
    (.<*>.) = ArithInOut.multOutEff effMultDF
    (.</>.) = ArithInOut.divOutEff effDivDF

    (.<*>|) = ArithInOut.mixedMultOutEff effMultDFI
--    (.</>|) = ArithInOut.mixedDivOutEff effDivDFI

    (#<+>.) = ArithInOut.mixedAddOutEff effAddTDF
    (#<*>.) = ArithInOut.mixedMultOutEff effMultTDF
    (#</>.) = ArithInOut.mixedDivOutEff effDivTDF
    
    effCompDF = ArithInOut.rrEffortNumComp c0 effRealDF
    effAddDF = ArithInOut.fldEffortAdd c0 $ ArithInOut.rrEffortField c0 effRealDF
    effMultDF = ArithInOut.fldEffortMult c0 $ ArithInOut.rrEffortField c0 effRealDF
    effDivDF = ArithInOut.fldEffortDiv c0 $ ArithInOut.rrEffortField c0 effRealDF
    
    effAddTDF = ArithInOut.mxfldEffortAdd a c0 effFldTDF
    effMultTDF = ArithInOut.mxfldEffortMult a c0 effFldTDF
    effDivTDF = ArithInOut.mxfldEffortDiv a c0 effFldTDF
    
    effMultDFI = ArithInOut.mxfldEffortMult sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF
--    effDivDFI = ArithInOut.mxfldEffortDiv sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF

    (MinmaxEffortIndicatorFromRingOps 
        effTToDom effRealDF effGetEDF 
        effEvalOpsT effEval 
        effRingF effMultFDF _sizeLimits 
        _effRingT effFldTDF) = eff


{-| compute an upper Bernstein approximation of the function max(x,c) over [0,1] -}
hillbaseApproxUp :: 
    (HasConstFns f, HasProjections f, HasOne f, ArithInOut.RoundedRing f, 
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     CanEvaluate f,
     Show (Domain f), Show f)
    =>
    NumOrd.PartialCompareEffortIndicator (Domain f) -> 
    ArithInOut.RingOpsEffortIndicator f -> 
    ArithInOut.MixedMultEffortIndicator f (Domain f) -> 
    ArithInOut.RoundedRealEffortIndicator (Domain f) -> 
    EvaluationEffortIndicator f -> 
    f {-^ the variable @x@ to use in the result uni-variate polynomial -} ->
    Domain f {-^ @c@ the only non-smooth point of the approximated piece-wise linear function -} ->
    Int {-^ @n@ Bernstein approximation degree -} ->
    (f, Domain f)
hillbaseApproxUp effComp effRingF effMultFDF effRealDF effEval x c n =
--    unsafePrintReturn ( "hillbaseApproxUp:"
--        ++ "\n x = " ++ show x
--        ++ "\n c = " ++ show c
--        ++ "\n n = " ++ show n
--        ++ "\n result = "
--    ) $
    (result, errUp)
    where
    result =
        foldl1 (~<+>~) $ map mkBT [0..n]
    errUp = 
        valueAtC .<->. c
        where
        valueAtC =
            evalAtPointOutEff effEval varC result
        varC = fromAscList [(var, c)]
    mkBT p =
--        unsafePrint ( "mkBT:"
--            ++ "\n p = " ++ show p
--            ++ "\n pOverN = " ++ show pOverN
--            ++ "\n fOfpOverN = " ++ show fOfpOverN
--            ++ "\n bernstein = " ++ show bernstein
--        ) $
        (newConstFnFromSample x fOfpOverN) ~<*>~ bernstein
        where
        bernstein = bernsteinOut (effRingF, effRealDF, effMultFDF) x n p 
        fOfpOverN =
            fst $ RefOrd.getEndpointsOut fOfpOverNPre
        fOfpOverNPre -- = maxOutEff effMinmax c0 $ pOverN <-> c
            | (pOverN <? c) == Just True = c
            | otherwise = pOverN
            where
            (<?) = NumOrd.pLessEff effComp
        pOverN = (c1 .<*>| p) .</>| n
    c1 = one sampleDF
    sampleDF = getSampleDomValue x
    sampleF = x
    (var:_) = getVars $ getDomainBox $ x

    (~<+>~) = ArithInOut.addOutEff effAddF
    (~<*>~) = ArithInOut.multOutEff effMultF
    
--    (.<+>.) = ArithInOut.addOutEff effAddDF
    (.<->.) = ArithInOut.subtrOutEff effAddDF
--    (.<*>.) = ArithInOut.multOutEff effMultDF
--    (.</>.) = ArithInOut.divOutEff effDivDF

    (.<*>|) = ArithInOut.mixedMultOutEff effMultDFI
    (.</>|) = ArithInOut.mixedDivOutEff effDivDFI

--    (#<+>.) = ArithInOut.mixedAddOutEff effAddTDF
--    (#<*>.) = ArithInOut.mixedMultOutEff effMultTDF
--    (#</>.) = ArithInOut.mixedDivOutEff effDivTDF
    
    effAddF = ArithInOut.ringEffortAdd x effRingF
    effMultF = ArithInOut.ringEffortMult x effRingF
    
    effMultDFI = ArithInOut.mxfldEffortMult sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF
    effDivDFI = ArithInOut.mxfldEffortDiv sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF

--    effMultDF = ArithInOut.fldEffortMult sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF
    effAddDF = ArithInOut.fldEffortAdd sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF

{-| compute a lower Bernstein approximation of the function max(c,x) over [0,1] 
    This needs to be done by choosing the control points somewhat below the exact
    answer.  To work out how much below we search for a close-to-optimal parameter
    for the hillbaseApproxDnD function.  We expect the @dInit@ value to be larger
    than necessary.  First we recursively decrease this d until it below the safe limit.
    Then we recursively increase it to make a little above the safe limit value. 
-}
hillbaseApproxDn :: 
    (HasConstFns f, HasProjections f, HasOne f, ArithInOut.RoundedRing f, 
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     CanEvaluate f,
     Show (Domain f), Show f)
    =>
    RefOrd.GetEndpointsEffortIndicator (Domain f) -> 
    NumOrd.PartialCompareEffortIndicator (Domain f) -> 
    ArithInOut.RingOpsEffortIndicator f -> 
    ArithInOut.MixedMultEffortIndicator f (Domain f) -> 
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvaluationEffortIndicator f -> 
    f {-^ the variable @x@ to use in the result uni-variate polynomial -} ->
    Domain f {-^ @c@ the only non-smooth point of the approximated piece-wise linear function -} ->
    Domain f {-^ @dInit@ initial value for the offset @d@ by which to translate the approximated fn down at point c -} ->
    Int {-^ @n@ Bernstein approximation degree -} ->
    f
hillbaseApproxDn effGetE effComp effRingF effMultFDF effRealDF effEval x c dInit n =
--    unsafePrintReturn ( "hillbaseApproxDn:"
--        ++ "\n x = " ++ show x
--        ++ "\n c = " ++ show c
--        ++ "\n dInit = " ++ show dInit
--        ++ "\n n = " ++ show n
--        ++ "\n result = "
--    ) $
    findDWithBelowCAtC $ findDWithAboveCAtC dInit
    where
    findDWithAboveCAtC d =
        case (fnAtCRE <=? c) of
            Just False -> d
            _ -> findDWithAboveCAtC newD
        where
        (_, fnAtCRE) =
            RefOrd.getEndpointsOutEff effGetE fnAtC
        fnAtC = evalAtPointOutEff effEval varC fn
            where
            varC = fromAscList [(var, c)]
        fn = hillDnD d
        (newD, _) = -- round downwards because we are trying to decrease d below safe limit
             RefOrd.getEndpointsOutEff effGetE $
                d .<+>. (fnAtCMinusC .<*>| (1.25 :: Double)) -- get nearer the optimal value of d
            where
            fnAtCMinusC = fnAtC .<->. c
    findDWithBelowCAtC d =
        case (fnAtCLE <=? c) of
            Just True -> fn
            _ -> findDWithBelowCAtC newD
        where
        (fnAtCLE,_) =
            RefOrd.getEndpointsOutEff effGetE fnAtC
        fnAtC = evalAtPointOutEff effEval varC fn
            where
            varC = fromAscList [(var, c)]
        fn = hillDnD d
        (_, newD) = -- round upwards because we are trying to increase d above safe limit
             RefOrd.getEndpointsOutEff effGetE $
                d .<+>. (fnAtCMinusC .<*>| (1.25 :: Double)) -- get nearer the optimal value of d
            where
            fnAtCMinusC = fnAtC .<->. c
    hillDnD = 
        hillbaseApproxDnD effComp effRingF effMultFDF effRealDF x c n
    
    (var:_) = getVars $ getDomainBox $ x
    
    (<=?) = NumOrd.pLeqEff effComp
    
--    (~<+>~) = ArithInOut.addOutEff effAddF
--    (~<*>~) = ArithInOut.multOutEff effMultF
    
    (.<+>.) = ArithInOut.addOutEff effAddDF
    (.<->.) = ArithInOut.subtrOutEff effAddDF
--    (.<*>.) = ArithInOut.multOutEff effMultDF
--    (.</>.) = ArithInOut.divOutEff effDivDF

    (.<*>|) = ArithInOut.mixedMultOutEff effMultDFDbl
--    (.</>|) = ArithInOut.mixedDivOutEff effDivDFI

--    (#<+>.) = ArithInOut.mixedAddOutEff effAddTDF
--    (#<*>.) = ArithInOut.mixedMultOutEff effMultTDF
--    (#</>.) = ArithInOut.mixedDivOutEff effDivTDF

    effAddDF = ArithInOut.fldEffortAdd sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF
    effMultDFDbl = ArithInOut.mxfldEffortMult sampleDF (1::Double) $ ArithInOut.rrEffortDoubleMixedField sampleDF effRealDF
    sampleF = x
    sampleDF = getSampleDomValue x

{-| 
  Compute an upper Bernstein approximation of the function @max(c-l-x(d-l)/c,x-r-(1-x)(d-r)/(1-c))@ over @[0,1]@,
  which is a valid lower approximation of @max(c,x)@ when @l,d,r@ are large enough.
  @l,d,r@ are the downwards offsets of the control points at @0,c,1@, respectively. 
-}
hillbaseApproxDnD :: 
    (HasConstFns f, HasProjections f, HasOne f, ArithInOut.RoundedRing f, 
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f), Show f)
    =>
    NumOrd.PartialCompareEffortIndicator (Domain f) -> 
    ArithInOut.RingOpsEffortIndicator f -> 
    ArithInOut.MixedMultEffortIndicator f (Domain f) -> 
    ArithInOut.RoundedRealEffortIndicator (Domain f) -> 
    f {-^ the variable @x@ to use in the result uni-variate polynomial -} ->
    Domain f {-^ @c@ the only non-smooth point of the approximated piece-wise linear function -} ->
    Int {-^ @n@ Bernstein approximation degree -} ->
    Domain f {-^ @d@ the distance of the approximated piece-wise linear function from 0 at point c -} ->
    f
hillbaseApproxDnD effComp effRingF effMultFDF effRealDF x c n d =
--    unsafePrint ( "hillbaseApproxDnD:"
--        ++ " n = " ++ show n
--        ++ " c = " ++ show c
--        ++ " d = " ++ show d
--    ) $
    foldl1 (~<+>~) $
        map mkBT [0..n]
    where
    l = 
        d .<*>. ((c1 .<->. c) .<^> (4 :: Int))
    r = 
        d .<*>. (c .<^> (4 :: Int))
    mkBT p =
--        unsafePrintReturn ( "mkBT:"
--            ++ "\n p = " ++ show p
--            ++ "\n pOverN = " ++ show pOverN
--            ++ "\n fOfpOverN = " ++ show fOfpOverN
--            ++ "\n bernstein = " ++ show bernstein
--            ++ "\n result = "
--        ) $
        (newConstFnFromSample x fOfpOverN) ~<*>~ bernstein 
        where
        bernstein = bernsteinOut (effRingF, effRealDF, effMultFDF) x n p
        fOfpOverN =
            snd $ RefOrd.getEndpointsOut fOfpOverNPre
        fOfpOverNPre
            | (pOverN <? c) == Just True =
                cMinusL .<->. (pOverN .<*>. dMinusLOverC)
                -- (c-l) - (p/n(d-l)/c) 
            | otherwise =
                pOverN .<*>. onePlusDMinusROverOneMinusC .<->. rPlusDMinusROverOneMinusC
                -- (p/n)(1 + (d-r)/(1-c)) - (r + (d-r)/(1-c))
        pOverN = (c1 .<*>| p) .</>| n
    cMinusL =
        c .<->. l
    dMinusLOverC = 
        (d .<->. l) .</>. c
    onePlusDMinusROverOneMinusC = 
        c1 .<+>. dMinusROverOneMinusC
    rPlusDMinusROverOneMinusC = 
        r .<+>. dMinusROverOneMinusC
    dMinusROverOneMinusC = 
        (d .<->. r) .</>. (c1 .<->. c)

    (<?) = NumOrd.pLessEff effComp
    
    (~<+>~) = ArithInOut.addOutEff effAddF
    (~<*>~) = ArithInOut.multOutEff effMultF
    
    (.<+>.) = ArithInOut.addOutEff effAddDF
    (.<->.) = ArithInOut.subtrOutEff effAddDF
    (.<*>.) = ArithInOut.multOutEff effMultDF
    (.<^>) = ArithInOut.powerToNonnegIntOutEff effPowDF
    (.</>.) = ArithInOut.divOutEff effDivDF

    (.<*>|) = ArithInOut.mixedMultOutEff effMultDFI
    (.</>|) = ArithInOut.mixedDivOutEff effDivDFI

--    (#<+>.) = ArithInOut.mixedAddOutEff effAddTDF
--    (#<*>.) = ArithInOut.mixedMultOutEff effMultTDF
--    (#</>.) = ArithInOut.mixedDivOutEff effDivTDF

    effAddF = ArithInOut.ringEffortAdd x effRingF
    effMultF = ArithInOut.ringEffortMult x effRingF
    
    effMultDFI = ArithInOut.mxfldEffortMult sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF
    effDivDFI = ArithInOut.mxfldEffortDiv sampleDF (1::Int) $ ArithInOut.rrEffortIntMixedField sampleDF effRealDF

    effAddDF = ArithInOut.fldEffortAdd sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF
    effMultDF = ArithInOut.fldEffortMult sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF
    effPowDF = ArithInOut.fldEffortPow sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF
    effDivDF = ArithInOut.fldEffortDiv sampleDF $ ArithInOut.rrEffortField sampleDF effRealDF

    c1 = one sampleDF
    sampleDF = getSampleDomValue x
    

    
    

    