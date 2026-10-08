{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ImplicitParams #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Evaluation
    Description :  evaluation of interval polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Evaluation of interval polynomials in general using its instance of 'CanEvaluateOtherType' 
    and specilised cases for evaluation at a point and on an interval.
-}

module Numeric.AERN.Poly.IntPoly.Evaluation
    (
        PolyEvalOps(..),
        PolyEvalMonoOps(..)
--        ,
--        evalPolyAtPointOut,
--        evalPolyAtPointIn,
--        evalPolyOnIntervalOut,
--        evalPolyOnIntervalIn
    )
where

import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New ()
import Numeric.AERN.Poly.IntPoly.Show ()
import Numeric.AERN.Poly.IntPoly.Differentiation

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn 
--import Numeric.AERN.RealArithmetic.NumericOrderRounding (ConvertEffortIndicator) -- needed for ghc 6.12

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.RefinementOrder as RefOrd

import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Effort


import qualified Data.IntMap as IntMap
--import qualified Data.Map as Map
import Data.List (sortBy)

import Numeric.AERN.Misc.Debug
_ = unsafePrint

instance 
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     HasConsistency cf)
    =>
    CanEvaluateOtherType (IntPoly var cf)
    where
    type EvalOps (IntPoly var cf) =  PolyEvalOps var cf
    evalOtherType ops valsMap p@(IntPoly cfg _) =
        case polyEvalMonoOps ops of
            Nothing -> evalDirect valsLZ p
            _ -> evalPolyMono evalDirect ops valsLZ p 
        where
        evalDirect = evalPolyDirect ops
        valsLZ = valsMapToValuesLZ subtrCf cfg valsMap
        subtrCf val domLE =
             addV val (cfV $ neg domLE)
        addV = polyEvalAdd ops
        cfV = polyEvalCoeff ops

instance 
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     HasAntiConsistency cf)
    =>
    CanEvaluateOtherTypeInner (IntPoly var cf)
    where
    evalOtherTypeInner ops valsMap p@(IntPoly cfg _) =
        case polyEvalMonoOps ops of
            Nothing -> evalDirect valsLZ p
            _ -> evalPolyMono evalDirect ops valsLZ p 
        where
        evalDirect vals p2 = 
            flipConsistency $
                evalPolyDirect ops vals $ 
                    flipConsistencyPoly p2
        valsLZ = valsMapToValuesLZ subtrCf cfg valsMap
        subtrCf val domLE =
             addV val (cfV $ neg domLE)
        addV = polyEvalAdd ops
        cfV = polyEvalCoeff ops
    

valsMapToValuesLZ :: 
    (Ord var, Show var, Show t) 
     =>
     (t -> cf -> t) ->
     IntPolyCfg var cf -> 
     (VarBox (IntPoly var cf) t) -> 
     [t]
valsMapToValuesLZ subtractCf cfg valsMap =
    zipWith subtractCf vals domsLE
    where
    vals = map getValue vars
    vars = ipolycfg_vars cfg
    domsLE = ipolycfg_domsLE cfg
    getValue var = 
        case lookupVar valsMap var of 
            Just val -> val
            _ -> error $ 
                "aern-poly internal error in Evaluation...valsMapToValuesLZ:"
                ++ "\n var " ++ show var ++ " not present in valsMap"
                ++ "\n valsMap = " ++ show valsMap 
     
instance
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf, 
     RefOrd.IntervalLike cf)
    =>
    CanEvaluate (IntPoly var cf)
    where
    type (EvaluationEffortIndicator (IntPoly var cf)) = 
        IntPolyEffort cf
    evaluationDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg
    evalAtPointOutEff eff valsMap p@(IntPoly cfg _) 
        | valuesAreExact valsLZ =
            evalPolyAtPointOut effCf maxSplitSize valsLZ p
        | otherwise =
            evalPolyOnIntervalOut effCf maxSplitSize valsLZ p
        where
        valsLZ = valsMapToValuesLZ (<->) cfg valsMap
        (<->) = ArithInOut.subtrOutEff effAdd
        effAdd = ArithInOut.fldEffortAdd sampleCf $ ArithInOut.rrEffortField sampleCf effCf
        
        maxSplitSize = ipolyeff_evalMaxSplitSize eff
        effCf = ipolyeff_cfRoundedRealEffort eff
        sampleCf = ipolycfg_sample_cf cfg
        
    evalAtPointInEff eff valsMap p@(IntPoly cfg _) 
        | valuesAreExact valsLZ =
            evalPolyAtPointIn effCf maxSplitSize valsLZ p
        | otherwise =
            evalPolyOnIntervalIn effCf maxSplitSize valsLZ p
        where
        valsLZ = valsMapToValuesLZ (>-<) cfg valsMap
        (>-<) = ArithInOut.subtrInEff effAdd
        effAdd = ArithInOut.fldEffortAdd sampleCf $ ArithInOut.rrEffortField sampleCf effCf

        maxSplitSize = ipolyeff_evalMaxSplitSize eff
        effCf = ipolyeff_cfRoundedRealEffort eff
        sampleCf = ipolycfg_sample_cf cfg
    
valuesAreExact :: 
    HasConsistency t 
    => 
    [t] -> Bool
valuesAreExact values =
    and $ map isCertainlyExact values
    where
    isCertainlyExact val = 
        isExact val == Just True 
    
--instance
--    (Ord var, Show var,
--     ArithInOut.RoundedReal cf, RefOrd.IntervalLike cf,
--     HasConsistency cf, 
--     Show cf, Show (SizeLimits cf))
--    =>
--    HasEvalOps (IntPoly var cf) cf
--    where
--    type EvalOpsEffortIndicator (IntPoly var cf) cf =
--        IntPolyEffort cf
--    evalOpsDefaultEffort _p@(IntPoly cfg _) _sampleCf = 
--        ipolycfg_effort cfg
----        (ArithInOut.roundedRealDefaultEffort sampleCf, Int1To10 depth)
----        where
----        depth = 1 + (maxDeg `div` 2)
----        maxDeg = ipolycfg_maxdeg cfg
--         
--    evalOpsEff eff _sampleP sampleCf =
--        coeffPolyEvalOpsOut effCf maxSplitSize sampleCf
--        where
--        maxSplitSize = ipolyeff_evalMaxSplitSize eff
--        effCf = ipolyeff_cfRoundedRealEffort eff
        

data PolyEvalOps var cf val =
    PolyEvalOps
    {
        polyEvalZero :: val,
        polyEvalAdd :: (val -> val -> val),
        polyEvalMult :: (val -> val -> val),
        polyEvalPow :: (val -> Int -> val), {-^ non-negative integer power -}
        polyEvalCoeff :: (cf -> val), {-^ coeff conversion -}
        polyEvalMaybePoly :: (IntPolyTerms var cf -> Maybe val), {-^ optional direct poly conversion -}
        polyEvalSplitMaxSize :: Int,
        polyEvalMonoOps :: Maybe (PolyEvalMonoOps var cf val)
    }

data PolyEvalMonoOps var cf val =
    PolyEvalMonoOps
    {
        polyEvalMonoOuter :: PolyEvalOps var cf val,
        polyEvalMonoLeq :: (val -> val -> Maybe Bool),
        polyEvalMonoGetEndpoints :: val -> (val, val),
        polyEvalMonoFromEndpoints :: (val, val) -> val,
        polyEvalMonoIsExact :: val -> Bool,
        polyEvalMonoSplit :: val -> (val, val),
        polyEvalMonoMerge :: (val, val) -> val,
        polyEvalMonoMin :: val -> val -> val,
        polyEvalMonoMax :: val -> val -> val,
        polyEvalMonoGetWidthAsDouble :: val -> Double,
        polyEvalMonoCfEffortIndicator :: ArithInOut.RoundedRealEffortIndicator cf
    }


coeffPolyEvalOpsOut ::
    (RefOrd.IntervalLike cf, ArithInOut.RoundedReal cf, HasConsistency cf)
    =>
   (ArithInOut.RoundedRealEffortIndicator cf) ->
   Int ->
   cf ->
   PolyEvalOps var cf cf
coeffPolyEvalOpsOut eff depth sample =
    result
    where
    result =
        PolyEvalOps (zero sample) (<+>) (<*>) (<^>) id (const Nothing) depth $
            Just $ PolyEvalMonoOps
                result -- outer rounded ops = itself
                (<=?)
                RefOrd.getEndpointsOut
                RefOrd.fromEndpointsOut
                isDefinitelyExact
                split
                (uncurry (</\>))
                (NumOrd.minOutEff effMinmax)
                (NumOrd.maxOutEff effMinmax)
                getWidthAsDouble
                eff
    isDefinitelyExact a =
        isExact a == Just True
    split val = (val1, val2)
        where
        val1 = RefOrd.fromEndpointsOut (valL, valM)
        val2 = RefOrd.fromEndpointsOut (valM, valR)
        (valL, valR) = RefOrd.getEndpointsOut val
        valM = (valL <+> valR) </>| (2 :: Int)
    getWidthAsDouble val = wD
        where
        Just wD = ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort val (0::Double)) 0 w
        w = valR <-> valL
        (valL, valR) = RefOrd.getEndpointsOut val
        
    (<=?) = NumOrd.pLeqEff effComp
    (</\>) = RefOrd.meetOutEff effJoin
    
    (<+>) = ArithInOut.addOutEff effAdd
    (<->) = ArithInOut.subtrOutEff effAdd
    (<*>) = ArithInOut.multOutEff effMult
    (<^>) = ArithInOut.powerToNonnegIntOutEff effPwr
    (</>|) = ArithInOut.mixedDivOutEff effDivInt

    effMult = ArithInOut.fldEffortMult sample $ ArithInOut.rrEffortField sample eff
    effPwr = ArithInOut.fldEffortPow sample $ ArithInOut.rrEffortField sample eff
    effAdd = ArithInOut.fldEffortAdd sample $ ArithInOut.rrEffortField sample eff
    effDivInt = ArithInOut.mxfldEffortDiv sample (1::Int) $ ArithInOut.rrEffortIntMixedField sample eff
    effComp = ArithInOut.rrEffortNumComp sample eff
    effJoin = ArithInOut.rrEffortJoinMeet sample eff
    effMinmax = ArithInOut.rrEffortMinmaxInOut sample eff


instance 
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf, 
     RefOrd.IntervalLike cf)
    => 
    (HasDistance (IntPoly var cf))
    where
    type Distance (IntPoly var cf) = cf
    type DistanceEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    distanceDefaultEffort p =
        evaluationDefaultEffort p
    distanceBetweenEff eff p1 p2 =
        ArithInOut.absOutEff effAbs $ evalAtPointOutEff eff dombox diff
        where
        dombox = getDomainBox diff
        diff = polyJoinWith (zero sampleCf) (uncurry $ ArithInOut.subtrOutEff effAdd) (p1, p2) 
        sampleCf = getSampleDomValue p1 
        effAdd = 
            ArithInOut.fldEffortAdd sampleCf $ 
                ArithInOut.rrEffortField sampleCf effCf
        effAbs = 
            ArithInOut.rrEffortAbs sampleCf effCf
        effCf = ipolyeff_cfRoundedRealEffort eff
        
instance 
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf, 
     RefOrd.IntervalLike cf)
    => 
    (HasImprecision (IntPoly var cf))
    where
    type Imprecision (IntPoly var cf) = cf
    type ImprecisionEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    imprecisionDefaultEffort p =
        evaluationDefaultEffort p
    imprecisionOfEff eff p = distanceBetweenEff eff p p

--instance
--    (Ord var, Show var,
--     ArithInOut.RoundedReal cf, RefOrd.IntervalLike cf,
--     HasAntiConsistency cf, 
--     Show cf)
--    =>
--    ArithUpDn.Convertible (IntPoly var cf) cf
--    where
--    type ConvertEffortIndicator (IntPoly var cf) cf = 
--        (EvaluationEffortIndicator (IntPoly var cf), 
--         RefOrd.GetEndpointsEffortIndicator cf)
--    convertDefaultEffort sampleP sampleCf = 
--        (evaluationDefaultEffort sampleP, 
--         RefOrd.getEndpointsDefaultEffort sampleCf)
--    convertUpEff (effEval, effGetEndpts) sampleCf p =
--        Just $ snd $ RefOrd.getEndpointsOutEff effGetEndpts range
--        where
--        range = evalOtherType (evalOpsEff effEval sampleP sampleCf) varDoms p 
--        varDoms = getDomainBox p
--        sampleP = p
--    convertDnEff (effEval, effGetEndpts) sampleCf p =
--        Just $ fst $ RefOrd.getEndpointsOutEff effGetEndpts range
--        where
--        range = evalOtherType (evalOpsEff effEval sampleP sampleCf) varDoms p 
--        varDoms = getDomainBox p
--        sampleP = p
    

evalPolyAtPointOut, evalPolyAtPointIn ::
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf, RefOrd.IntervalLike cf,
     HasAntiConsistency cf, 
     Show cf) 
    => 
    (ArithInOut.RoundedRealEffortIndicator cf) ->
    Int ->
    [cf] {- values for each variable respectively -} -> 
    IntPoly var cf -> cf
evalPolyAtPointOut eff depth values p@(IntPoly cfg _)
    = 
    evalPolyDirect (coeffPolyEvalOpsOut eff depth sample) values p
    where
    sample = ipolycfg_sample_cf cfg
evalPolyAtPointIn eff depth values p@(IntPoly cfg _)
    = 
    flipConsistency $
        evalPolyDirect (coeffPolyEvalOpsOut eff depth sample) values $ 
            flipConsistencyPoly p
    where
    sample = ipolycfg_sample_cf cfg

evalPolyOnIntervalOut, evalPolyOnIntervalIn ::
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf, RefOrd.IntervalLike cf, 
     HasAntiConsistency cf, 
     Show cf, Show (SizeLimits cf)) 
    => 
    ArithInOut.RoundedRealEffortIndicator cf 
    ->
    Int
    ->
    [cf] {- values for each variable respectively -} 
    -> 
    IntPoly var cf -> cf
evalPolyOnIntervalOut eff maxSplitDepth values p@(IntPoly cfg _)
    =
    evalPolyMono evalOut ops values p
    where
    evalOut = evalPolyDirect ops
    ops = coeffPolyEvalOpsOut eff maxSplitDepth sample 
    sample = ipolycfg_sample_cf cfg
evalPolyOnIntervalIn eff maxSplitDepth values p@(IntPoly cfg _)
    = 
    evalPolyMono evalIn ops values p
    where
    evalIn values2 p2 = 
        flipConsistency $ 
            evalPolyDirect ops values2 $ 
                flipConsistencyPoly p2
    ops = coeffPolyEvalOpsOut eff maxSplitDepth sample 
    sample = ipolycfg_sample_cf cfg

evalPolyDirect ::
    (Ord var, Show var, Show cf, Show val, Neg cf) => 
    (PolyEvalOps var cf val) ->
    [val] ->
    IntPoly var cf -> val
evalPolyDirect opsV valuesLZ _p@(IntPoly _cfg terms)
    = 
--    unsafePrint
--    (
--        "evalPolyDirect: "
--        ++ "\n  values = " ++ show values
--        ++ "\n  p = " ++ show p
--    ) $
    ev valuesLZ terms
    where
    zV = polyEvalZero opsV
    addV = polyEvalAdd opsV
    multV = polyEvalMult opsV
    powV = polyEvalPow opsV
    cfV = polyEvalCoeff opsV
    polyV = polyEvalMaybePoly opsV
    ev [] (IntPolyC cf) = cfV cf
    ev (varValueLZ : restValues) (IntPolyV _ powers)
        | IntMap.null powers = zV
        | lowestExponent == 0 = 
            resultMaybeWithoutConstantTerm
        | otherwise =
            (varValueLZ `powV` lowestExponent) `multV` resultMaybeWithoutConstantTerm 
        where
        (lowestExponent, resultMaybeWithoutConstantTerm) 
            = IntMap.foldrWithKey addTerm (highestExponent, zV) powers 
        (highestExponent, _) = IntMap.findMax powers
        addTerm exponent_2 poly (prevExponent, prevVal) = (exponent_2, newVal)
            where
            newVal = -- Horner scheme:
                polyValue 
                `addV`
                (prevVal `multV` (varValueLZ `powV` (prevExponent - exponent_2)))
            polyValue =
                case polyV poly of
                    Just value -> value
                    Nothing -> ev restValues poly  
    ev varVals terms_2 =
        error $ 
            "evalPolyDirect: illegal case:" 
            ++ "\n varVals = " ++ show varVals 
            ++ "\n terms = " ++ show terms_2

-- TODO: make the following function generic for any function representation with nominal derivatives    
evalPolyMono ::
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf, 
     RefOrd.IntervalLike cf,
     HasConsistency cf,
     Show cf, Show (SizeLimits cf), Show val) 
    =>
    ([val] -> IntPoly var cf -> val) -- ^ direct evaluator (typically, @evalPolyDirect opsV@) 
    ->
    (PolyEvalOps var cf val) 
    ->
    [val] {-^ value for each variable -} 
    -> 
    IntPoly var cf 
    -> 
    val
evalPolyMono evalDirect opsV valuesG pOrig@(IntPoly cfg _)
    | noMonoOps = direct
--    | noMonotoneVar = useMonotonicityAndSplit
    | otherwise =
--        (case segCount > 1 of
--            False -> id
--            True ->
--                unsafePrint
--                (
--                    "evalPolyMono:"
--                    ++ "\n split into " ++ show segCount ++ " segment(s), " 
--                    ++ show (emsCountIgnoreNodes resultEMS) ++ " pruned"
--                    ++ " (maxSplitSize = " ++ show maxSplitSize ++ ")"
--                    ++ "\n polyTermSize p = " ++ show (polyTermSize p)
--                ))
--        unsafePrint
--        (
--            "evalPolyMono: "
--            ++ "\n pOrig = " ++ show pOrig
--            ++ "\n maybeResultL = " ++ show maybeResultL
--            ++ "\n maybeResultR = " ++ show maybeResultR
--            ++ "\n direct = " ++ show direct
--        ) $ 
        case (maybeResultL, maybeResultR) of
            (Just resultL, Just resultR) -> 
                mergeV (resultL, resultR)
            _ -> error $ 
                    "evalPolyMono: maybeResultL = " ++ show maybeResultL 
                    ++ "; maybeResultR = " ++ show maybeResultR
    where
    vars = ipolycfg_vars cfg
    
    direct = evalDirect valuesG pOrig
    
    (pL, pR) = RefOrd.getEndpointsOut pOrig
    maybeResultL = maybeResultForP pL
    maybeResultR = maybeResultForP pR
    maybeResultForP p =
        emsCollectResultsSoFar (curry mergeV) $
            useMonotonicityAndSplitting (direct, direct) $ 
                EMSTODO (direct, [direct]) $ zip valuesG $ repeat (False, False)
        where
        useMonotonicityAndSplitting (minSoFar, maxSoFar) emsTree =
            case evalNextLevel 0 emsTree of
                (emsTreeNew, Nothing, _) -> emsTreeNew 
                (emsTreeNew, _, False) -> emsTreeNew 
                (emsTreeNew, _, _) -> 
                    case emsCollectMinMax minV maxV emsTreeNew of
                        Just (minSoFarNew, maxSoFarNew) ->
--                            unsafePrint
--                            (
--                                "evalPolyMono: useMonotonicityAndSplitting: processing next layer:"
--                                ++ "\n size of emsTreeNew = " ++ show (emsCountNodes emsTreeNew)
--                                ++ "\n minSoFarNew = " ++ show minSoFarNew
--                                ++ "\n maxSoFarNew = " ++ show maxSoFarNew
--                            ) $
                            useMonotonicityAndSplitting (minSoFarNew, maxSoFarNew) emsTreeNew
                        _ ->
                            error $ 
                                "evalPolyMono: useMonotonicityAndSplitting: no result in tree!"
                                ++ "\n p = " ++ show p
                                ++ "\n valuesG = " ++ show valuesG
                                ++ "\n minSoFar = " ++ show minSoFar
                                ++ "\n maxSoFar = " ++ show maxSoFar
                                ++ "\n emsTree = " ++ show emsTree
                                ++ "\n emsTreeNew = " ++ show emsTreeNew
            where
            evalNextLevel nodeCountSoFar (EMSSplit left right) =
                (EMSSplit leftNew rightNew, maybeNodeCountR, updatedL || updatedR)
                where
                (leftNew, maybeNodeCountL, updatedL) = evalNextLevel nodeCountSoFar left
                (rightNew, maybeNodeCountR, updatedR) =
                    case maybeNodeCountL of
                        Just nodeCountL -> 
                            evalNextLevel nodeCountL right
                        Nothing -> 
                            (right, Nothing, False)
            evalNextLevel nodeCountSoFar t@(EMSDone _) = (t, Just (nodeCountSoFar + 1), False)
            evalNextLevel nodeCountSoFar t@(EMSIgnore _) = (t, Just (nodeCountSoFar + 1), False)
            evalNextLevel nodeCountSoFar (EMSTODO nosplitResultSamples valuesAndDetectedMonotonicity)
                | insideOthers = 
                    (EMSIgnore nosplitResultSamples, Just (nodeCountSoFar + 1), False)
                | nodeCountSoFar + 1 >= maxSplitSize = -- node count limit reached 
                    (EMSDone nosplitResultSamples, Nothing, False)
                | splitHelps =
                    (EMSSplit (EMSTODO (leftRes, leftSamples) leftValsEtc) (EMSTODO (rightRes, rightSamples) rightValsEtc), 
                     Just $ nodeCountSoFar + 2, True)
                | otherwise = -- ie splitting further does not help:
                    (EMSDone nosplitResultSamples, Just (nodeCountSoFar + 1), False)
                where
                insideOthers = 
                    (minSoFar `leqV` nosplitResult == Just True) &&
                    (nosplitResult `leqV` maxSoFar == Just True)
                
                nosplitResultWidth = getWidthDblV nosplitResult
                nosplitResult = fst nosplitResultSamples
    
                leftSamples = getSamplesFor leftValsEtc
                rightSamples = getSamplesFor rightValsEtc
                getSamplesFor valuesAndDetectedMonotonicity2 =
--                    unsafePrint
--                    (
--                        "getSamplesFor:"
--                        ++ "\n valuesAndDetectedMonotonicity2 = " ++ show valuesAndDetectedMonotonicity2
--                        ++ "\n samplePoints2 = \n" 
--                        ++ unlines (map show samplePoints2)
--                    ) $
                    map (fst . useMonotonicity) samplePoints
                    where
                    samplePoints = take maxSplitSize samplePoints2
                    samplePoints2 
                        | someMonotonicityInfo =
                            interleaveLists
                            (getPoints True [] valuesAndDetectedMonotonicity2)
                            (getPoints False [] valuesAndDetectedMonotonicity2)
                        | otherwise =
                            getPoints True [] valuesAndDetectedMonotonicity2
                    someMonotonicityInfo =
                        or $ map (\(_,(isMono, _)) -> isMono) valuesAndDetectedMonotonicity2 
                    getPoints _shouldAimDown prevValues [] = [reverse prevValues]
                    getPoints shouldAimDown prevValues (vdm@(value, dm@(detectedMono, isIncreasing)) : rest)
                        | isExactV value =
                            getPoints shouldAimDown (vdm : prevValues) rest
                        | detectedMono =
                            getPoints shouldAimDown ((valueEndpt, (True, True)) : prevValues) rest
                        | otherwise =
                            interleaveLists
                            (getPoints shouldAimDown ((valueLE, (True, True)) : prevValues) rest)
                            (getPoints shouldAimDown ((valueRE, (True, True)) : prevValues) rest)
                        where
                        valueEndpt 
                            | isIncreasing `xor` shouldAimDown = valueLE 
                            | otherwise = valueRE
                        (valueLE, valueRE) = getEndPtsV value
    
                splitHelps 
                    | null possibleSplits = False
                    | otherwise = bestSplitWidth < nosplitResultWidth
                ((bestSplitWidth, ((leftRes, leftValsEtc), (rightRes, rightValsEtc))))
                    = bestSplitInfo
                (bestSplitInfo : _) =
                    sortBy (\ (a,_) (b,_) -> compare a b) splitsInfo
                splitsInfo =
                    map getWidth splitResults
                    where
                    getWidth result@((leftVal, _),(rightVal, _)) = (width :: Double, result)
                        where
                        width = getWidthDblV $ mergeV (leftVal, rightVal) 
                splitResults =
                    map computeSplitResult possibleSplits
                possibleSplits =
                    getSplits [] [] valuesAndDetectedMonotonicity
                    where
                    getSplits prevSplits _prevValues [] = prevSplits
                    getSplits prevSplits prevValues (vd@(value, dmAndIncr@(detectedMono, _)) : rest) 
                        | detectedMono =
                            getSplits prevSplits (vd : prevValues) rest -- do not split value for which p is monotone
                        | otherwise =
                            getSplits (newSplit : prevSplits) (vd : prevValues) rest
                        where
                        newSplit =
                            (
                                prevValuesRev ++ [(valueL, dmAndIncr)] ++ rest
                            ,
                                prevValuesRev ++ [(valueR, dmAndIncr)] ++ rest
                            )
                        prevValuesRev = reverse prevValues
                        (valueL, valueR) = splitV value
                    
                computeSplitResult (valuesAndDM_L, valuesAndDM_R) =
                    (useMonotonicity valuesAndDM_L, useMonotonicity valuesAndDM_R)
    
        useMonotonicity valuesAndPrevDetectedMonotonicity =
            (fromEndPtsV (left, right), valuesAndCurrentDetectedMonotonicity)
            where
            values = map fst valuesAndPrevDetectedMonotonicity
            left = evalDirect valuesL p
            right = evalDirect valuesR p
            (_noMonotoneVar, valuesL, valuesR, valuesAndCurrentDetectedMonotonicity) =
                let ?mixedMultInOutEffort = effMult in
                detectMono True [] [] [] $ 
                    reverse $  -- undo reverse due to the accummulators
                        zip vars $ valuesAndPrevDetectedMonotonicity
            detectMono noMonotoneVarPrev valuesLPrev valuesRPrev 
                    valuesAndCurrentDetectedMonotonicityPrev []
                = (noMonotoneVarPrev, valuesLPrev, valuesRPrev, 
                        valuesAndCurrentDetectedMonotonicityPrev)
            detectMono noMonotoneVarPrev valuesLPrev valuesRPrev 
                    valuesAndCurrentDetectedMonotonicityPrev ((var, (val, dmAndIncr@(prevDM, isIncreasing))) : rest)
                =
    --            unsafePrint 
    --            (
    --                "evalPolyMono: detectMono: deriv = " ++ show deriv
    --            ) $ 
                detectMono noMonotoneVarNew (valLNew : valuesLPrev) (valRNew : valuesRPrev) 
                     ((val, newDMAndIncr) : valuesAndCurrentDetectedMonotonicityPrev) rest
                where
                noMonotoneVarNew = noMonotoneVarPrev && varNotMono
                (valLNew, valRNew, newDMAndIncr)
                    | prevDM && isIncreasing = (valL, valR, dmAndIncr)
                    | prevDM = (valR, valL, dmAndIncr)
                    | varNotMono = (val, val, dmAndIncr) -- not monotone, we have to be safe
                    | varNonDecr = (valL, valR, (True, True)) -- non-decreasing on the whole domain - can use endpoints
                    | otherwise = (valR, valL, (True, False)) -- non-increasing on the whole domain - can use swapped endpoints
                (varNonDecr, varNotMono) =
                    case (valIsExact, zV `leqV` deriv, deriv `leqV` zV) of
                        (True, _, _) -> (undefined, True) -- when a variable has a thin domain, do not bother separating endpoints 
                        (_, Just True, _) -> (True, False) 
                        (_, _, Just True) -> (False, False)
                        _ -> (undefined, True)
                deriv =
                    evalPolyDirect opsVOut values $ -- this evaluation must be outer-rounded!
                        diffPolyOut effCf var p -- range of (d p)/(d var)    
                (valL, valR) = getEndPtsV val
                valIsExact = isExactV val

        
    zV = polyEvalZero opsV
    maxSplitSize = polyEvalSplitMaxSize opsV 
    (noMonoOps, monoOpsV) = 
        case polyEvalMonoOps opsV of
            Nothing -> (True, error "evalPolyMono: internal error: monoOpsV used when not present")
            Just monoOpsV_2 -> (False, monoOpsV_2)
    leqV = polyEvalMonoLeq monoOpsV
    fromEndPtsV = polyEvalMonoFromEndpoints monoOpsV
    getEndPtsV = polyEvalMonoGetEndpoints monoOpsV
    isExactV = polyEvalMonoIsExact monoOpsV
    splitV = polyEvalMonoSplit monoOpsV
    mergeV = polyEvalMonoMerge monoOpsV
    minV = polyEvalMonoMin monoOpsV
    maxV = polyEvalMonoMax monoOpsV
    getWidthDblV = polyEvalMonoGetWidthAsDouble monoOpsV
    
    effCf = polyEvalMonoCfEffortIndicator monoOpsV
    opsVOut = polyEvalMonoOuter monoOpsV
    
    effMult = ArithInOut.mxfldEffortMult sampleCf (1::Int) $ ArithInOut.rrEffortIntMixedField sampleCf effCf
    sampleCf = ipolycfg_sample_cf cfg
            
--    useMonotonicityAndSplitWith remainingDepth valuesAndPrevDetectedMonotonicity
--        | remainingDepth > 0 && splitHelps =
----            unsafePrint
----            (
----                "evalPolyMono: useMonotonicityAndSplitWith: SPLIT"
----                ++ "\n remainingDepth = " ++ show remainingDepth
----                ++ "\n valuesAndPrevDetectedMonotonicity = " ++ show valuesAndPrevDetectedMonotonicity
----                ++ "\n valuesAndCurrentDetectedMonotonicity = " ++ show valuesAndCurrentDetectedMonotonicity
----                ++ "\n nosplitResult = " ++ show nosplitResult
----                ++ "\n nosplitResultWidth = " ++ show nosplitResultWidth
----                ++ "\n bestSplit = " ++ show bestSplit
----                ++ "\n bestSplitResult = " ++ show _bestSplitResult
----                ++ "\n bestSplitWidth = " ++ show bestSplitWidth
----            ) $ 
--            computeSplitResultContinueSplitting bestSplit
--        | otherwise = 
----            unsafePrint
----            (
----                "evalPolyMono: useMonotonicityAndSplitWith: DONE"
----                ++ "\n remainingDepth = " ++ show remainingDepth
----                ++ "\n valuesAndPrevDetectedMonotonicity = " ++ show valuesAndPrevDetectedMonotonicity
----                ++ "\n valuesAndCurrentDetectedMonotonicity = " ++ show valuesAndCurrentDetectedMonotonicity
----                ++ "\n nosplitResult = " ++ show nosplitResult
----                ++ "\n nosplitResultWidth = " ++ show nosplitResultWidth
----            ) $ 
--            (nosplitResult, 1 :: Int)
--        where
--        nosplitResult = fromEndPtsV nosplitResultLR
--        (nosplitResultLR, valuesAndCurrentDetectedMonotonicity) = 
--            useMonotonicity valuesAndPrevDetectedMonotonicity
--        nosplitResultWidth = getWidthDblV nosplitResult
--            
--        splitHelps 
--            | null possibleSplits = False
--            | otherwise = bestSplitWidth < nosplitResultWidth
--        (((bestSplitWidth, _bestSplitResult), bestSplit) : _) =
--            sortBy (\ ((a,_),_) ((b,_),_) -> compare a b) $ zip splitWidths possibleSplits
--        splitWidths =
--            map getWidth splitResults
--            where
--            getWidth result = (width :: Double, result)
--                where
--                width = getWidthDblV result 
--        splitResults =
--            map computeSplitResult possibleSplits
--        possibleSplits =
--            getSplits [] [] valuesAndCurrentDetectedMonotonicity
--            where
--            getSplits prevSplits _prevValues [] = prevSplits
--            getSplits prevSplits prevValues (vd@(value, dmAndIncr@(detectedMono, _)) : rest) 
--                | detectedMono =
--                    getSplits prevSplits (vd : prevValues) rest -- do not split value for which p is monotone
--                | otherwise =
--                    getSplits (newSplit : prevSplits) (vd : prevValues) rest
--                where
--                newSplit =
--                    (
--                        prevValuesRev ++ [(valueL, dmAndIncr)] ++ rest
--                    ,
--                        prevValuesRev ++ [(valueR, dmAndIncr)] ++ rest
--                    )
--                prevValuesRev = reverse prevValues
--                (valueL, valueR) = splitV value
--        
--        computeSplitResult (valuesAndDM_L, valuesAndDM_R) =
--            mergeV (resultL, resultR)
--            where
--            resultL = fromEndPtsV $ fst $ useMonotonicity valuesAndDM_L
--            resultR = fromEndPtsV $ fst $ useMonotonicity valuesAndDM_R
--        
--        computeSplitResultContinueSplitting (valuesAndDM_L, valuesAndDM_R) =
--            (mergeV (resultL, resultR), splitCountL + splitCountR)
--            where
--            (resultL, splitCountL) = useMonotonicityAndSplitWith (remainingDepth - 1) valuesAndDM_L
--            (resultR, splitCountR) = useMonotonicityAndSplitWith (remainingDepth - 1) valuesAndDM_R
                    
interleaveLists :: [a] -> [a] -> [a]
interleaveLists (h1 : t1) (h2 : t2) = h1 : h2 : (interleaveLists t1 t2)
interleaveLists [] list2 = list2
interleaveLists list1 [] = list1
    
xor False b = b
xor True b = not b 
    
data EvalMonoSpliting task value =
    EMSSplit (EvalMonoSpliting task value) (EvalMonoSpliting task value)
    | EMSTODO (value, [value]) task
    | EMSDone (value, [value])
    | EMSIgnore (value, [value])
    deriving Show
    
emsCountNodes ::
    EvalMonoSpliting task value -> Int
emsCountNodes (EMSSplit left right) =
    (emsCountNodes left) + (emsCountNodes right)
emsCountNodes _ = 1

emsCountIgnoreNodes ::
    EvalMonoSpliting task value -> Int
emsCountIgnoreNodes (EMSSplit left right) =
    (emsCountIgnoreNodes left) + (emsCountIgnoreNodes right)
emsCountIgnoreNodes (EMSIgnore _) = 1
emsCountIgnoreNodes _ = 0

emsCollectResultsSoFar :: 
    (value -> value -> value) ->
    EvalMonoSpliting task value ->
    Maybe value
emsCollectResultsSoFar mergeV emsTree =
    aux emsTree
    where
    aux (EMSIgnore (val, _)) = Just val
    aux (EMSDone (val, _)) = Just val
    aux (EMSTODO (val, _) _) = Just val
    aux (EMSSplit left right) =
        do
        leftVal <- aux left
        rightVal <- aux right
        return $ leftVal `mergeV` rightVal
            
emsCollectMinMax :: 
    (value -> value -> value) ->
    (value -> value -> value) ->
    EvalMonoSpliting task value ->
    Maybe (value, value)
emsCollectMinMax minV maxV emsTree =
    aux emsTree
    where
    aux (EMSIgnore (_, samples)) = minmaxFromSamples samples
    aux (EMSDone (_, samples)) = minmaxFromSamples samples
    aux (EMSTODO (_, samples) _) = minmaxFromSamples samples
    aux (EMSSplit left right) =
        do
        (leftMin, leftMax) <- aux left
        (rightMin, rightMax) <- aux right
        return $ 
            (leftMin `minV` rightMin, leftMax `maxV` rightMax)
    minmaxFromSamples [] = Nothing
    minmaxFromSamples samples =
        Just (foldl1 minV samples, foldl1 maxV samples)
        
--partiallyEvalPolyAtPointOut ::
--    (Ord var, ArithInOut.RoundedReal cf) 
--    =>
--    ArithInOut.RoundedRealEffortIndicator cf -> 
--    Map.Map var cf -> 
--    IntPoly var cf -> 
--    IntPoly var cf
--partiallyEvalPolyAtPointOut effCf valsMap _p@(IntPoly cfg terms) =
--    {-  currently there is massive dependency effect here,
--        move this to Composition and deal with it as a special case
--        of composition
--    -}
--    IntPoly cfgVarsRemoved $ pev domsLE terms
--    where
--    cfgVarsRemoved =
--        cfg
--        {
--            ipolycfg_vars = newVars,
--            ipolycfg_domsLE = newDomsLE,
--            ipolycfg_domsLZ = newDomsLZ
--        }
--        where
--        (newVars, newDomsLE, newDomsLZ)
--            = unzip3 $ filter notSubstituted $ zip3 vars domsLE domsLZ
--            where
--            vars = ipolycfg_vars cfg
--            domsLZ = ipolycfg_domsLZ cfg
--            notSubstituted (var, _, _) =
--                var `Map.notMember` valsMap
--    domsLE = ipolycfg_domsLE cfg
--    pev _ t@(IntPolyC _) = t
--    pev (domLE : restDomsLE) (IntPolyV var powers) =
--        case Map.lookup var valsMap of
--            Just value ->
--                let ?addInOutEffort = effAdd in
--                -- evaluate evaluatedPowers using the Horner scheme: 
--                foldl (addAndScale (value <-> domLE) highestExponent) heTerms lowerPowers
--            _ ->
--                IntPolyV var evaluatedPowers
--        where
--        evaluatedPowers =
--            IntMap.map (pev restDomsLE) powers
--        ((highestExponent, heTerms) : lowerPowers) =  
--            reverse $ IntMap.toAscList evaluatedPowers
--        addAndScale value prevExponent termsSoFar (currExponent, currTerms) =
--            let ?multInOutEffort = effMult in
--            addTermsAux currTerms $ termsMapCoeffs (<*> valuePower) termsSoFar
--            where
--            valuePower = 
--                let ?intPowerInOutEffort = effPow in
--                value <^> (prevExponent - currExponent)  
--            addTermsAux (IntPolyV v powers1) (IntPolyV _ powers2) =
--                IntPolyV v $ IntMap.unionWith addTermsAux powers1 powers2
--            addTermsAux (IntPolyC val1) (IntPolyC val2) = 
--                let ?addInOutEffort = effAdd in
--                IntPolyC $ val1 <+> val2 
--        effAdd = ArithInOut.fldEffortAdd sampleCf $ ArithInOut.rrEffortField sampleCf effCf
--        effMult = ArithInOut.fldEffortMult sampleCf $ ArithInOut.rrEffortField sampleCf effCf
--        effPow = ArithInOut.fldEffortPow sampleCf $ ArithInOut.rrEffortField sampleCf effCf
--        sampleCf = domLE
--                
