{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Composition
    Description :  composition of interval polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Evaluation of interval polynomials in general using its instance of 'CanEvaluateOtherType' 
    and specilised cases for evaluation at a point and on an interval.
-}

module Numeric.AERN.Poly.IntPoly.Composition
()
where

import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New ()
import Numeric.AERN.Poly.IntPoly.Conversion ()
import Numeric.AERN.Poly.IntPoly.Evaluation
import Numeric.AERN.Poly.IntPoly.Show ()
import Numeric.AERN.Poly.IntPoly.NumericOrder ()
import Numeric.AERN.Poly.IntPoly.RefinementOrder ()
import Numeric.AERN.Poly.IntPoly.Addition ()
import Numeric.AERN.Poly.IntPoly.Multiplication ()

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation


import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort ((<+>|))
--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.SizeLimits

import qualified Data.Set as Set

import Test.QuickCheck (Arbitrary)

--import qualified Data.IntMap as IntMap
--import qualified Data.Map as Map

instance
    (Ord var, Show var,
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show cf, Show (SizeLimits cf),
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf))
    =>
    HasEvalOps (IntPoly var cf) (IntPoly var cf)
    where
    type EvalOpsEffortIndicator (IntPoly var cf) (IntPoly var cf) = 
        IntPolyEffort cf
    evalOpsDefaultEffort _ _sampleP@(IntPoly cfg _) = 
        ipolycfg_effort cfg
    evalOpsEff eff _ sampleP =
        polyPolyEvalOps eff sampleP sampleCf
        where
        sampleCf = getSampleDomValue sampleP
    
{-|
    Operations for evaluating a polynomial in which each
    variable is evaluated to a polynomial.
-}
polyPolyEvalOps ::
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     RefOrd.IntervalLike cf, 
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     Arbitrary cf,
     NumOrd.PartialComparison (Imprecision cf),
     Show (Imprecision cf))
    =>
   IntPolyEffort cf ->
   (IntPoly var cf) ->
   cf ->
   PolyEvalOps var cf (IntPoly var cf)
polyPolyEvalOps eff sampleP sampleCf =
    result
    where
    result =
        let (<+>) = ArithInOut.addOutEff eff in
        let (<*>) = ArithInOut.multOutEff eff in
        let (<^>) = ArithInOut.powerToNonnegIntOutEff eff in
        let (<=?) = NumOrd.pLeqEff eff in
        PolyEvalOps (zero sampleP) (<+>) (<*>) (<^>) (newConstFnFromSample sampleP) (const Nothing) maxSplitSize $
            Just $ PolyEvalMonoOps
                result
                (<=?)
                RefOrd.getEndpointsOut
                RefOrd.fromEndpointsOut
                isDefinitelyExact
                split
                join
                (curry join) -- a very dummy min
                (curry join) -- a very dummy max
                getWidthAsDoubleDummy
                effCf
    split val = (val1, val2)
        where
        val1 = RefOrd.fromEndpointsOut (valL, valM)
        val2 = RefOrd.fromEndpointsOut (valM, valR)
        (valL, valR) = RefOrd.getEndpointsOut val
        valM =
            let (<+>) = ArithInOut.addOutEff eff in
            let (</>|) = ArithInOut.mixedDivOutEff eff in
            (valL <+> valR) </>| (2 :: Int)
    getWidthAsDoubleDummy _ = 0 -- no splitting...
    join = 
        let (</\>) = RefOrd.meetOutEff effJoinCf in
        polyJoinWith (zero sampleCf) $ uncurry (</\>)
    isDefinitelyExact p = 
        polyIsExact p == Just True
    effJoinCf = ArithInOut.rrEffortJoinMeet sampleCf effCf

    maxSplitSize = ipolyeff_evalMaxSplitSize eff
    effCf = ipolyeff_cfRoundedRealEffort eff
        
        
        
instance 
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf,
     Arbitrary cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf)
    => 
    (CanCompose (IntPoly var cf))
    where
    type CompositionEffortIndicator (IntPoly var cf) = 
        IntPolyEffort cf
    compositionDefaultEffort p = evaluationDefaultEffort p
    composeVarsOutEff = 
        polyComposeVarsOutEff
    composeVarsInEff = 
        polyComposeVarsInEff

instance
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf,
     Arbitrary cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf)
    => 
    CanPartiallyEvaluate (IntPoly var cf)
    where
    type PartialEvaluationEffortIndicator (IntPoly var cf) = 
        CompositionEffortIndicator (IntPoly var cf)
    partialEvaluationDefaultEffort p =
        compositionDefaultEffort p
    pEvalAtPointOutEff effComp evalValuesBox p@(IntPoly cfg _) = 
        composeVarsOutEff effComp composeBox p
        where
        composeBox =
            fromAscList $ 
                map getVarComposeValue $
                    evalValuesBoxAscList
        evalValuesBoxAscList = 
            toAscList evalValuesBox
        getVarComposeValue (var,value) =
            (var, newConstFn limits varDomsWithoutEvalVars value)
--        cfgWithoutEvalVars =
--            cfgAdjustDomains varsWithoutEvalVars domsWithoutEvalVars cfg 
--        (varsWithoutEvalVars, domsWithoutEvalVars) = 
--            unzip varDomsWithoutEvalVars
        varDomsWithoutEvalVars =
            filter notSubstitutedVarDom varDoms
            where
            notSubstitutedVarDom (var, _) =
                not $ Set.member var substitutedVarsSet
        substitutedVarsSet = 
            Set.fromList $ fst $ unzip evalValuesBoxAscList
        varDoms = getVarDoms p
        limits = ipolycfg_limits cfg
    pEvalAtPointInEff effComp evalValuesBox p@(IntPoly cfg _) =
        composeVarsInEff effComp composeBox p
        where
        composeBox =
            fromAscList $ 
                map getVarComposeValue $
                    evalValuesBoxAscList
        evalValuesBoxAscList = 
            toAscList evalValuesBox
        getVarComposeValue (var,value) =
            (var, newConstFn limits varDomsWithoutEvalVars value)
--        cfgWithoutEvalVars =
--            cfgAdjustDomains varsWithoutEvalVars domsWithoutEvalVars cfg 
--        (varsWithoutEvalVars, domsWithoutEvalVars) = unzip varDomsWithoutEvalVars
        varDomsWithoutEvalVars =
            filter notSubstitutedVarDom varDoms
            where
            notSubstitutedVarDom (var, _) =
                not $ Set.member var substitutedVarsSet
        substitutedVarsSet = 
            Set.fromList $ fst $ unzip evalValuesBoxAscList
        varDoms = getVarDoms p
        limits = ipolycfg_limits cfg

---- TODO: move this function to aern-order
--performEndpointWiseWithDefaultEffort ::
--   (RefOrd.IntervalLike t1,
--    RefOrd.IntervalLike t2) 
--    =>
--   (t1 -> t2) 
--   -> 
--   (t1 -> t2)
--performEndpointWiseWithDefaultEffort fn p =
--    RefOrd.fromEndpointsOut (resL, resR)
--    where
--    resL = fn pL
--    resR = fn pR
--    (pL, pR) = RefOrd.getEndpointsOut p
     
        
-- TODO: the following functions should be made independent of IntPoly and moved to aern-realfn        
polyComposeVarsOutEff ::
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     f ~ (IntPoly var cf))
    =>
    CompositionEffortIndicator f ->
    VarBox f f ->
    f ->
    f 
polyComposeVarsOutEff eff substBox p@(IntPoly cfg _) =
    result
    where
    vars = ipolycfg_vars cfg
    result =
--        unsafePrintReturn
--        (
--            "polyComposeVarsOutEff:"
--            ++ "\n varSubstBox = " ++ show varSubstBox
--            ++ "\n p = " ++ show p
--            ++ "\n result = "
--        ) $
        evalOtherType ops varSubstBox p
        where
        varSubstBox =
            fromList $ map makeVarSubst vars
        makeVarSubst var = 
            case lookupVar substBox var of
                Just substPoly -> (var, substPoly)
                _ -> (var,  proj)
            where
            proj =
                newProjectionFromSample sampleP var
    ops =
        polyPolyEvalOps eff sampleP sampleCf
    ((_,sampleP) : _) = toAscList substBox
    sampleCf = getSampleDomValue sampleP

polyComposeVarsInEff ::
    (Ord var, Show var, 
     Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf, 
     HasAntiConsistency cf,
     Arbitrary cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf,
     f ~ (IntPoly var cf))
    =>
    CompositionEffortIndicator f ->
    VarBox f f ->
    f ->
    f 
polyComposeVarsInEff eff substBox p =
    result
    where
    vars = getVars $ getDomainBox p -- ipolycfg_vars cfg
    result =
--        unsafePrintReturn
--        (
--            "polyComposeVarsOutEff:"
--            ++ "\n varSubstBox = " ++ show varSubstBox
--            ++ "\n p = " ++ show p
--            ++ "\n result = "
--        ) $
        evalOtherTypeInner ops varSubstBox p
        where
        varSubstBox =
            fromList $ map makeVarSubst vars
        makeVarSubst var = 
            case lookupVar substBox var of
                Just substPoly -> (var, substPoly)
                _ -> (var,  proj)
            where
            proj =
                newProjectionFromSample sampleP var
    ops =
        polyPolyEvalOps eff sampleP sampleCf
    ((_,sampleP) : _) = toAscList substBox
    sampleCf = getSampleDomValue sampleP

    
{- 
    the following are optimized versions of the above; 
    TODO: let the above detect the special situations and use these optimizations 
-}
        
--polyComposeMainVarKeepOutEff ::
--    (Ord var, Show var, 
--     Show cf,
--     ArithInOut.RoundedReal cf,
--     HasAntiConsistency cf,
--     Show (Imprecision cf),
--     NumOrd.PartialComparison (Imprecision cf),
--     RefOrd.IntervalLike cf)
--    => 
--    (ArithInOut.RoundedRealEffortIndicator cf) ->
--    IntPoly var cf {- polynomial to substitute the main var with -} ->
--    IntPoly var cf -> 
--    IntPoly var cf
--polyComposeMainVarKeepOutEff eff substPoly p@(IntPoly cfg _) =
----    unsafePrint
----    (
----        "polyComposeMainVarKeepOutEff: "
----        ++ "\n p = " ++ showP p
----        ++ "\n substPoly = " ++ showP substPoly
----        ++ "\n result = " ++ showP result
----    ) $
--    result
--    where
--    (mainVar : otherVars) = ipolycfg_vars cfg
--    result =
--        evalOtherType opsMainVarOnly varvaluesBox p
--        where
--        varvaluesBox =
--            fromList $ (mainVar, substPoly) : otherVarsSubsts
--        otherVarsSubsts =
--            map makeVarSubst $ zip otherVars $ tail $ ipolycfg_domsLE cfg
--        makeVarSubst (var, domLE) =
--            (var,  proj)
--            where
--            proj =
--                newProjectionFromSample p var
--    opsMainVarOnly =
--        (polyPolyEvalOps (Int1To1000 0, eff) sampleP sampleCf)
--            {
--                polyEvalMaybePoly = polyEvalCopySubPolynomials
--            }
--    polyEvalCopySubPolynomials subterms 
--        | isMainVar = Nothing
--        | otherwise =
--            Just $
--                IntPoly cfg $ IntPolyV mainVar $ IntMap.singleton 0 subterms
--        where
--        isMainVar =
--            case subterms of
--                (IntPolyV var _) -> var == mainVar
--                _ -> False
--            
--    sampleP = p
--    sampleCf = getSampleDomValue p
--
--
--polyComposeMainVarElimOutEff ::
--    (Ord var, Show var, 
--     Show cf,
--     ArithInOut.RoundedReal cf, 
--     HasAntiConsistency cf,
--     Show (Imprecision cf),
--     NumOrd.PartialComparison (Imprecision cf),
--     RefOrd.IntervalLike cf)
--    => 
--    (ArithInOut.RoundedRealEffortIndicator cf) ->
--    IntPoly var cf {- polynomial to substitute the main var with -} ->
--    IntPoly var cf -> 
--    IntPoly var cf
--polyComposeMainVarElimOutEff eff substPoly p@(IntPoly cfg _) =
----    unsafePrint
----    (
----        "substPolyMainVar: "
----        ++ "\n p = " ++ showP p
----        ++ "\n substPoly = " ++ showP substPoly
----        ++ "\n result = " ++ showP result
----    ) $
--    result
--    where
--    (mainVar : otherVars) = ipolycfg_vars cfg
--    result =
--        evalOtherType opsMainVarOnly varvaluesBox p
--        where
--        varvaluesBox =
--            fromList $ (mainVar, substPoly) : otherVarsSubsts
--        otherVarsSubsts =
--            map makeVarSubst otherVars
--        makeVarSubst var =
--            (var, newProjectionFromSample sampleP var)
--    opsMainVarOnly =
--        (polyPolyEvalOps (Int1To1000 0, eff) sampleP sampleCf)
--            {
--                polyEvalMaybePoly = polyEvalCopySubPolynomials
--            }
--    polyEvalCopySubPolynomials subterms 
--        | isMainVar = Nothing
--        | otherwise =
--            Just $ IntPoly cfgR subterms
--        where
--        cfgR = cfgRemFirstVar cfg
--        isMainVar =
--            case subterms of
--                (IntPolyV var _) -> var == mainVar
--                _ -> False
--            
--    sampleP = substPoly
--    sampleCf = getSampleDomValue p
--
--
