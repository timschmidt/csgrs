{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Interval
    Description :  intervals whose endpoints are interval polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Polynomial intervals, ie intervals whose endpoints are interval polynomials.  
    In particular, evaluation of polynomial intervals in general, using its instance of 'CanEvaluateOtherType'.
-}

module Numeric.AERN.Poly.IntPoly.Interval
()
where

import Numeric.AERN.Poly.IntPoly

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation


import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort ((<+>|))
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.RefinementOrder as RefOrd

import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.Interval
import Numeric.AERN.RealArithmetic.Interval (IntervalRealEffort(..), intrealeff_intordeff)
import Numeric.AERN.RealArithmetic.Interval.MixedFieldOps (intervalMixedAddInEff, intervalMixedAddOutEff)

import Numeric.AERN.RmToRn.Interval ()

import Numeric.AERN.Basics.Consistency

import Test.QuickCheck (Arbitrary)

import Numeric.AERN.Misc.Debug

_ = unsafePrint

instance
    (Ord var, Show var, GeneratableVariables var, 
     ArithInOut.RoundedReal (Interval e), 
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     HasAntiConsistency (Interval e),
     RefOrd.IntervalLike (Interval e), 
     ArithUpDn.Convertible (Interval e) (Interval e),
     Show (Interval e), Arbitrary (Interval e), 
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     NumOrd.HasExtrema e, ArithUpDn.RoundedReal e
    )
    =>
    HasEvalOps (IntPoly var (Interval e)) (Interval (IntPoly var (Interval e)))
    where
    type EvalOpsEffortIndicator (IntPoly var (Interval e)) (Interval (IntPoly var (Interval e))) = 
        IntPolyEffort (Interval e)
    evalOpsDefaultEffort _sampleP@(IntPoly cfg _) _samplePI = 
        ipolycfg_effort cfg
    evalOpsEff eff sampleP samplePI =
        intpolyPolyEvalOps eff samplePI sampleCf
        where
        sampleCf = getSampleDomValue sampleP
        
        
intpolyPolyEvalOps ::
    (Ord var, Show var, GeneratableVariables var, 
     Show cf, Arbitrary cf, 
     ArithUpDn.Convertible cf cf,
     RefOrd.IntervalLike cf, 
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf cf,
     HasAntiConsistency cf,
     NumOrd.PartialComparison (Imprecision cf),
     Show (Imprecision cf),
     NumOrd.RoundedLattice poly,
     poly ~ IntPoly var cf, cf ~ Interval e,
     ArithUpDn.RoundedReal e,
     NumOrd.HasExtrema e)
    =>
   IntPolyEffort cf ->
   (Interval poly) ->
   cf ->
   PolyEvalOps var cf (Interval poly)
intpolyPolyEvalOps effIP samplePI@(Interval sampleIP _) sampleCf =
    result
    where
    result =
        let (<+>) = ArithInOut.addOutEff  (ArithInOut.ringEffortAdd samplePI effPI) in
        let (<*>) = ArithInOut.multOutEff (ArithInOut.ringEffortMult samplePI effPI) in
        let (<^>) = ArithInOut.powerToNonnegIntOutEff (ArithInOut.ringEffortPow samplePI effPI) in
        let (<=?) = NumOrd.pLeqEff (intrealeff_intordeff sampleIP effPI) in
        PolyEvalOps (zero samplePI) (<+>) (<*>) (<^>) (newConstFnFromSample samplePI) (const Nothing) maxSplitSize $
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
            let (<+>) = ArithInOut.addOutEff (ArithInOut.ringEffortAdd samplePI effPI) in
            let (<*>) = ArithInOut.multOutEff (ArithInOut.ringEffortMult samplePI effPI) in
            (valL <+> valR) <*> (newConstFnFromSample samplePI half)
            where
            half = ArithInOut.mixedAddOutEff effAddCfDbl (zero sampleCf) (0.5 :: Double)
            effAddCfDbl = 
                ArithInOut.mxfldEffortAdd sampleCf (1::Double) $ 
                    ArithInOut.rrEffortDoubleMixedField sampleCf effCf
    getWidthAsDoubleDummy _ = 0 -- no splitting...
--    effAddCf = ArithInOut.fldEffortAdd sampleCf $ ArithInOut.rrEffortField sampleCf effCf
    join (Interval l1 _, Interval _ r2) = Interval l1 r2 
    isDefinitelyExact (Interval l r) = 
        (NumOrd.pEqualEff effIP l r) == Just True

    effPI = ipolyeff_intrealeff sampleIP effIP
    maxSplitSize = ipolyeff_evalMaxSplitSize effIP
    effCf = ipolyeff_cfRoundedRealEffort effIP
        
ipolyeff_intrealeff sampleIP effIP =
    IntervalRealEffort
    {
        -- TODO
    }
