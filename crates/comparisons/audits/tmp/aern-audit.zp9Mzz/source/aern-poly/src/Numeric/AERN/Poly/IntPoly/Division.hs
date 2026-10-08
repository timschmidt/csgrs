{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Division
    Description :  out-rounded polynomial division
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Out-rounded polynomial division.  Currently very inaccurate.
-}

module Numeric.AERN.Poly.IntPoly.Division
    (
    )
where
    
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.Conversion ()
import Numeric.AERN.Poly.IntPoly.Multiplication ()
import Numeric.AERN.Poly.IntPoly.Evaluation ()

import Numeric.AERN.RmToRn

import Numeric.AERN.RmToRn.RefinementOrderRounding.Reciprocal (recipOutUsingTauEff)

import Numeric.AERN.Basics.Interval (Interval)


--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Numeric.AERN.RealArithmetic.Measures

import qualified 
       Numeric.AERN.RefinementOrder as RefOrd
import qualified 
       Numeric.AERN.NumericOrder as NumOrd


--import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Effort (EffortIndicator)

--import Numeric.AERN.Misc.Debug

import Test.QuickCheck (Arbitrary)


instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithInOut.RoundedDivideEffort (IntPoly var cf) 
    where
    type DivEffortIndicator (IntPoly var cf) = 
        IntPolyEffort cf
    divDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
   (cf ~ Interval e, 
    Ord var, Show var, Show cf, 
    Show (Imprecision cf),
    ArithInOut.RoundedReal cf,
    NumOrd.PartialComparison (Imprecision cf),
    HasAntiConsistency cf, 
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithInOut.RoundedDivide (IntPoly var cf) 
    where
    divOutEff eff p1 p2 = 
        divOutEffUsingRecip (eff, effRealD, effFldFD) tauDegree p1 p2
        where
        effRealD = ipolyeff_cfRoundedRealEffort eff
        effFldFD = eff
        tauDegree = ipolyeff_recipTauDegree eff
    divInEff = 
        error "aern-poly: IntPoly does not support inwards-rounded division" 

divOutEffUsingRecip :: 
   (Ord var, Show var, Show cf, Show (Imprecision cf),
    HasAntiConsistency cf, ArithInOut.RoundedReal cf,
    NumOrd.PartialComparison (Imprecision cf),
    RefOrd.IntervalLike cf,
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
   =>
   (ArithInOut.RingOpsEffortIndicator (IntPoly var cf),
    ArithInOut.RoundedRealEffortIndicator cf,
    ArithInOut.MixedFieldOpsEffortIndicator (IntPoly var cf) cf)
    -> Int -- ^ order or the tau approximation of the reciprocal 
    -> IntPoly var cf 
    -> IntPoly var cf 
    -> IntPoly var cf
divOutEffUsingRecip effRecip@(effRingF, effRealD, _effFldFD) tauDegree f1 f2 =
    f1 <*> f2Recip 
    where
    f2Recip =
        case getConstantIfPolyConstant f2 of
            Just c2 -> 
                newConstFnFromSample sampleF (ArithInOut.recipOutEff effDivD c2)
            _  | polyIsExact f2 == Just True ->
                recipOutUsingTauEff effRecip tauDegree f2
               | otherwise ->
                RefOrd.fromEndpointsOut (f2RRecip, f2LRecip) -- flip because 1/x is decreasing
                where
                f2LRecip = recipOutUsingTauEff effRecip tauDegree f2L
                f2RRecip = recipOutUsingTauEff effRecip tauDegree f2R
                (f2L, f2R) = RefOrd.getEndpointsOut f2
                
    (<*>) = ArithInOut.multOutEff effMultF

    effMultF = ArithInOut.ringEffortMult sampleF effRingF
    sampleF = f1

    effDivD = ArithInOut.fldEffortDiv sampleD effFldD
    effFldD = ArithInOut.rrEffortField sampleD effRealD
    sampleD = getSampleDomValue sampleF 

--divOutEffDummyViaRange ::
--    (CanEvaluate f,
--     HasConstFns f,
--     ArithInOut.RoundedMultiply f)
--    =>
--    (EvaluationEffortIndicator f) ->
--    (ArithInOut.MultEffortIndicator f) ->
--    f -> f -> f
--divOutEffDummyViaRange effEval effMulF f1 f2 =
--    ArithInOut.multOutEff effMulF f1 $ newConstFnFromSample f1 f2Range
--    where
--    f2Range = evalAtPointOutEff effEval (getDomainBox f2) f2 

instance
   (Ord var, Show var, Show cf, Show (Imprecision cf),
    HasAntiConsistency cf, ArithInOut.RoundedReal cf,
    RefOrd.IntervalLike cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    => 
    ArithInOut.RoundedFieldEffort (IntPoly var cf)
    where
    type FieldOpsEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    fieldOpsDefaultEffort (IntPoly cfg _) = 
        ipolycfg_effort cfg 
    fldEffortAdd _ eff = eff 
    fldEffortMult _ eff = eff
    fldEffortPow _ eff = eff
    fldEffortDiv _ eff = eff
  
instance 
   (cf ~ (Interval e),
    Ord var, Show var, Show cf,
    HasAntiConsistency cf, ArithInOut.RoundedReal cf,
    RefOrd.IntervalLike cf,
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf, 
    NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedField (IntPoly var cf)


instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithInOut.RoundedMixedDivideEffort 
        (IntPoly var cf) 
        (IntPoly var cf) 
    where
    type MixedDivEffortIndicator 
            (IntPoly var cf) 
            (IntPoly var cf) = 
        IntPolyEffort cf 
    mixedDivDefaultEffort _ sample = 
        ArithInOut.divDefaultEffort sample

instance
   (cf ~ Interval e, 
    Ord var, Show var, Show cf, 
    Show (Imprecision cf),
    ArithInOut.RoundedReal cf,
    NumOrd.PartialComparison (Imprecision cf),
    HasAntiConsistency cf, 
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithInOut.RoundedMixedDivide 
        (IntPoly var cf)
        (IntPoly var cf)
    where
    mixedDivOutEff = ArithInOut.divOutEff
    mixedDivInEff = 
        error "aern-poly: IntPoly does not support inwards-rounded mixed division" 
        
instance
    (EffortIndicator (IntPolyEffort (Interval e)),
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMixedField (IntPoly var (Interval e)) (Interval e)) 
    =>
    ArithInOut.RoundedMixedFieldEffort 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    type MixedFieldOpsEffortIndicator 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
   (cf ~ Interval e, 
    Ord var, Show var, Show cf, 
    Show (Imprecision cf),
    ArithInOut.RoundedReal cf,
    NumOrd.PartialComparison (Imprecision cf),
    HasAntiConsistency cf, 
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithInOut.RoundedMixedField 
        (IntPoly var cf)
        (IntPoly var cf)
        