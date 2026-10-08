{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE CPP #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Minmax
    Description :  pointwise min/max
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Pointwise min/max of interval polynomials rounding up/down/in/out.
-}

module Numeric.AERN.Poly.IntPoly.Minmax
(
    minmaxUpDnDefaultEffortIntPolyWithBezierDegree,
    minmaxInOutDefaultEffortIntPolyWithBezierDegree
)
where

import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New ()
import Numeric.AERN.Poly.IntPoly.Evaluation ()
import Numeric.AERN.Poly.IntPoly.NumericOrder ()
import Numeric.AERN.Poly.IntPoly.RefinementOrder ()
import Numeric.AERN.Poly.IntPoly.Addition ()
import Numeric.AERN.Poly.IntPoly.Multiplication () 
import Numeric.AERN.Poly.IntPoly.Composition ()

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.Evaluation

--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Comparison
--import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Arbitrary
import Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Minmax

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (AbsEffortIndicator)

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder 
    (MinmaxEffortIndicator, MinmaxInOutEffortIndicator)
    -- needed for ghc 6.12
import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder
    (JoinMeetEffortIndicator)
    -- needed for ghc 6.12
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Basics.Interval
--import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

import Test.QuickCheck (Arbitrary)

import Numeric.AERN.Misc.Debug
_ = unsafePrint

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     NumOrd.RefinementRoundedLatticeEffort (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    NumOrd.RoundedLatticeEffort (IntPoly var (Interval e))
    where
    type MinmaxEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    minmaxDefaultEffort _p@(IntPoly cfg _) =
        ipolycfg_effort cfg

minmaxUpDnDefaultEffortIntPolyWithBezierDegree ::
    Int -> 
    (IntPoly var cf) -> 
    IntPolyEffort cf
minmaxUpDnDefaultEffortIntPolyWithBezierDegree degree _p@(IntPoly cfg _) =
    (ipolycfg_effort cfg)
    {
        ipolyeff_minmaxBernsteinDegree = degree
    }

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     NumOrd.RefinementRoundedLattice (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    NumOrd.RoundedLattice (IntPoly var (Interval e))
    where
    maxUpEff eff a b =
--        unsafePrint 
--            ( "IntPoly maxUpEff:"
--                ++ "\n degreeMinusOne = " ++ show degreeMinusOne
--                ++ "\n cfg a = " ++ show (intpoly_cfg a)
--                ++ "\n cfg b = " ++ show (intpoly_cfg b)
--                ++ "\n a = " ++ showPoly show show a
--                ++ "\n b = " ++ showPoly show show b
--                ++ "\n a `maxUp` b = " ++ showPoly show show result 
--            ) $
        result
        where
        result =
            case (getConstantIfPolyConstant a, getConstantIfPolyConstant b) of
                (Just aC, Just bC) ->
                    newConstFnFromSample a $ NumOrd.maxOutEff effMinmaxDom aC bC 
                _ ->
                    fst $ maxUpEffFromRingOps a getX effMinmax (getDegree degree a) aR bR
        (_aL,aR) = RefOrd.getEndpointsOutEff () a
        (_bL,bR) = RefOrd.getEndpointsOutEff () b
        
        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        
        degree = ipolyeff_minmaxBernsteinDegree eff
        effMinmaxDom = ipolyeff_cfMinMaxEffort eff
--        effDom = ipolyeff_cfRoundedRealEffort eff
        
        
    maxDnEff eff a b =
        result
        where
        result =
            case (getConstantIfPolyConstant a, getConstantIfPolyConstant b) of
                (Just aC, Just bC) ->
                    newConstFnFromSample a $ NumOrd.maxOutEff effMinmaxDom aC bC 
                _ ->
                    maxDnEffFromRingOps a getX effMinmax (getDegree degree a) aL bL
        (aL,_aR) = RefOrd.getEndpointsOutEff () a
        (bL,_bR) = RefOrd.getEndpointsOutEff () b
        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        degree = ipolyeff_minmaxBernsteinDegree eff
        effMinmaxDom = ipolyeff_cfMinMaxEffort eff

    minUpEff eff a b = 
        result
        where
        result =
            case (getConstantIfPolyConstant a, getConstantIfPolyConstant b) of
                (Just aC, Just bC) ->
                    newConstFnFromSample a $ NumOrd.minOutEff effMinmaxDom aC bC 
                _ ->
                    minUpEffFromRingOps a getX effMinmax (getDegree degree a) aR bR
        (_aL,aR) = RefOrd.getEndpointsOutEff () a
        (_bL,bR) = RefOrd.getEndpointsOutEff () b
        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        degree = ipolyeff_minmaxBernsteinDegree eff
        effMinmaxDom = ipolyeff_cfMinMaxEffort eff

    minDnEff eff a b = 
        result
        where
        result =
            case (getConstantIfPolyConstant a, getConstantIfPolyConstant b) of
                (Just aC, Just bC) ->
                    newConstFnFromSample a $ NumOrd.minOutEff effMinmaxDom aC bC 
                _ ->
                    fst $ minDnEffFromRingOps a getX effMinmax (getDegree degree a) aL bL
        (aL,_aR) = RefOrd.getEndpointsOutEff () a
        (bL,_bR) = RefOrd.getEndpointsOutEff () b
        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        degree = ipolyeff_minmaxBernsteinDegree eff
        effMinmaxDom = ipolyeff_cfMinMaxEffort eff

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    ArithUpDn.RoundedAbsEffort (IntPoly var (Interval e))
    where
    type AbsEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    absDefaultEffort _p@(IntPoly cfg _) =
        ipolycfg_effort cfg

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    ArithUpDn.RoundedAbs (IntPoly var (Interval e))
    where
    absUpEff eff p =
        case getConstantIfPolyConstant p of
            Just c -> 
                newConstFnFromSample p $ 
                    snd $ RefOrd.getEndpointsOut $ 
                        ArithInOut.absOutEff effAbsDom c
            _ -> NumOrd.maxUpEff effMinmax p (neg p)
        where
        effMinmax = eff
        effAbsDom = ipolyeff_cfAbsEffort eff
    absDnEff eff p =
        case getConstantIfPolyConstant p of
            Just c -> 
                newConstFnFromSample p $ 
                    fst $ RefOrd.getEndpointsOut $ 
                        ArithInOut.absOutEff effAbsDom c
            _ -> NumOrd.maxDnEff effMinmax p (neg p)
        where
        effMinmax = eff
        effAbsDom = ipolyeff_cfAbsEffort eff


instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    NumOrd.RefinementRoundedLatticeEffort (IntPoly var (Interval e))
    where
    type MinmaxInOutEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)

    minmaxInOutDefaultEffort _p@(IntPoly cfg _) =
        ipolycfg_effort cfg

minmaxInOutDefaultEffortIntPolyWithBezierDegree :: 
    Int -> IntPoly var cf -> IntPolyEffort cf
minmaxInOutDefaultEffortIntPolyWithBezierDegree degree _p@(IntPoly cfg _) =
    (ipolycfg_effort cfg)
    {
        ipolyeff_minmaxBernsteinDegree = degree
    }

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    NumOrd.RefinementRoundedLattice (IntPoly var (Interval e))
    where
    maxOutEff eff a b =
        result
        where
        result =
            makeCoeffsConsistentOut $ 
            RefOrd.fromEndpointsOutEff () (resL,resR)
        resL = maxDnEffFromRingOps a getX effMinmax (getDegree degree a) aL bL
        (resR, _) = maxUpEffFromRingOps a getX effMinmax (getDegree degree a) aR bR
        (aL,aR) = RefOrd.getEndpointsOutEff () a
        (bL,bR) = RefOrd.getEndpointsOutEff () b

        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        degree = ipolyeff_minmaxBernsteinDegree eff

    maxInEff =
        error "aern-poly: inner-rounded max not available for IntPoly"
    minOutEff eff a b =
        result
        where
        result = 
            makeCoeffsConsistentOut $ 
            RefOrd.fromEndpointsOutEff () (resL,resR)
        (resL, _) = minDnEffFromRingOps a getX effMinmax (getDegree degree a) aL bL
        resR = minUpEffFromRingOps a getX effMinmax (getDegree degree a) aR bR
        (aL,aR) = RefOrd.getEndpointsOutEff () a
        (bL,bR) = RefOrd.getEndpointsOutEff () b

        effMinmax = effMinmaxFromIntPolyEffort sampleP eff
            where
            sampleP = a
        
        degree = ipolyeff_minmaxBernsteinDegree eff

    minInEff =
        error "aern-poly: inner-rounded min not available for IntPoly"
    

effMinmaxFromIntPolyEffort :: 
    (Ord var, Show var,
     cf ~ Interval e,
     ArithInOut.RoundedReal cf, RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
     =>
    IntPoly var cf
    -> IntPolyEffort cf 
    -> MinmaxEffortIndicatorFromRingOps (IntPoly var cf) (IntPoly var cf)
effMinmaxFromIntPolyEffort sampleP eff =
    MinmaxEffortIndicatorFromRingOps
    {
        minmaxFromRO_eff_convertTDF = eff,
        minmaxFromRO_eff_roundedRealD = ipolyeff_cfRoundedRealEffort eff,
        minmaxFromRO_eff_getEndpointsD = ipolyeff_cfGetEndpointsEffort eff,
        minmaxFromRO_eff_evalFT = eff,
        minmaxFromRO_eff_evalFDF = eff,
        minmaxFromRO_eff_ringOpsF = eff,
        minmaxFromRO_eff_mixmultFDF = eff,
        minmaxFromRO_eff_sizelimitsF = getSizeLimits sampleP,
        minmaxFromRO_eff_ringOpsT = eff,
        minmaxFromRO_eff_mixedFldTDF = eff
    }
--    where
--    sampleCf = ipolycfg_sample_cf cfg
--    (IntPoly cfg _) = sampleP
    
instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    ArithInOut.RoundedAbsEffort (IntPoly var (Interval e))
    where
    type AbsEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    absDefaultEffort _p@(IntPoly cfg _) =
        ipolycfg_effort cfg

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency (Interval e), 
     Arbitrary (Interval e), 
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     Show var,
     Show (Interval e),
     Show (Imprecision (Interval e)),
     NumOrd.PartialComparison (Imprecision (Interval e)))
    =>
    ArithInOut.RoundedAbs (IntPoly var (Interval e))
    where
    absOutEff eff p =
        case getConstantIfPolyConstant p of
            Just c -> newConstFnFromSample p $ ArithInOut.absOutEff effCfAbs c
            _ -> NumOrd.maxOutEff eff p (neg p)
        where
        effCfAbs = ipolyeff_cfAbsEffort eff
    absInEff =
        error "aern-poly: inner-rounded abs not available for IntPoly"
    
instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency cf, 
     Arbitrary cf, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedField cf cf,
     Show var,
     Show cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf))
    =>
    RefOrd.RoundedLatticeEffort (IntPoly var cf)
    where
    type JoinMeetEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    joinmeetDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg

--joinmeetDefaultEffortIntPolyWithBezierDegree degree f =
--    (defaultMinmaxEffortIndicatorFromRingOps f f,
--     Int1To10 (degree - 1),
--     RefOrd.getEndpointsDefaultEffort f,
--     RefOrd.fromEndpointsDefaultEffort f)

instance
    (Ord var,
     GeneratableVariables var,
     HasAntiConsistency cf, 
     Arbitrary cf, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedField cf cf,
     Show var,
     Show cf,
     Show (Imprecision cf),
     NumOrd.PartialComparison (Imprecision cf))
    =>
    RefOrd.RoundedLattice (IntPoly var cf)
    where
    joinOutEff =
        error "aern-poly: join not defined for IntPoly"
    joinInEff =
        error "aern-poly: join not defined for IntPoly"
    meetOutEff eff a b@(IntPoly cfg _) =
        polyJoinWith (zero sampleCf) (uncurry $ RefOrd.meetOutEff effCfJoin) (a, b)
        where
        effCfJoin = ArithInOut.rrEffortJoinMeet sampleCf $ ipolyeff_cfRoundedRealEffort eff
        sampleCf = ipolycfg_sample_cf cfg
    meetInEff =
        error "aern-poly: inner-rounded meet not defined for IntPoly"

{-|
    Take a sample polynomial with at least one variable in its domain
    and return the univariate polynomial \x : [0,1] -> x.  
-}
getX ::
    (Ord var, Show var, 
     Show cf, HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) 
    =>
    (IntPoly var cf) -> (IntPoly var cf)
getX (IntPoly (IntPolyCfg vars _ _ sample limits) _) =
    newProjection limits [(var, unit)] var
    where
    var = head vars
    unit = RefOrd.fromEndpointsOut (zero sample, one sample)
    
getDegree :: Int -> IntPoly var cf -> Int
getDegree degree (IntPoly cfg _) =
    max 2 $ min degree maxDeg
    where
    maxDeg = ipolycfg_maxdeg cfg