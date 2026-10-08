{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Multiplication
    Description :  out-rounded polynomial multiplication
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Out-rounded polynomial multiplication and scaling.
-}

module Numeric.AERN.Poly.IntPoly.Multiplication
    (
        multTerms,
        powTerms,
        scaleTerms
    )
where
    
import Prelude hiding ((+),(*))
import qualified Prelude
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New
import Numeric.AERN.Poly.IntPoly.Reduction
import Numeric.AERN.Poly.IntPoly.Addition

--import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding
    (MultEffortIndicator, PowerToNonnegIntEffortIndicator, RingOpsEffortIndicator, 
     MixedMultEffortIndicator, MixedRingOpsEffortIndicator, 
     MixedDivEffortIndicator, MixedFieldOpsEffortIndicator)

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.Auxiliary

import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Interval (Interval)
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Effort (EffortIndicator)
import Numeric.AERN.Basics.SizeLimits (SizeLimits)
--import Numeric.AERN.Basics.ShowInternals

--import Numeric.AERN.Misc.Debug

import qualified Data.IntMap as IntMap

import Test.QuickCheck (Arbitrary)

instance
    (EffortIndicator (IntPolyEffort cf)) =>
    ArithInOut.RoundedMultiplyEffort (IntPoly var cf) 
    where
    type MultEffortIndicator (IntPoly var cf) = 
        IntPolyEffort cf 
    multDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedReal cf,
     NumOrd.PartialComparison (Imprecision cf),
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf, 
     Arbitrary cf,
     Ord var, Show var, Show cf) 
    =>
    ArithInOut.RoundedMultiply (IntPoly var cf) 
    where
    multOutEff eff = multPolysOut $ ipolyeff_cfRoundedRealEffort eff 
    multInEff = 
        error "aern-poly: IntPoly does not support inwards-rounded multiplication" 

multPolysOut ::
    (Ord var, Show var, Show cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedReal cf, 
     NumOrd.PartialComparison (Imprecision cf),
     HasAntiConsistency cf,
     Arbitrary cf) 
    =>
    (ArithInOut.RoundedRealEffortIndicator cf) -> 
    IntPoly var cf -> IntPoly var cf -> IntPoly var cf
multPolysOut eff p1@(IntPoly cfg1 terms1) (IntPoly cfg2 terms2)
    =
    reducePolyTermCountOut eff $ 
        reducePolyDegreeOut eff $ 
            IntPoly cfg $
                termsNormalise $ 
                    let (<+>>) = ArithInOut.addOutEff effAdd in
                    let (<*>>) = ArithInOut.multOutEff effMult in
                    multTerms (<+>>) (<*>>) terms1 terms2
    where
    cfg = combineIntPolyCfgs cfg1 cfg2
    effMult = ArithInOut.fldEffortMult sample $ ArithInOut.rrEffortField sample eff
    effAdd = ArithInOut.fldEffortAdd sample $ ArithInOut.rrEffortField sample eff
    sample = getSampleDomValue p1
    
multTerms ::
    (Show var, Show cf) 
    =>
    (cf -> cf -> cf) -> 
    (cf -> cf -> cf) -> 
    IntPolyTerms var cf -> IntPolyTerms var cf -> IntPolyTerms var cf
multTerms _ (*) _terms1@(IntPolyC val1) _terms2@(IntPolyC val2) 
    = 
    IntPolyC $ val1 * val2 
multTerms (+) (*) _terms1@(IntPolyV _ powers1) _terms2@(IntPolyV xName2 powers2)
    =
    IntPolyV xName2 multSubPolys
    where
    multSubPolys 
        = IntMap.fromListWith (addTerms (+)) $
            [(n1 Prelude.+ n2, multTerms (+) (*) t1 t2) | 
                (n1, t1) <- IntMap.toAscList powers1, 
                (n2, t2) <- IntMap.toAscList powers2 ] 
multTerms _ _ terms1 terms2
    =
    error $ "AERN internal error: multTerms: incompatible operands: "
        ++ "\n terms1 = " ++ show terms1
        ++ "\n terms2 = " ++ show terms2


instance
    (EffortIndicator (IntPolyEffort cf)) 
    =>
    ArithInOut.RoundedPowerToNonnegIntEffort (IntPoly var cf)
    where
    type PowerToNonnegIntEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    powerToNonnegIntDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedReal cf, 
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf, 
     HasAntiConsistency cf,
     Show var, Show cf, Ord var) 
    =>
    ArithInOut.RoundedPowerToNonnegInt (IntPoly var cf) 
    where
    powerToNonnegIntInEff eff p n =
        flipConsistencyPoly $ 
            ArithInOut.powerToNonnegIntOutEff eff (flipConsistencyPoly p) n
    powerToNonnegIntOutEff eff (IntPoly cfg terms) n = 
        IntPoly cfg $ 
            powTerms
                effCf
                sample cfg
                (ArithInOut.addOutEff effAdd) 
                (ArithInOut.multOutEff effMult) 
                terms n
        where
        sample = ipolycfg_sample_cf cfg
        effMult = ArithInOut.fldEffortMult sample $ ArithInOut.rrEffortField sample effCf
        effAdd = ArithInOut.fldEffortAdd sample $ ArithInOut.rrEffortField sample effCf
        effCf = ipolyeff_cfRoundedRealEffort eff
        
powTerms ::
    (Ord var, Show var, Show cf,
     ArithInOut.RoundedReal cf,
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf, 
     HasAntiConsistency cf) 
    =>
    (ArithInOut.RoundedRealEffortIndicator cf) ->
    cf ->
    (IntPolyCfg var cf) ->
    (cf -> cf -> cf) ->
    (cf -> cf -> cf) ->
    IntPolyTerms var cf -> Int -> IntPolyTerms var cf
powTerms eff sample cfg (+) (*) = 
    powerFromMult 
        (mkConstTerms (one sample) vars) 
        multTermsReduce
        where
        vars = ipolycfg_vars cfg
        multTermsReduce t1 t2 =
            reduceTermsTermCountOut eff cfg $ 
                reduceTermsDegreeOut eff cfg $ 
                    termsNormalise $ multTerms (+) (*) t1 t2
    
    
instance
    (EffortIndicator (IntPolyEffort cf)) 
    =>
    ArithInOut.RoundedMixedMultiplyEffort (IntPoly var cf) Integer 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Integer = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiply cf Integer, Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedMultiply (IntPoly var cf) Integer 
    where
    mixedMultOutEff = mixedMultOutEffGeneric ArithInOut.rrEffortIntegerMixedField 0
    mixedMultInEff = mixedMultInEffGeneric  ArithInOut.rrEffortIntegerMixedField 0

instance
    (EffortIndicator (IntPolyEffort cf)) 
    =>
    ArithInOut.RoundedMixedMultiplyEffort (IntPoly var cf) Int 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Int = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiply cf Int, Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedMultiply (IntPoly var cf) Int 
    where
    mixedMultOutEff = mixedMultOutEffGeneric ArithInOut.rrEffortIntMixedField 0
    mixedMultInEff = mixedMultInEffGeneric  ArithInOut.rrEffortIntMixedField 0

instance
    (EffortIndicator (IntPolyEffort cf)) 
    =>
    ArithInOut.RoundedMixedMultiplyEffort (IntPoly var cf) Rational 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Rational = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiply cf Rational,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedMultiply (IntPoly var cf) Rational 
    where
    mixedMultOutEff = mixedMultOutEffGeneric ArithInOut.rrEffortRationalMixedField 0
    mixedMultInEff = mixedMultInEffGeneric  ArithInOut.rrEffortRationalMixedField 0

instance
    (EffortIndicator (IntPolyEffort cf)) 
    =>
    ArithInOut.RoundedMixedMultiplyEffort (IntPoly var cf) Double 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Double = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiply cf Double,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedMultiply (IntPoly var cf) Double 
    where
    mixedMultOutEff = mixedMultOutEffGeneric ArithInOut.rrEffortDoubleMixedField 0
    mixedMultInEff = mixedMultInEffGeneric  ArithInOut.rrEffortDoubleMixedField 0


mixedMultOutEffGeneric, mixedMultInEffGeneric :: 
    (Ord var, Show cf, Show var, ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf, HasConsistency cf,
     ArithInOut.RoundedMixedField cf t) 
     =>
    (cf -> ArithInOut.RoundedRealEffortIndicator cf -> ArithInOut.MixedFieldOpsEffortIndicator cf t)
    -> t
    -> IntPolyEffort cf
    -> IntPoly var cf -> t -> IntPoly var cf
mixedMultOutEffGeneric rrEffortMixedField sampleT eff (IntPoly cfg terms) a = 
    IntPoly cfg $ scaleTerms (*|) a terms
    where
    (*|) = ArithInOut.mixedMultOutEff effMixedMult
    effMixedMult = ArithInOut.mxfldEffortMult sampleCf sampleT effMixedField
    effMixedField = rrEffortMixedField sampleCf effCf
    effCf = ipolyeff_cfRoundedRealEffort eff
    sampleCf = ipolycfg_sample_cf cfg
mixedMultInEffGeneric =
    error "aern-poly: IntPoly does not support inwards-rounded mixed multiplication" 


scaleTerms :: 
    (cf -> t -> cf) -> 
    t -> 
    IntPolyTerms var cf -> 
    IntPolyTerms var cf
scaleTerms (*|) factor terms =
    termsMapCoeffs (*| factor) terms

instance
    (EffortIndicator (IntPolyEffort (Interval e))) 
    =>
    ArithInOut.RoundedMixedMultiplyEffort (IntPoly var (Interval e)) (Interval e) 
    where
    type MixedMultEffortIndicator (IntPoly var (Interval e)) (Interval e) = 
        IntPolyEffort (Interval e)
    mixedMultDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg 

instance
    (poly ~ IntPoly var cf, cf ~ Interval e,
     Show cf,  HasConsistency cf,
     ArithInOut.RoundedMultiply poly) 
    =>
    ArithInOut.RoundedMixedMultiply (IntPoly var (Interval e)) (Interval e) 
    where
    mixedMultOutEff eff p@(IntPoly cfg _terms) a =
        -- TODO: use  multiplySingletonWithInterval from Numeric.AERN.RealArithmetic.Interval.MixedFieldOps
        ArithInOut.multOutEff eff p (IntPoly cfg $ mkConstTerms a vars)
        where
        vars = ipolycfg_vars cfg
    mixedMultInEff = 
        error "aern-poly: IntPoly does not support inwards-rounded mixed multiplication" 

instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e))
    =>
    ArithInOut.RoundedMixedMultiplyEffort 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    type MixedMultEffortIndicator 
            (IntPoly var (Interval e)) 
            (IntPoly var (Interval e)) = 
        IntPolyEffort (Interval e) 
    mixedMultDefaultEffort _ sample = 
        ArithInOut.addDefaultEffort sample

instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMultiply (IntPoly var (Interval e)))
    =>
    ArithInOut.RoundedMixedMultiply 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    mixedMultOutEff = ArithInOut.multOutEff
    mixedMultInEff = 
        error "aern-poly: IntPoly does not support inwards-rounded mixed multiplication" 

instance
    (RefOrd.IntervalLike cf, 
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedRingEffort (IntPoly var cf)
    where
    type RingOpsEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    ringOpsDefaultEffort (IntPoly cfg _) = 
        ipolycfg_effort cfg
    ringEffortAdd _ eff = eff  
    ringEffortMult _ eff = eff
    ringEffortPow _ eff = eff  

instance 
    (ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedRing (IntPoly var cf)


instance
    (EffortIndicator (IntPolyEffort cf), 
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRingEffort cf Integer) 
    => 
    ArithInOut.RoundedMixedRingEffort (IntPoly var cf) Integer
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Integer =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Integer,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing (IntPoly var cf) Integer

instance
    (EffortIndicator (IntPolyEffort cf), 
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedRingEffort (IntPoly var cf) Int
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Int =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Int,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing (IntPoly var cf) Int

instance
    (EffortIndicator (IntPolyEffort cf), 
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedRingEffort (IntPoly var cf) Rational
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Rational =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Rational,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing (IntPoly var cf) Rational

instance
    (EffortIndicator (IntPolyEffort cf), 
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedRingEffort (IntPoly var cf) Double
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Double =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Double,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing (IntPoly var cf) Double


instance
    (EffortIndicator (IntPolyEffort (Interval e)))
    => 
    ArithInOut.RoundedMixedRingEffort (IntPoly var (Interval e)) (Interval e)
    where
    type MixedRingOpsEffortIndicator (IntPoly var (Interval e)) (Interval e) =
        IntPolyEffort (Interval e)
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (cf ~ Interval e,
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf (Interval e),
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing (IntPoly var (Interval e)) (Interval e)

instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e))
    => 
    ArithInOut.RoundedMixedRingEffort 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    type MixedRingOpsEffortIndicator 
            (IntPoly var (Interval e))
            (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxringEffortAdd _ _ eff = eff
    mxringEffortMult _ _ eff = eff 

instance 
    (cf ~ Interval e,
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf (Interval e),
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedRing 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))

{-------------------------------}
{----  division by a scalar ----}
{-------------------------------}

instance
    (EffortIndicator (IntPolyEffort cf))
    => 
    ArithInOut.RoundedMixedDivideEffort (IntPoly var cf) Integer 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Integer = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg

instance
    (ArithInOut.RoundedMixedDivide cf Integer,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedDivide (IntPoly var cf) Integer 
    where
    mixedDivOutEff = mixedDivOutEffGeneric ArithInOut.rrEffortIntegerMixedField 0
    mixedDivInEff = mixedDivInEffGeneric ArithInOut.rrEffortIntegerMixedField 0
    
instance
    (EffortIndicator (IntPolyEffort cf))
    => 
    ArithInOut.RoundedMixedDivideEffort (IntPoly var cf) Int 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Int = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg

instance
    (ArithInOut.RoundedMixedDivide cf Int,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedDivide (IntPoly var cf) Int 
    where
    mixedDivOutEff = mixedDivOutEffGeneric ArithInOut.rrEffortIntMixedField 0
    mixedDivInEff = mixedDivInEffGeneric ArithInOut.rrEffortIntMixedField 0
    
instance
    (EffortIndicator (IntPolyEffort cf))
    => 
    ArithInOut.RoundedMixedDivideEffort (IntPoly var cf) Rational 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Rational = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg

instance
    (ArithInOut.RoundedMixedDivide cf Rational,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedDivide (IntPoly var cf) Rational 
    where
    mixedDivOutEff = mixedDivOutEffGeneric ArithInOut.rrEffortRationalMixedField 0
    mixedDivInEff = mixedDivInEffGeneric ArithInOut.rrEffortRationalMixedField 0
    
instance
    (EffortIndicator (IntPolyEffort cf))
    => 
    ArithInOut.RoundedMixedDivideEffort (IntPoly var cf) Double 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Double = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg

instance
    (ArithInOut.RoundedMixedDivide cf Double,  Ord var, 
     Show var, Show cf,
     ArithInOut.RoundedReal cf, 
     HasConsistency cf, 
     RefOrd.IntervalLike cf) 
    =>
    ArithInOut.RoundedMixedDivide (IntPoly var cf) Double 
    where
    mixedDivOutEff = mixedDivOutEffGeneric ArithInOut.rrEffortDoubleMixedField 0
    mixedDivInEff = mixedDivInEffGeneric ArithInOut.rrEffortDoubleMixedField 0

mixedDivOutEffGeneric, mixedDivInEffGeneric :: 
    (Ord var, Show cf, Show var, ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf, HasConsistency cf,
     ArithInOut.RoundedMixedField cf t) 
     =>
    (cf -> ArithInOut.RoundedRealEffortIndicator cf -> ArithInOut.MixedFieldOpsEffortIndicator cf t)
    -> t
    -> IntPolyEffort cf
    -> IntPoly var cf -> t -> IntPoly var cf
mixedDivOutEffGeneric rrEffortMixedField sampleT eff (IntPoly cfg terms) a = 
    IntPoly cfg $ scaleTerms (/|) a terms
    where
    (/|) = ArithInOut.mixedDivOutEff effMixedDiv
    effMixedDiv = ArithInOut.mxfldEffortDiv sampleCf sampleT effMixedField
    effMixedField = rrEffortMixedField sampleCf effCf
    effCf = ipolyeff_cfRoundedRealEffort eff
    sampleCf = ipolycfg_sample_cf cfg
mixedDivInEffGeneric =
    error "aern-poly: IntPoly does not support inwards-rounded mixed division" 
        
instance
    (EffortIndicator (IntPolyEffort (Interval e)))
    => 
    ArithInOut.RoundedMixedDivideEffort (IntPoly var (Interval e)) (Interval e) 
    where
    type MixedDivEffortIndicator (IntPoly var (Interval e)) (Interval e) = 
        IntPolyEffort (Interval e)
    mixedDivDefaultEffort (IntPoly cfg _) _c = 
        ipolycfg_effort cfg

instance
    (ArithInOut.RoundedDivide (Interval e),  Ord var, 
     ArithInOut.RoundedReal (Interval e), 
     HasConsistency (Interval e)) 
    =>
    ArithInOut.RoundedMixedDivide (IntPoly var (Interval e)) (Interval e) 
    where
    mixedDivOutEff eff (IntPoly cfg terms) a = 
        IntPoly cfg $ scaleTerms (/|) a terms
        where
        (/|) = ArithInOut.divOutEff effDiv
        effDiv = ArithInOut.fldEffortDiv sampleCf effField
        effField = ArithInOut.rrEffortField sampleCf effCf
        effCf = ipolyeff_cfRoundedRealEffort eff
        sampleCf = ipolycfg_sample_cf cfg
    mixedDivInEff =
        error "aern-poly: IntPoly does not support inwards-rounded mixed division" 

        
instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedFieldEffort (IntPoly var cf) Integer
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Integer =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Integer,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedField (IntPoly var cf) Integer
        
instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedFieldEffort (IntPoly var cf) Int
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Int =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Int,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedField (IntPoly var cf) Int

instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedFieldEffort (IntPoly var cf) Rational
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Rational =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Rational,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedField (IntPoly var cf) Rational
        
instance
    (EffortIndicator (IntPolyEffort cf),
     ArithInOut.RoundedReal cf) 
    => 
    ArithInOut.RoundedMixedFieldEffort (IntPoly var cf) Double
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Double =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Double,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedField (IntPoly var cf) Double

instance
    (cf ~ Interval e,
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedFieldEffort cf (Interval e)) 
    => 
    ArithInOut.RoundedMixedFieldEffort (IntPoly var (Interval e)) (Interval e)
    where
    type MixedFieldOpsEffortIndicator (IntPoly var (Interval e)) (Interval e) =
        IntPolyEffort (Interval e)
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sample = 
        ipolycfg_effort cfg
    mxfldEffortAdd _ _ eff = eff
    mxfldEffortMult _ _ eff = eff 
    mxfldEffortDiv _ _ eff = eff 

instance 
    (cf ~ Interval e,
     ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf (Interval e),
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithInOut.RoundedMixedField (IntPoly var (Interval e)) (Interval e)

