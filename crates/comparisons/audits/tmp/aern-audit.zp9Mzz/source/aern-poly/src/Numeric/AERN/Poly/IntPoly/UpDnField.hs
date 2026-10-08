{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}

{-|
    Module      :  Numeric.AERN.Poly.IntPoly.UpDnField
    Description :  up/dn-rounded ops for polys as interval endpoints
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Up/dn-rounded polynomial field operations, making the polynomials usable as interval endpoints.
-}

module Numeric.AERN.Poly.IntPoly.UpDnField
    ()
where
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
--import Numeric.AERN.Poly.IntPoly.Reduction
import Numeric.AERN.Poly.IntPoly.Conversion ()
import Numeric.AERN.Poly.IntPoly.Addition ()
import Numeric.AERN.Poly.IntPoly.Multiplication ()
import Numeric.AERN.Poly.IntPoly.Division ()

--import Numeric.AERN.RmToRn

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import Numeric.AERN.RealArithmetic.NumericOrderRounding
    (AddEffortIndicator, 
     MultEffortIndicator,
     PowerNonnegToNonnegIntEffortIndicator, 
     PowerToNonnegIntEffortIndicator,
     RingOpsEffortIndicator, 
     MixedAddEffortIndicator,
     MixedMultEffortIndicator,
     MixedRingOpsEffortIndicator,
     MixedDivEffortIndicator,
     MixedFieldOpsEffortIndicator
     )
--import Numeric.AERN.RealArithmetic.NumericOrderRounding.OpsImplicitEffort

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

--import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures
--import Numeric.AERN.RealArithmetic.Auxiliary

--import Numeric.AERN.RealArithmetic.Interval

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort
import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.Effort (EffortIndicator)
import Numeric.AERN.Basics.Consistency

--import Numeric.AERN.Misc.Debug

--import qualified Data.IntMap as IntMap

import Test.QuickCheck (Arbitrary)

{----- addition up/dn via out -----}    

instance
    (ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedAddEffort (IntPoly var cf) 
    where
    type AddEffortIndicator (IntPoly var cf) = 
        IntPolyEffort cf
    addDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedReal cf,
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Arbitrary cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedAdd (IntPoly var cf) 
    where
    addUpEff eff p1 p2 =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.addOutEff eff p1 p2
    addDnEff eff p1 p2 =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.addOutEff eff p1 p2

instance
    (ArithInOut.RoundedReal cf,
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Arbitrary cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedSubtr (IntPoly var cf) 
    
  
{----- multiplication up/dn via out -----}    

instance
    (ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMultiplyEffort (IntPoly var cf) 
    where
    type MultEffortIndicator (IntPoly var cf) = 
        IntPolyEffort cf
    multDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedReal cf,
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Arbitrary cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMultiply (IntPoly var cf) 
    where
    multUpEff eff p1 p2 =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.multOutEff eff p1 p2
    multDnEff eff p1 p2 =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.multOutEff eff p1 p2
  
{----- integer power (both nonneg and general versions) up/dn via out -----}      

instance
    (Ord var,
     ArithInOut.RoundedReal cf, 
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedPowerNonnegToNonnegIntEffort (IntPoly var cf)
    where
    type PowerNonnegToNonnegIntEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    powerNonnegToNonnegIntDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedReal cf, 
     NumOrd.PartialComparison (Imprecision cf),
     RefOrd.IntervalLike cf, 
     HasAntiConsistency cf,
--     NumOrd.PartialComparison (Imprecision cf), 
--     Show (Imprecision cf),
     Show var, Show cf, Ord var) 
    =>
    ArithUpDn.RoundedPowerNonnegToNonnegInt (IntPoly var cf) 
    where
    powerNonnegToNonnegIntUpEff eff p n =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.powerToNonnegIntOutEff eff p n   
    powerNonnegToNonnegIntDnEff eff p n =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.powerToNonnegIntOutEff eff p n   
    
instance
    (Ord var,
     ArithInOut.RoundedReal cf, 
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedPowerToNonnegIntEffort (IntPoly var cf)
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
--     NumOrd.PartialComparison (Imprecision cf), 
--     Show (Imprecision cf),
     Show var, Show cf, Ord var) 
    =>
    ArithUpDn.RoundedPowerToNonnegInt (IntPoly var cf) 
    where
    powerToNonnegIntUpEff eff p n =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.powerToNonnegIntOutEff eff p n   
    powerToNonnegIntDnEff eff p n =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.powerToNonnegIntOutEff eff p n   
  
instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedRingEffort (IntPoly var cf)
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
    ArithUpDn.RoundedRing (IntPoly var cf)


{----- up/dn division -----}    

instance
   (Ord var, Show var, Show cf, Show (Imprecision cf),
    HasAntiConsistency cf, ArithInOut.RoundedReal cf,
    RefOrd.IntervalLike cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    => 
    ArithUpDn.RoundedDivideEffort (IntPoly var cf) 
    where
    type DivEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    divDefaultEffort (IntPoly cfg _) = 
        ipolycfg_effort cfg 

instance
   (cf ~ (Interval e),
    Ord var, Show var, Show cf, 
    Show (Imprecision cf),
    ArithInOut.RoundedReal cf,
    NumOrd.PartialComparison (Imprecision cf),
    HasAntiConsistency cf, 
    RefOrd.IntervalLike cf,
    Arbitrary cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    =>
    ArithUpDn.RoundedDivide (IntPoly var cf) 
    where
    divUpEff eff p1 p2 =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.divOutEff eff p1 p2
    divDnEff eff p1 p2 =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.divOutEff eff p1 p2
  

instance
   (Ord var, Show var, Show cf, Show (Imprecision cf),
    HasAntiConsistency cf, ArithInOut.RoundedReal cf,
    RefOrd.IntervalLike cf,
    ArithInOut.RoundedMixedField (IntPoly var cf) cf) 
    => 
    ArithUpDn.RoundedFieldEffort (IntPoly var cf)
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
    ArithUpDn.RoundedField (IntPoly var cf)

{----- mixed addition up/dn via out -----}    

instance
    (EffortIndicator (IntPolyEffort (Interval e))) 
    =>
    ArithUpDn.RoundedMixedAddEffort 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    type MixedAddEffortIndicator 
            (IntPoly var (Interval e)) 
            (IntPoly var (Interval e)) = 
        IntPolyEffort (Interval e)
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (EffortIndicator (IntPolyEffort (Interval e)),
     ArithInOut.RoundedMixedAddEffort (Interval e) (Interval e)) 
    =>
    ArithUpDn.RoundedMixedAddEffort (IntPoly var (Interval e)) (Interval e) 
    where
    type MixedAddEffortIndicator (IntPoly var (Interval e)) (Interval e) = 
        IntPolyEffort (Interval e)
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedAddEffort cf Int,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedAddEffort (IntPoly var cf) Int 
    where
    type MixedAddEffortIndicator (IntPoly var cf) Int = 
        IntPolyEffort cf
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedAddEffort cf Integer,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedAddEffort (IntPoly var cf) Integer
    where
    type MixedAddEffortIndicator (IntPoly var cf) Integer = 
        IntPolyEffort cf
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedAddEffort cf Rational,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedAddEffort (IntPoly var cf) Rational
    where
    type MixedAddEffortIndicator (IntPoly var cf) Rational = 
        IntPolyEffort cf
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 


instance
    (ArithInOut.RoundedMixedAddEffort cf Double,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedAddEffort (IntPoly var cf) Double
    where
    type MixedAddEffortIndicator (IntPoly var cf) Double = 
        IntPolyEffort cf
    mixedAddDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedAdd (IntPoly var (Interval e)))
    =>
    ArithUpDn.RoundedMixedAdd 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedAdd (Interval e) (Interval e),
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),  
     HasAntiConsistency (Interval e),
     Ord var, 
     Show var, Show (Interval e)) 
    =>
    ArithUpDn.RoundedMixedAdd (IntPoly var (Interval e)) (Interval e) 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedAdd cf Int,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedAdd (IntPoly var cf) Int 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedAdd cf Integer,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedAdd (IntPoly var cf) Integer 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedAdd cf Rational,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedAdd (IntPoly var cf) Rational 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedAdd cf Double,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedAdd (IntPoly var cf) Double 
    where
    mixedAddUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other
    mixedAddDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedAddOutEff eff p1 other



{----- mixed multiplication up/dn via out -----}    

instance
    (EffortIndicator (IntPolyEffort (Interval e)))
    => 
    ArithUpDn.RoundedMixedMultiplyEffort 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    type MixedMultEffortIndicator 
            (IntPoly var (Interval e)) 
            (IntPoly var (Interval e)) = 
        IntPolyEffort (Interval e)
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiplyEffort (Interval e) (Interval e), 
     EffortIndicator (IntPolyEffort (Interval e)))
    => 
    ArithUpDn.RoundedMixedMultiplyEffort (IntPoly var (Interval e)) (Interval e) 
    where
    type MixedMultEffortIndicator (IntPoly var (Interval e)) (Interval e) = 
        IntPolyEffort (Interval e)
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiplyEffort cf Int,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedMultiplyEffort (IntPoly var cf) Int 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Int = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiplyEffort cf Integer,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedMultiplyEffort (IntPoly var cf) Integer 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Integer = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiplyEffort cf Double,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedMultiplyEffort (IntPoly var cf) Double 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Double = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedMultiplyEffort cf Rational,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedMultiplyEffort (IntPoly var cf) Rational 
    where
    type MixedMultEffortIndicator (IntPoly var cf) Rational = 
        IntPolyEffort cf
    mixedMultDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMultiply (IntPoly var (Interval e)))
    =>
    ArithUpDn.RoundedMixedMultiply 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedMultiply (Interval e) (Interval e),
     ArithInOut.RoundedReal (Interval e),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     RefOrd.IntervalLike (Interval e),  
     HasAntiConsistency (Interval e),
     Arbitrary (Interval e),
     Ord var, 
     Show var, Show (Interval e)) 
    =>
    ArithUpDn.RoundedMixedMultiply (IntPoly var (Interval e)) (Interval e) 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedMultiply cf Int,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedMultiply (IntPoly var cf) Int 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedMultiply cf Integer,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedMultiply (IntPoly var cf) Integer 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedMultiply cf Double,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedMultiply (IntPoly var cf) Double 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    
instance
    (ArithInOut.RoundedMixedMultiply cf Rational,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedMultiply (IntPoly var cf) Rational 
    where
    mixedMultUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other
    mixedMultDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedMultOutEff eff p1 other


instance
    (EffortIndicator (IntPolyEffort (Interval e)), 
     ArithInOut.RoundedReal (Interval e))
    => 
    ArithUpDn.RoundedMixedRingEffort 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    type MixedRingOpsEffortIndicator 
            (IntPoly var (Interval e))
            (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

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
    ArithUpDn.RoundedMixedRing 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))

instance
    (Ord var,
     ArithInOut.RoundedReal (Interval e),
     RefOrd.IntervalLike (Interval e),
     ArithInOut.RoundedMixedRingEffort (Interval e) (Interval e)) 
    => 
    ArithUpDn.RoundedMixedRingEffort (IntPoly var (Interval e)) (Interval e)
    where
    type MixedRingOpsEffortIndicator (IntPoly var (Interval e)) (Interval e) =
        IntPolyEffort (Interval e)
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMixedRing (Interval e) (Interval e),
     HasAntiConsistency (Interval e),
     RefOrd.IntervalLike (Interval e),
     Arbitrary (Interval e),
     Show var, Ord var, Show (Interval e),
     NumOrd.PartialComparison (Imprecision (Interval e)), Show (Imprecision (Interval e)))
    =>
    ArithUpDn.RoundedMixedRing (IntPoly var (Interval e)) (Interval e)

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedRingEffort cf Int) 
    => 
    ArithUpDn.RoundedMixedRingEffort (IntPoly var cf) Int
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Int =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Int,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedRing (IntPoly var cf) Int

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedRingEffort cf Integer) 
    => 
    ArithUpDn.RoundedMixedRingEffort (IntPoly var cf) Integer
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Integer =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Integer,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedRing (IntPoly var cf) Integer

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedRingEffort cf Rational) 
    => 
    ArithUpDn.RoundedMixedRingEffort (IntPoly var cf) Rational
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Rational =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Rational,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedRing (IntPoly var cf) Rational
        
        

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedRingEffort cf Double) 
    => 
    ArithUpDn.RoundedMixedRingEffort (IntPoly var cf) Double
    where
    type MixedRingOpsEffortIndicator (IntPoly var cf) Double =
        IntPolyEffort cf
    mixedRingOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxringEffortAdd _sampleP _sampleI eff = eff 
    mxringEffortMult _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedRing cf Double,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedRing (IntPoly var cf) Double

{----- mixed division up/dn via out -----}    

instance
    (EffortIndicator (IntPolyEffort (Interval e)),
     ArithInOut.RoundedMixedDivideEffort (Interval e) (Interval e)) 
    => 
    ArithUpDn.RoundedMixedDivideEffort 
        (IntPoly var (Interval e)) 
        (IntPoly var (Interval e)) 
    where
    type MixedDivEffortIndicator 
            (IntPoly var (Interval e)) 
            (IntPoly var (Interval e)) = 
        IntPolyEffort (Interval e)
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
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
    ArithUpDn.RoundedMixedDivide 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other

instance
    (EffortIndicator (IntPolyEffort (Interval e)),
     ArithInOut.RoundedMixedDivideEffort (Interval e) (Interval e)) 
    => 
    ArithUpDn.RoundedMixedDivideEffort (IntPoly var (Interval e)) (Interval e) 
    where
    type MixedDivEffortIndicator (IntPoly var (Interval e)) (Interval e) = 
        IntPolyEffort (Interval e)
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedDivide (Interval e) (Interval e),
     ArithInOut.RoundedReal (Interval e),
     HasAntiConsistency (Interval e),
     Ord var, 
     Show var, Show (Interval e)) 
    =>
    ArithUpDn.RoundedMixedDivide (IntPoly var (Interval e)) (Interval e) 
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other

instance
    (ArithInOut.RoundedMixedDivideEffort cf Int,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedDivideEffort (IntPoly var cf) Int 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Int = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedDivide cf Int,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedDivide (IntPoly var cf) Int 
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other

instance
    (ArithInOut.RoundedMixedDivideEffort cf Integer,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedDivideEffort (IntPoly var cf) Integer 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Integer = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedDivide cf Integer,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedDivide (IntPoly var cf) Integer 
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other

instance
    (ArithInOut.RoundedMixedDivideEffort cf Rational,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedDivideEffort (IntPoly var cf) Rational 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Rational = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedDivide cf Rational,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedDivide (IntPoly var cf) Rational 
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other

instance
    (ArithInOut.RoundedMixedDivideEffort cf Double,
     EffortIndicator (IntPolyEffort cf),
     RefOrd.IntervalLike cf) 
    => 
    ArithUpDn.RoundedMixedDivideEffort (IntPoly var cf) Double 
    where
    type MixedDivEffortIndicator (IntPoly var cf) Double = 
        IntPolyEffort cf
    mixedDivDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 

instance
    (ArithInOut.RoundedMixedDivide cf Double,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,  
     HasAntiConsistency cf,
     Ord var, 
     Show var, Show cf) 
    =>
    ArithUpDn.RoundedMixedDivide (IntPoly var cf) Double 
    where
    mixedDivUpEff eff p1 other =
        snd $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other
    mixedDivDnEff eff p1 other =
        fst $ 
            polyGetEndpointsOutEff (ipolyeff_cfGetEndpointsEffort eff) $ 
                ArithInOut.mixedDivOutEff eff p1 other


instance
    (Ord var,
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMixedFieldEffort (Interval e) (Interval e)) 
    => 
    ArithUpDn.RoundedMixedFieldEffort (IntPoly var (Interval e)) (Interval e)
    where
    type MixedFieldOpsEffortIndicator (IntPoly var (Interval e)) (Interval e) =
        IntPolyEffort (Interval e)
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

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
    ArithUpDn.RoundedMixedField 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))

instance
    (Ord var,
     ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMixedFieldEffort (Interval e) (Interval e)) 
    => 
    ArithUpDn.RoundedMixedFieldEffort 
        (IntPoly var (Interval e))
        (IntPoly var (Interval e))
    where
    type MixedFieldOpsEffortIndicator 
            (IntPoly var (Interval e))
            (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     HasAntiConsistency (Interval e),
     Arbitrary (Interval e),
     Show var, Ord var, Show (Interval e),
     NumOrd.PartialComparison (Imprecision (Interval e)), Show (Imprecision (Interval e)))
    =>
    ArithUpDn.RoundedMixedField (IntPoly var (Interval e)) (Interval e)

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedFieldEffort cf Int) 
    => 
    ArithUpDn.RoundedMixedFieldEffort (IntPoly var cf) Int
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Int =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Int,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedField (IntPoly var cf) Int

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedFieldEffort cf Integer) 
    => 
    ArithUpDn.RoundedMixedFieldEffort (IntPoly var cf) Integer
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Integer =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Integer,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedField (IntPoly var cf) Integer

instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedFieldEffort cf Rational) 
    => 
    ArithUpDn.RoundedMixedFieldEffort (IntPoly var cf) Rational
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Rational =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Rational,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedField (IntPoly var cf) Rational
        
instance
    (Ord var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedMixedFieldEffort cf Double) 
    => 
    ArithUpDn.RoundedMixedFieldEffort (IntPoly var cf) Double
    where
    type MixedFieldOpsEffortIndicator (IntPoly var cf) Double =
        IntPolyEffort cf
    mixedFieldOpsDefaultEffort (IntPoly cfg _) _sampleOther = 
        ipolycfg_effort cfg 
    mxfldEffortAdd _sampleP _sampleI eff = eff 
    mxfldEffortMult _sampleP _sampleI eff = eff 
    mxfldEffortDiv _sampleP _sampleI eff = eff 

instance
    (ArithInOut.RoundedReal cf,
     ArithInOut.RoundedMixedField cf Double,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    ArithUpDn.RoundedMixedField (IntPoly var cf) Double
        
        
