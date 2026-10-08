{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps
    Description :  rounded arithmetic instances for MPFR
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded arithmetic instances for MPFR.
    
    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps 
()
where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.MPFR.Utilities

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Effort

import Numeric.AERN.Misc.Debug

instance RoundedAddEffort MPFR where
    type AddEffortIndicator MPFR = ()
    addDefaultEffort _ = ()

instance RoundedAdd MPFR where
    addUpEff _ d1 d2 = 
        detectNaNUp ("addition " ++ show d1 ++ " +^ " ++ show d2 ) $
            liftRoundedToMPFR2 "+^" (+) d1 d2
    addDnEff _ d1 d2 =
        detectNaNDn ("addition " ++ show d1 ++ " +. " ++ show d2 ) $ 
            liftRoundedToMPFR2 "+." (\ a b -> -((-a) + (-b))) d1 d2
    
instance RoundedSubtr MPFR

instance RoundedAbsEffort MPFR where
    type AbsEffortIndicator MPFR = ()
    absDefaultEffort _ = ()

instance RoundedAbs MPFR where
    absDnEff _ = liftRoundedToMPFR1 abs
    absUpEff _ = liftRoundedToMPFR1 abs

instance RoundedMultiplyEffort MPFR where
    type MultEffortIndicator MPFR = () 
    multDefaultEffort _ = ()

instance RoundedMultiply MPFR where
    multUpEff _ d1 d2 = 
--        unsafePrintReturn
--        (
--            "MPFR multiplication UP with prec = " ++ show prec 
--            ++ " " ++ show d1 ++ " * " ++ show d2 ++ " = " 
--        ) $
        detectNaNUp ("multiplication " ++ show d1 ++ " *^ " ++ show d2 ) $ 
            liftRoundedToMPFR2 "*^" (*) d1 d2
    multDnEff _ d1 d2 = 
--        unsafePrintReturn
--        (
--            "MPFR multiplication DOWN with prec = " ++ show prec 
--            ++ " " ++ show d1 ++ " * " ++ show d2 ++ " = " 
--        ) $
        detectNaNDn ("multiplication " ++ show d1 ++ " *. " ++ show d2 ) $ 
            liftRoundedToMPFR2 "*." (\ a b -> -((-a) * b)) d1 d2


instance RoundedPowerNonnegToNonnegIntEffort MPFR where
    type PowerNonnegToNonnegIntEffortIndicator MPFR = ()
    powerNonnegToNonnegIntDefaultEffort _ = ()

instance RoundedPowerNonnegToNonnegInt MPFR where 
-- TODO: add to rounded an interface to MPFR powi 
    powerNonnegToNonnegIntUpEff _ =
        powerNonnegToNonnegIntUpEffFromMult ()
--        M.powi M.Up (M.getPrec x) x n 
    powerNonnegToNonnegIntDnEff _ = 
        powerNonnegToNonnegIntDnEffFromMult ()
--        M.powi M.Down (M.getPrec x) x n 

instance RoundedPowerToNonnegIntEffort MPFR where
    type PowerToNonnegIntEffortIndicator MPFR = ()
    powerToNonnegIntDefaultEffort _ = ()

instance RoundedPowerToNonnegInt MPFR where
-- TODO: add to rounded an interface to MPFR powi 
    powerToNonnegIntUpEff _ =
        powerToNonnegIntUpEffFromMult ((),(),())
--        M.powi M.Up (M.getPrec x) x n 
    powerToNonnegIntDnEff _ = 
        powerToNonnegIntDnEffFromMult ((),(),())
--        M.powi M.Down (M.getPrec x) x n 

instance RoundedDivideEffort MPFR where
    type DivEffortIndicator MPFR = ()
    divDefaultEffort _ = ()

instance RoundedDivide MPFR where
    divUpEff _ d1 d2 = 
        detectNaNUp ("division " ++ show d1 ++ " *^ " ++ show d2 ) $ 
            liftRoundedToMPFR2 "/^" (/) d1 d2
    divDnEff _ d1 d2 = 
        detectNaNDn ("division " ++ show d1 ++ " *. " ++ show d2 ) $ 
            liftRoundedToMPFR2 "/." (\ a b -> -((-a) / b)) d1 d2 

instance RoundedRingEffort MPFR
    where
    type RingOpsEffortIndicator MPFR = ()
    ringOpsDefaultEffort _ = ()
    ringEffortAdd _ _ = ()
    ringEffortMult _ _ = ()
    ringEffortPow _ _ = ()

instance RoundedRing MPFR

instance RoundedFieldEffort MPFR
    where
    type FieldOpsEffortIndicator MPFR = ()
    fieldOpsDefaultEffort _ = ()
    fldEffortAdd _ _ = ()
    fldEffortMult _ _ = ()
    fldEffortPow _ _ = ()
    fldEffortDiv _ _ = ()

instance RoundedField MPFR
    

    