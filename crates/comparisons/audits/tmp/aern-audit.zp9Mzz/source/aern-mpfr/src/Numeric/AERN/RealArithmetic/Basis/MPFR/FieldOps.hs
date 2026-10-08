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

import Numeric.AERN.RealArithmetic.Basis.MPFR.Effort
import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.MPFR.Utilities

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Effort

import Numeric.AERN.Misc.Debug

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up
import qualified Data.Number.MPFR.Mutable as MM

instance RoundedAddEffort MPFR where
    type AddEffortIndicator MPFR = ()
    addDefaultEffort _ = ()

instance RoundedAdd MPFR where
    addUpEff _ d1 d2 = 
        detectNaNUp ("addition " ++ show d1 ++ " +^ " ++ show d2 ) $ 
            M.add M.Up (getPrec2 d1 d2) d1 d2
    addDnEff _ d1 d2 =
        detectNaNDn ("addition " ++ show d1 ++ " +. " ++ show d2 ) $ 
            M.add M.Down (getPrec2 d1 d2) d1 d2
    
getPrec2 :: MPFR -> MPFR -> M.Precision
getPrec2 d1 d2 =
    (M.getPrec d1) `max` (M.getPrec d2)
    
instance RoundedSubtr MPFR

instance RoundedAbsEffort MPFR where
    type AbsEffortIndicator MPFR = ()
    absDefaultEffort _ = ()

instance RoundedAbs MPFR where
    absDnEff _ = abs
    absUpEff _ = abs

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
            M.mul M.Up (getPrec2 d1 d2) d1 d2
    multDnEff _ d1 d2 = 
--        unsafePrintReturn
--        (
--            "MPFR multiplication DOWN with prec = " ++ show prec 
--            ++ " " ++ show d1 ++ " * " ++ show d2 ++ " = " 
--        ) $
        detectNaNDn ("multiplication " ++ show d1 ++ " *. " ++ show d2 ) $ 
            M.mul M.Down (getPrec2 d1 d2) d1 d2


instance RoundedPowerNonnegToNonnegIntEffort MPFR where
    type PowerNonnegToNonnegIntEffortIndicator MPFR = ()
    powerNonnegToNonnegIntDefaultEffort _ = ()

instance RoundedPowerNonnegToNonnegInt MPFR where
    powerNonnegToNonnegIntUpEff _ x n =
        M.powi M.Up (M.getPrec x) x n 
    powerNonnegToNonnegIntDnEff _ x n = 
        M.powi M.Down (M.getPrec x) x n 

instance RoundedPowerToNonnegIntEffort MPFR where
    type PowerToNonnegIntEffortIndicator MPFR = ()
    powerToNonnegIntDefaultEffort _ = ()

instance RoundedPowerToNonnegInt MPFR where
    powerToNonnegIntUpEff _ x n =
        M.powi M.Up (M.getPrec x) x n 
    powerToNonnegIntDnEff _ x n = 
        M.powi M.Down (M.getPrec x) x n 

instance RoundedDivideEffort MPFR where
    type DivEffortIndicator MPFR = ()
    divDefaultEffort _ = ()

instance RoundedDivide MPFR where
    divUpEff _ d1 d2 = 
        detectNaNUp ("division " ++ show d1 ++ " *^ " ++ show d2 ) $ 
            M.div M.Up (getPrec2 d1 d2) d1 d2
    divDnEff _ d1 d2 = 
        detectNaNDn ("division " ++ show d1 ++ " *. " ++ show d2 ) $ 
            M.div M.Down (getPrec2 d1 d2) d1 d2

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
    

    