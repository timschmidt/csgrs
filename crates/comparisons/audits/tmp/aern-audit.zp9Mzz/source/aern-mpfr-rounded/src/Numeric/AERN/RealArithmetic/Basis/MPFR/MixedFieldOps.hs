{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.MixedFieldOps
    Description :  rounded basic arithmetic operations mixing MPFR and another type
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations mixing MPFR and another type.
    
    This module is hidden and reexported via its parent MPFR. 
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.MixedFieldOps where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.Conversion
import Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.Utilities

import Numeric.AERN.RealArithmetic.NumericOrderRounding

instance RoundedMixedAddEffort MPFR Int
    where
    type MixedAddEffortIndicator MPFR Int = ()
    mixedAddDefaultEffort _ _ = ()

-- TODO: will need to extend rounded to link to MPFR addi etc ...
instance RoundedMixedAdd MPFR Int
    where
    mixedAddUpEff _ d i = mixedAddUpEffByConversion ((), ()) d i
    mixedAddDnEff _ d i = mixedAddDnEffByConversion ((), ()) d i
--    mixedAddUpEff _ d n = M.addi M.Up (getPrecision d) d n
--    mixedAddDnEff _ d n = M.addi M.Down (getPrecision d) d n

instance RoundedMixedMultiplyEffort MPFR Int
    where
    type MixedMultEffortIndicator MPFR Int = ()
    mixedMultDefaultEffort _ _ = ()

instance RoundedMixedMultiply MPFR Int
    where
    mixedMultUpEff _ d i = mixedMultUpEffByConversion ((), (), ()) d i
    mixedMultDnEff _ d i = mixedMultDnEffByConversion ((), (), ()) d i
--    mixedMultUpEff _ d i = 
--        -- infty * 0 is NaN
--        detectNaNUp ("mixed multiplication " ++ show d ++ " *^| " ++ show i ) $ 
--                M.muli M.Up (getPrecision d) d i
--    mixedMultDnEff _ d i = 
--        detectNaNDn ("mixed multiplication " ++ show d ++ " *.| " ++ show i ) $ 
--                M.muli M.Down (getPrecision d) d i

instance RoundedMixedDivideEffort MPFR Int
    where
    type MixedDivEffortIndicator MPFR Int = ()
    mixedDivDefaultEffort _ _ = ()

instance RoundedMixedDivide MPFR Int
    where
    mixedDivUpEff _ d i = mixedDivUpEffByConversion ((), (), ((),())) d i
    mixedDivDnEff _ d i = mixedDivDnEffByConversion ((), (), ((),())) d i
--    mixedDivUpEff _ d i = 
--        -- 0 / 0 is NaN
--        detectNaNUp ("mixed division " ++ show d ++ " /^| " ++ show i ) $ 
--                M.divi M.Up (getPrecision d) d i
--    mixedDivDnEff _ d i = 
--        detectNaNDn ("mixed division " ++ show d ++ " /.| " ++ show i ) $ 
--                M.divi M.Down (getPrecision d) d i

instance RoundedMixedRingEffort MPFR Int
    where
    type MixedRingOpsEffortIndicator MPFR Int = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ()
    mxringEffortMult _ _ _ = ()

instance RoundedMixedRing MPFR Int

instance RoundedMixedFieldEffort MPFR Int
    where
    type MixedFieldOpsEffortIndicator MPFR Int = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ()
    mxfldEffortMult _ _ _ = ()
    mxfldEffortDiv _ _ _ = ()

instance RoundedMixedField MPFR Int

instance RoundedMixedAddEffort MPFR Integer
    where
    type MixedAddEffortIndicator MPFR Integer = ()
    mixedAddDefaultEffort _ _ = () 

instance RoundedMixedAdd MPFR Integer
    where
    mixedAddUpEff _ d i = mixedAddUpEffByConversion ((), ()) d i
    mixedAddDnEff _ d i = mixedAddDnEffByConversion ((), ()) d i

instance RoundedMixedMultiplyEffort MPFR Integer
    where
    type MixedMultEffortIndicator MPFR Integer = ()
    mixedMultDefaultEffort _ _ = () 

instance RoundedMixedMultiply MPFR Integer
    where
    mixedMultUpEff _ d i = mixedMultUpEffByConversion ((), (), ()) d i
    mixedMultDnEff _ d i = mixedMultDnEffByConversion ((), (), ()) d i

instance RoundedMixedDivideEffort MPFR Integer
    where
    type MixedDivEffortIndicator MPFR Integer = ()
    mixedDivDefaultEffort _ _  = () 

instance RoundedMixedDivide MPFR Integer
    where
    mixedDivUpEff _ d i = mixedDivUpEffByConversion ((), (), ((),())) d i
    mixedDivDnEff _ d i = mixedDivDnEffByConversion ((), (), ((),())) d i

instance RoundedMixedRingEffort MPFR Integer
    where
    type MixedRingOpsEffortIndicator MPFR Integer = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ eff = ()
    mxringEffortMult _ _ eff = ()

instance RoundedMixedRing MPFR Integer

instance RoundedMixedFieldEffort MPFR Integer
    where
    type MixedFieldOpsEffortIndicator MPFR Integer = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ eff = ()
    mxfldEffortMult _ _ eff = ()
    mxfldEffortDiv _ _ eff = ()

instance RoundedMixedField MPFR Integer



instance RoundedMixedAddEffort MPFR Rational
    where
    type MixedAddEffortIndicator MPFR Rational = ()
    mixedAddDefaultEffort _ _ = () 

instance RoundedMixedAdd MPFR Rational
    where
    mixedAddUpEff _ d r = mixedAddUpEffByConversion ((), ()) d r
    mixedAddDnEff _ d r = mixedAddDnEffByConversion ((), ()) d r

instance RoundedMixedMultiplyEffort MPFR Rational
    where
    type MixedMultEffortIndicator MPFR Rational = ()
    mixedMultDefaultEffort _ _  = () 


instance RoundedMixedMultiply MPFR Rational
    where
    mixedMultUpEff _ d r = mixedMultUpEffByConversion ((), (), ()) d r
    mixedMultDnEff _ d r = mixedMultDnEffByConversion ((), (), ()) d r

instance RoundedMixedDivideEffort MPFR Rational
    where
    type MixedDivEffortIndicator MPFR Rational = ()
    mixedDivDefaultEffort _ _ = () 


instance RoundedMixedDivide MPFR Rational
    where
    mixedDivUpEff _ d r = mixedDivUpEffByConversion ((), (), ((),())) d r
    mixedDivDnEff _ d r = mixedDivDnEffByConversion ((), (), ((),())) d r

instance RoundedMixedRingEffort MPFR Rational
    where
    type MixedRingOpsEffortIndicator MPFR Rational = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd sample _ eff = ()
    mxringEffortMult sample _ eff = ()

instance RoundedMixedRing MPFR Rational

instance RoundedMixedFieldEffort MPFR Rational
    where
    type MixedFieldOpsEffortIndicator MPFR Rational = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd sample _ eff = ()
    mxfldEffortMult sample _ eff = ()
    mxfldEffortDiv sample _ eff = ()

instance RoundedMixedField MPFR Rational



instance RoundedMixedAddEffort MPFR Double
    where
    type MixedAddEffortIndicator MPFR Double = ()
    mixedAddDefaultEffort _ _ = () 

instance RoundedMixedAdd MPFR Double
    where
    mixedAddUpEff _ d r = mixedAddUpEffByConversion ((), ()) d r
    mixedAddDnEff _ d r = mixedAddDnEffByConversion ((), ()) d r
--    mixedAddUpEff _ d i = 
--        -- infty - infty is NaN
--        detectNaNUp ("mixed addition " ++ show d ++ " +^| " ++ show i ) $ 
--        M.addd M.Up (getPrecision d) d i
--    mixedAddDnEff _ d i = 
--        detectNaNUp ("mixed addition " ++ show d ++ " +.| " ++ show i ) $ 
--        M.addd M.Down (getPrecision d) d i

instance RoundedMixedMultiplyEffort MPFR Double
    where
    type MixedMultEffortIndicator MPFR Double = ()
    mixedMultDefaultEffort _ _ = ()

instance RoundedMixedMultiply MPFR Double
    where
    mixedMultUpEff _ d r = mixedMultUpEffByConversion ((), (), ()) d r
    mixedMultDnEff _ d r = mixedMultDnEffByConversion ((), (), ()) d r
--    mixedMultUpEff _ d i = 
--        -- infty * 0 is NaN
--        detectNaNUp ("mixed multiplication " ++ show d ++ " *^| " ++ show i ) $ 
--                M.muld M.Up (getPrecision d) d i
--    mixedMultDnEff _ d i = 
--        detectNaNDn ("mixed multiplication " ++ show d ++ " *.| " ++ show i ) $ 
--                M.muld M.Down (getPrecision d) d i

instance RoundedMixedDivideEffort MPFR Double
    where
    type MixedDivEffortIndicator MPFR Double = ()
    mixedDivDefaultEffort _ _ = ()

instance RoundedMixedDivide MPFR Double
    where
    mixedDivUpEff _ d i = mixedDivUpEffByConversion ((), (), ((),())) d i
    mixedDivDnEff _ d i = mixedDivDnEffByConversion ((), (), ((),())) d i
--    mixedDivUpEff _ d i = 
--        -- 0 / 0 is NaN
--        detectNaNUp ("mixed division " ++ show d ++ " /^| " ++ show i ) $ 
--                M.divd M.Up (getPrecision d) d i
--    mixedDivDnEff _ d i = 
--        detectNaNDn ("mixed division " ++ show d ++ " /.| " ++ show i ) $ 
--                M.divd M.Down (getPrecision d) d i

instance RoundedMixedRingEffort MPFR Double
    where
    type MixedRingOpsEffortIndicator MPFR Double = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ()
    mxringEffortMult _ _ _ = ()

instance RoundedMixedRing MPFR Double

instance RoundedMixedFieldEffort MPFR Double
    where
    type MixedFieldOpsEffortIndicator MPFR Double = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ()
    mxfldEffortMult _ _ _ = ()
    mxfldEffortDiv _ _ _ = ()

instance RoundedMixedField MPFR Double


instance RoundedMixedAddEffort MPFR MPFR
    where
    type MixedAddEffortIndicator MPFR MPFR = ()
    mixedAddDefaultEffort _ _ = ()
instance RoundedMixedAdd MPFR MPFR
    where
    mixedAddUpEff = addUpEff
    mixedAddDnEff = addDnEff

instance RoundedMixedMultiplyEffort MPFR MPFR
    where
    type MixedMultEffortIndicator MPFR MPFR = ()
    mixedMultDefaultEffort _ _ = () 
instance RoundedMixedMultiply MPFR MPFR
    where
    mixedMultUpEff = multUpEff
    mixedMultDnEff = multDnEff

instance RoundedMixedDivideEffort MPFR MPFR
    where
    type MixedDivEffortIndicator MPFR MPFR = ()
    mixedDivDefaultEffort _ _ = () 
instance RoundedMixedDivide MPFR MPFR
    where
    mixedDivUpEff = divUpEff
    mixedDivDnEff = divDnEff

instance RoundedMixedRingEffort MPFR MPFR
    where
    type MixedRingOpsEffortIndicator MPFR MPFR = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ()
    mxringEffortMult _ _ _ = ()

instance RoundedMixedRing MPFR MPFR

instance RoundedMixedFieldEffort MPFR MPFR
    where
    type MixedFieldOpsEffortIndicator MPFR MPFR = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ()
    mxfldEffortMult _ _ _ = ()
    mxfldEffortDiv _ _ _ = ()

instance RoundedMixedField MPFR MPFR
