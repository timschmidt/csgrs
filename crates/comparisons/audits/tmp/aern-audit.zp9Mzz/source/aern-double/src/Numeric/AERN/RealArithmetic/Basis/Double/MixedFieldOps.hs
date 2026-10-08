{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.MixedFieldOps
    Description :  rounded basic arithmetic operations mixing Double and another type
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations mixing Double and another type.
    
    This module is hidden and reexported via its parent Double. 
-}

module Numeric.AERN.RealArithmetic.Basis.Double.MixedFieldOps where

import Numeric.AERN.RealArithmetic.Basis.Double.Conversion
import Numeric.AERN.RealArithmetic.Basis.Double.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.Double.FieldOps

import Numeric.AERN.RealArithmetic.NumericOrderRounding

instance RoundedMixedAddEffort Double Int
    where
    type MixedAddEffortIndicator Double Int =
        MixedAddEffortIndicatorByConversion Double Int 
    mixedAddDefaultEffort = mixedAddDefaultEffortByConversion 
instance RoundedMixedAdd Double Int
    where
    mixedAddUpEff = mixedAddUpEffByConversion
    mixedAddDnEff = mixedAddDnEffByConversion

instance RoundedMixedMultiplyEffort Double Int
    where
    type MixedMultEffortIndicator Double Int =
        MixedMultEffortIndicatorByConversion Double Int 
    mixedMultDefaultEffort = mixedMultDefaultEffortByConversion 
instance RoundedMixedMultiply Double Int
    where
    mixedMultUpEff = mixedMultUpEffByConversion
    mixedMultDnEff = mixedMultDnEffByConversion

instance RoundedMixedDivideEffort Double Int
    where
    type MixedDivEffortIndicator Double Int =
        MixedDivEffortIndicatorByConversion Double Int 
    mixedDivDefaultEffort = mixedDivDefaultEffortByConversion 
instance RoundedMixedDivide Double Int
    where
    mixedDivUpEff = mixedDivUpEffByConversion
    mixedDivDnEff = mixedDivDnEffByConversion

instance RoundedMixedRingEffort Double Int
    where
    type MixedRingOpsEffortIndicator Double Int = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ((),())
    mxringEffortMult _ _ _ = ((),(),())

instance RoundedMixedRing Double Int

instance RoundedMixedFieldEffort Double Int
    where
    type MixedFieldOpsEffortIndicator Double Int = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ((),())
    mxfldEffortMult _ _ _ = ((),(),())
    mxfldEffortDiv _ _ _ = ((),(),((),()))

instance RoundedMixedField Double Int



instance RoundedMixedAddEffort Double Integer
    where
    type MixedAddEffortIndicator Double Integer =
        MixedAddEffortIndicatorByConversion Double Integer 
    mixedAddDefaultEffort = mixedAddDefaultEffortByConversion 
instance RoundedMixedAdd Double Integer
    where
    mixedAddUpEff = mixedAddUpEffByConversion
    mixedAddDnEff = mixedAddDnEffByConversion

instance RoundedMixedMultiplyEffort Double Integer
    where
    type MixedMultEffortIndicator Double Integer =
        MixedMultEffortIndicatorByConversion Double Integer 
    mixedMultDefaultEffort = mixedMultDefaultEffortByConversion 
instance RoundedMixedMultiply Double Integer
    where
    mixedMultUpEff = mixedMultUpEffByConversion
    mixedMultDnEff = mixedMultDnEffByConversion

instance RoundedMixedDivideEffort Double Integer
    where
    type MixedDivEffortIndicator Double Integer =
        MixedDivEffortIndicatorByConversion Double Integer 
    mixedDivDefaultEffort = mixedDivDefaultEffortByConversion 
instance RoundedMixedDivide Double Integer
    where
    mixedDivUpEff = mixedDivUpEffByConversion
    mixedDivDnEff = mixedDivDnEffByConversion

instance RoundedMixedRingEffort Double Integer
    where
    type MixedRingOpsEffortIndicator Double Integer = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ((),())
    mxringEffortMult _ _ _ = ((),(),())

instance RoundedMixedRing Double Integer

instance RoundedMixedFieldEffort Double Integer
    where
    type MixedFieldOpsEffortIndicator Double Integer = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ((),())
    mxfldEffortMult _ _ _ = ((),(),())
    mxfldEffortDiv _ _ _ = ((),(),((),()))

instance RoundedMixedField Double Integer



instance RoundedMixedAddEffort Double Rational
    where
    type MixedAddEffortIndicator Double Rational =
        MixedAddEffortIndicatorByConversion Double Rational 
    mixedAddDefaultEffort = mixedAddDefaultEffortByConversion 
instance RoundedMixedAdd Double Rational
    where
    mixedAddUpEff = mixedAddUpEffByConversion
    mixedAddDnEff = mixedAddDnEffByConversion

instance RoundedMixedMultiplyEffort Double Rational
    where
    type MixedMultEffortIndicator Double Rational =
        MixedMultEffortIndicatorByConversion Double Rational 
    mixedMultDefaultEffort = mixedMultDefaultEffortByConversion 
instance RoundedMixedMultiply Double Rational
    where
    mixedMultUpEff = mixedMultUpEffByConversion
    mixedMultDnEff = mixedMultDnEffByConversion

instance RoundedMixedDivideEffort Double Rational
    where
    type MixedDivEffortIndicator Double Rational =
        MixedDivEffortIndicatorByConversion Double Rational 
    mixedDivDefaultEffort = mixedDivDefaultEffortByConversion 
instance RoundedMixedDivide Double Rational
    where
    mixedDivUpEff = mixedDivUpEffByConversion
    mixedDivDnEff = mixedDivDnEffByConversion

instance RoundedMixedRingEffort Double Rational
    where
    type MixedRingOpsEffortIndicator Double Rational = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ((),())
    mxringEffortMult _ _ _ = ((),(),())

instance RoundedMixedRing Double Rational

instance RoundedMixedFieldEffort Double Rational
    where
    type MixedFieldOpsEffortIndicator Double Rational = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ((),())
    mxfldEffortMult _ _ _ = ((),(),())
    mxfldEffortDiv _ _ _ = ((),(),((),()))

instance RoundedMixedField Double Rational

instance RoundedMixedAddEffort Double Double
    where
    type MixedAddEffortIndicator Double Double = ()
    mixedAddDefaultEffort _ _ = ()
instance RoundedMixedAdd Double Double
    where
    mixedAddUpEff = addUpEff
    mixedAddDnEff = addDnEff

instance RoundedMixedMultiplyEffort Double Double
    where
    type MixedMultEffortIndicator Double Double = ()
    mixedMultDefaultEffort _ _ = () 
instance RoundedMixedMultiply Double Double
    where
    mixedMultUpEff = multUpEff
    mixedMultDnEff = multDnEff

instance RoundedMixedDivideEffort Double Double
    where
    type MixedDivEffortIndicator Double Double = ()
    mixedDivDefaultEffort _ _ = () 
instance RoundedMixedDivide Double Double
    where
    mixedDivUpEff = divUpEff
    mixedDivDnEff = divDnEff

instance RoundedMixedRingEffort Double Double
    where
    type MixedRingOpsEffortIndicator Double Double = ()
    mixedRingOpsDefaultEffort _ _ = ()
    mxringEffortAdd _ _ _ = ()
    mxringEffortMult _ _ _ = ()

instance RoundedMixedRing Double Double

instance RoundedMixedFieldEffort Double Double
    where
    type MixedFieldOpsEffortIndicator Double Double = ()
    mixedFieldOpsDefaultEffort _ _ = ()
    mxfldEffortAdd _ _ _ = ()
    mxfldEffortMult _ _ _ = ()
    mxfldEffortDiv _ _ _ = ()

instance RoundedMixedField Double Double



