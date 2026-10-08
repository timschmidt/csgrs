{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.ShowInternals
    Description :  showing MPFR values with mantissa and exponent
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded arithmetic instances for MPFR.
    
    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.ShowInternals where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics

import Numeric.AERN.Basics.ShowInternals

import Data.Word
import Numeric (showGFloat)

instance ShowInternals MPFR where
    type ShowInternalsIndicator MPFR = (Maybe Int, Bool)
    defaultShowIndicator d = (Just 40, False)
    showInternals (maybeDigits, shouldShowPrecision) d@(MPFR v) =
        floatS "" ++ precisionS
        where
        floatS = showGFloat maybeDigits v 
        precisionS
            | shouldShowPrecision = "[prec = " ++ show gran ++ "]"
            | otherwise = ""  
        gran = getPrecision d
        