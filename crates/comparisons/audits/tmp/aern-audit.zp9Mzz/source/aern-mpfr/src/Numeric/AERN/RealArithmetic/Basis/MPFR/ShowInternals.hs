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

import Numeric.AERN.Basics.ShowInternals

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up
import qualified Data.Number.MPFR.Mutable as MM

import Data.Word

instance ShowInternals MPFR where
    type ShowInternalsIndicator MPFR = (Word, Bool)
--    defaultShowIndicator d = (10000, True)
    defaultShowIndicator d = (40, False)
    showInternals (prec,True) d =
        M.toStringExp prec d ++ "[gran = " ++ show gran ++ "]"
        where
        gran = M.getPrec d
    showInternals (prec,False) d = M.toStringExp prec d
        