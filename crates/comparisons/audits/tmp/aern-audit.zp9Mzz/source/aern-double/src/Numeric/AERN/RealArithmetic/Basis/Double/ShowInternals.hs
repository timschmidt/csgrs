{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.ShowInternals
    Description :  showing Double values with mantissa and exponent
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded arithmetic instances for Double.
    
    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.Double.ShowInternals where

import Numeric.AERN.Basics.ShowInternals

instance ShowInternals Double where
    type ShowInternalsIndicator Double = Bool
    defaultShowIndicator d = False
    showInternals True d =
        show d ++ "[" ++ show mantissa ++ "," ++ show exponent ++ "]"
        where
        (mantissa, exponent) = decodeFloat d
    showInternals False d = show d
        