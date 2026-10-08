{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.Measures
    Description :  distance between Double numbers
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Distance between Double numbers.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.Double.Measures where

import Numeric.AERN.RealArithmetic.Basis.Double.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.Double.FieldOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.Interval.Double

import Numeric.AERN.Basics.Interval

instance HasDistance Double where
    type Distance Double = DI
    type DistanceEffortIndicator Double = ArithInOut.AddEffortIndicator DI
    distanceDefaultEffort d = ArithInOut.addDefaultEffort (sampleDI :: DI)
    distanceBetweenEff effort d1 d2 =
--        | d1 == 1/0 && d2 == 1/0 = zero 
--          -- distance between two infinities is zero (beware: distance not continuous at infinities!)  
--        | d1 == -1/0 && d2 == -1/0 = zero
--        | otherwise =
            ArithInOut.absOut (d2I <-> d1I)
            where
            d1I = Interval d1 d1
            d2I = Interval d2 d2
