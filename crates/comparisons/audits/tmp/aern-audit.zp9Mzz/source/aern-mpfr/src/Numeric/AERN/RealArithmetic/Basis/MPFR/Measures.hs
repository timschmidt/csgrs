{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Measures
    Description :  distance between MPFR numbers
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Distance between MPFR numbers.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Measures where

import Numeric.AERN.RealArithmetic.Basis.MPFR.NumericOrder
import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.Interval
import Numeric.AERN.RealArithmetic.Interval.MPFR

import Numeric.AERN.Basics.Interval

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up
import qualified Data.Number.MPFR.Mutable as MM

instance HasDistance MPFR where
    type Distance MPFR = MI
    type DistanceEffortIndicator MPFR = ArithInOut.AddEffortIndicator MI
    distanceDefaultEffort d = ArithInOut.addDefaultEffort (Interval d d)
    distanceBetweenEff effort d1 d2 =
--        | d1 == 1/0 && d2 == 1/0 = zero 
--          -- distance between two infinities is zero (beware: distance not continuous at infinities!)  
--        | d1 == -1/0 && d2 == -1/0 = zero
--        | otherwise =
            ArithInOut.absOut (d2I <-> d1I)
            where
            d1I = Interval d1 d1
            d2I = Interval d2 d2
            (<->) = ArithInOut.subtrOutEff effort
