{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.NumericOrder
    Description :  numeric order instances for Double  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Numeric order Comparison and lattice instances for Double.
    
    This is a private module reexported publicly via its parent.
-}
module Numeric.AERN.RealArithmetic.Basis.Double.NumericOrder
where

import Prelude hiding (EQ,LT,GT)

import Numeric.AERN.Basics.Exception
import Control.Exception

import Numeric.AERN.Basics.PartialOrdering
import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder (MinmaxEffortIndicator)

import Numeric.AERN.Basics.SizeLimits

import Numeric.AERN.Misc.List
import Numeric.AERN.Misc.Debug

import Data.Maybe

sampleD :: Double
sampleD = 0

nanD :: Double
nanD = 0/0

instance HasSizeLimits Double where
    type SizeLimits Double = ()
    getSizeLimits _ = ()
    defaultSizeLimits _ = ()
       
instance CanChangeSizeLimits Double where
    type SizeLimitsChangeEffort Double = ()
    sizeLimitsChangeDefaultEffort _ = ()
    changeSizeLimitsDnEff _ _ d = d
    changeSizeLimitsUpEff _ _ d = d
    changeSizeLimitsOutEff = error $ "AERN: changeSizeLimitsOutEff not defined for Double"
    changeSizeLimitsInEff = error $ "AERN: changeSizeLimitsInEff not defined for Double"

instance NumOrd.HasLeast Double where
    least _ = - 1/0

instance NumOrd.HasGreatest Double where
    greatest _ = 1/0

instance NumOrd.HasExtrema Double where

instance NumOrd.RoundedLatticeEffort Double where
    type MinmaxEffortIndicator Double = ()
    minmaxDefaultEffort _ = ()

instance NumOrd.RoundedLattice Double where
    maxUpEff _ a b = Prelude.max a b
    maxDnEff = NumOrd.maxUpEff 
    minUpEff _ a b = Prelude.min a b
    minDnEff = NumOrd.minUpEff 
    
