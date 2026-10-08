{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.SpecialConst
    Description :  common constants such as pi
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Common constants such as pi.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.SpecialConst 
()
where

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Numeric.AERN.Basics.Interval

instance (ArithUpDn.RoundedSpecialConstEffort e) =>
    (ArithInOut.RoundedSpecialConstEffort (Interval e))
    where
    type SpecialConstEffortIndicator (Interval e) = 
        ArithUpDn.SpecialConstEffortIndicator e
    specialConstDefaultEffort (Interval l r) = 
        ArithUpDn.specialConstDefaultEffort l

instance (ArithUpDn.RoundedSpecialConst e) => 
    (ArithInOut.RoundedSpecialConst (Interval e)) 
    where
    piInEff effort (Interval sample _) =
        Interval (ArithUpDn.piUpEff effort sample) (ArithUpDn.piDnEff effort sample)
    piOutEff effort (Interval sample _) =
        Interval (ArithUpDn.piDnEff effort sample) (ArithUpDn.piUpEff effort sample)
    eInEff effort (Interval sample _) =
        Interval (ArithUpDn.eUpEff effort sample) (ArithUpDn.eDnEff effort sample)
    eOutEff effort (Interval sample _) =
        Interval (ArithUpDn.eDnEff effort sample) (ArithUpDn.eUpEff effort sample)

