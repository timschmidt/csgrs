{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.SpecialConst
    Description :  support for common constants such as pi
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Support for common constants such as pi.
    
    This module is hidden and reexported via its parent RefinementOrderRounding. 
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.SpecialConst where

import Numeric.AERN.Basics.Effort

--import Numeric.AERN.Misc.Debug

class 
    (EffortIndicator (SpecialConstEffortIndicator t))
    =>
    RoundedSpecialConstEffort t 
    where
    type SpecialConstEffortIndicator t
    specialConstDefaultEffort :: t -> SpecialConstEffortIndicator t

class (RoundedSpecialConstEffort t) => RoundedSpecialConst t where
    piInEff :: (SpecialConstEffortIndicator t) -> t -> t
    piOutEff :: (SpecialConstEffortIndicator t) -> t -> t
    eInEff :: (SpecialConstEffortIndicator t) -> t -> t
    eOutEff :: (SpecialConstEffortIndicator t) -> t -> t

-- | Inward rounded pi with default effort
piIn :: (RoundedSpecialConst t) => t -> t
piIn sample = piInEff (specialConstDefaultEffort sample) sample

-- | Outward rounded pi with default effort
piOut :: (RoundedSpecialConst t) => t -> t
piOut sample = piOutEff (specialConstDefaultEffort sample) sample

-- | Inward rounded e with default effort
eIn :: (RoundedSpecialConst t) => t -> t
eIn sample = eInEff (specialConstDefaultEffort sample) sample

-- | Outward rounded e with default effort
eOut :: (RoundedSpecialConst t) => t -> t
eOut sample = eOutEff (specialConstDefaultEffort sample) sample

