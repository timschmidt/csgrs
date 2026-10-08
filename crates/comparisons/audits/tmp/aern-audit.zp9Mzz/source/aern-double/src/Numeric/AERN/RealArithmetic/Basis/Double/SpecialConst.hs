{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.Double.SpecialConst
    Description :  common constants such as pi
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Common constants such as pi.
-}

module Numeric.AERN.RealArithmetic.Basis.Double.SpecialConst where

import Numeric.AERN.RealArithmetic.NumericOrderRounding

instance  
    RoundedSpecialConstEffort Double 
    where
    type SpecialConstEffortIndicator Double = () 
    specialConstDefaultEffort _ = () 

instance 
    RoundedSpecialConst Double 
    where
    piUpEff _ _ = encodeFloat 7074237752028441 (-51)
    piDnEff _ _ = encodeFloat 7074237752028440 (-51)
    eUpEff _ _ = encodeFloat 6121026514868074 (-51)
    eDnEff _ _ = encodeFloat 6121026514868073 (-51)
    