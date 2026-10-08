{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Effort
    Description :  MPFR precision is an effort indicator
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    MPFR precision is an effort indicator.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Effort
(
)
where

import Numeric.AERN.Basics.Effort

import qualified Data.Number.MPFR as M

import Test.QuickCheck

instance Arbitrary M.Precision where
    arbitrary =
        do
        p <- choose (10,1000)
        return (fromInteger $ toInteger (p :: Int))
    
instance EffortIndicator M.Precision where
    effortIncrementVariants p = [2*p + 1]
    effortRepeatIncrement (p1, p2) 
        | p2 > 2*p1 = 2*p2
        | otherwise = p2 + (p2 - p1)
    effortIncrementSequence p =
        map (p +) fibsp2p
        where
        fibsp2p = scanl (+) p (p:fibsp2p)
    effortCombine = max
        
    
    