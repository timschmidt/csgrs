{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Utilities
    Description :  utilities for MPFR instances
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Utilities for MPFR instances.
    
    This is a hidden internal module.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Utilities where

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up

import Numeric.AERN.Basics.Exception
import Control.Exception

detectNaNThrow :: String -> MPFR -> MPFR
detectNaNThrow msg a 
    | M.isNaN a =
        throw (AERNDomViolationException $ "domain violation in MPFR " ++ msg)
    | otherwise = a

detectNaNUp :: String -> MPFR -> MPFR
detectNaNUp _ a 
    | M.isNaN a = 1/0
    | otherwise = a

detectNaNDn :: String -> MPFR -> MPFR
detectNaNDn _ a 
    | M.isNaN a = -1/0
    | otherwise = a

