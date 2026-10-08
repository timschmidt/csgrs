{-|
    Module      :  Numeric.AERN.RealArithmetic.Auxiliary
    Description :  auxiliary generic operators and utilities  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Auxiliary generic operators and utilities.

    This is a hidden internal module.    
-}

module Numeric.AERN.RealArithmetic.Auxiliary where

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.Basics.Exception

import Control.Exception
import Control.Monad.ST


powerFromMult :: 
    t {-^ unit of @*@ -} -> 
    (t -> t -> t) {-^ associative binary operation @*@ -} ->
    t {-^ @x@ -} ->
    Int {-^ @n@ positive -} ->
    t {-^ product @x * x * ... * x@ of @n@ copies of @x@ -}
powerFromMult one mult x n
    | n < 0 = throw $ AERNException "powerFromMult does not support negative exponents"
    | otherwise = p n
    where
    p n
        | n == 0 = one
        | n == 1 = x
        | otherwise =
            case even n of
                True -> 
                    powHalf `mult` powHalf 
                False -> 
                    x `mult` (powHalf `mult` powHalf)
        where
        powHalf = p (n `div` 2)

