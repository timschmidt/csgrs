{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
    Description :  convenience operators and functions with default effort  
    Description :  re-export of parent's operators for easier direct import  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Re-export of operators from the parent module for easier direct import.
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
( 
    (>+<), (<+>), (>-<), (<->),
    (>*<), (<*>), (>/<), (</>),
--    (>/+<), (</+>), (/+),
    (>^<), (<^>),
    (|>+<), (|<+>), (|+), (>+<|), (<+>|), (+|),
    (|>*<), (|<*>), (|*), (>*<|), (<*>|), (*|),
    (>/<|), (</>|), (/|)
)
where

import Prelude hiding ((<*>))
import Numeric.AERN.RealArithmetic.RefinementOrderRounding


