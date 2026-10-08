{-|
    Module      :  Numeric.AERN.Misc.Bool
    Description :  miscellaneous boolean functions  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Miscellaneous boolean functions.
-}
module Numeric.AERN.Misc.Bool where

(===>) :: Bool -> Bool -> Bool
a ===> b = not a || b 

(<===>) :: Bool -> Bool -> Bool
a <===> b = a == b 
