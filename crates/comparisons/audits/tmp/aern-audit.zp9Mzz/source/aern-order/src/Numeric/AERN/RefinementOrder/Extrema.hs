{-|
    Module      :  Numeric.AERN.RefinementOrder.Extrema
    Description :  types that have top and bottom  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Types that have top and bottom.
    
    This module is hidden and reexported via its parent RefinementOrder. 
-}
module Numeric.AERN.RefinementOrder.Extrema where

{-|
    A type with extrema.
-}
class (HasTop t, HasBottom t) => HasExtrema t

{-|
    A type with a top element.
-}
class HasTop t where
    top :: t -> t

{-|
    A type with a top element.
-}
class HasBottom t where
    bottom :: t -> t

-- | Convenience Unicode notation for 'top'
(⊤) :: (HasTop t) => t -> t   
(⊤) = top

-- | Convenience Unicode notation for 'bottom'
(⊥) :: (HasBottom t) => t -> t 
(⊥) = bottom
