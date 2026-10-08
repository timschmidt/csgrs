{-|
    Module      :  Numeric.AERN.NumericOrder.Extrema
    Description :  types that have least and greatest elements  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Types that have least and greatest elements.
    
    This module is hidden and reexported via its parent NumericOrder. 
-}
module Numeric.AERN.NumericOrder.Extrema where

{-|
    A type with extrema.
-}
class (HasLeast t, HasGreatest t) => HasExtrema t

{-|
    A type with a least element.
-}
class HasLeast t where
    least :: t -> t

{-|
    A type with a greatest element.
-}
class HasGreatest t where
    greatest :: t -> t
    