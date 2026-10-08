{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.Basics.Interval
    Description :  a minimal interval datatype  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    A minimal interval datatype and its instances.
-}
module Numeric.AERN.Basics.Interval 
(
   module Numeric.AERN.Basics.Interval.Basics,
   module Numeric.AERN.Basics.Interval.Consistency,
   module Numeric.AERN.Basics.Interval.NumericOrder,
   module Numeric.AERN.Basics.Interval.RefinementOrder,
)
where

import Numeric.AERN.Basics.Interval.Basics
import Numeric.AERN.Basics.Interval.Consistency
import Numeric.AERN.Basics.Interval.NumericOrder
import Numeric.AERN.Basics.Interval.RefinementOrder

