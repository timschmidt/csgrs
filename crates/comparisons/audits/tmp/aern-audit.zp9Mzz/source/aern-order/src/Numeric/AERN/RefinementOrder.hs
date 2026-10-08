{-|
    Module      :  Numeric.AERN.RefinementOrder
    Description :  classical and approximate domain bases and lattices (&#8849;⊑,&#8851;⊓,&#8852;⊔)  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Type classes representing classical as well as approximate 
    Comparisons, domain bases and lattices with the refinement order notation 
    (&#8849;,&#8851;,&#8852;).
    
    This module is meant to be imported qualified.
    It is recommended to use the prefix RefOrd.
-}
module Numeric.AERN.RefinementOrder 
(
    module Numeric.AERN.RefinementOrder.PartialComparison,
    module Numeric.AERN.RefinementOrder.Arbitrary,
    module Numeric.AERN.RefinementOrder.RoundedBasis,
    module Numeric.AERN.RefinementOrder.RoundedLattice,
    module Numeric.AERN.RefinementOrder.Extrema,
    module Numeric.AERN.RefinementOrder.IntervalLike,
)
where

import Numeric.AERN.RefinementOrder.PartialComparison
import Numeric.AERN.RefinementOrder.Arbitrary
import Numeric.AERN.RefinementOrder.RoundedBasis
import Numeric.AERN.RefinementOrder.RoundedLattice
import Numeric.AERN.RefinementOrder.Extrema
import Numeric.AERN.RefinementOrder.IntervalLike
