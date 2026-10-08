{-|
    Module      :  Numeric.AERN.Basics.Laws.PartialRelation
    Description :  common properties of partial relations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common properties of partial relations.
-}
module Numeric.AERN.Basics.Laws.PartialRelation
(
    partialReflexive, partialTransitive, partialSymmetric,
    partialOrderExtrema
)
where

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Maybe
import Numeric.AERN.Basics.Laws.Operation
import Numeric.AERN.Basics.Laws.Relation
import Data.Maybe (isNothing)

--import qualified Algebra.Laws as NP -- numeric-prelude

partialReflexive :: (t -> t -> Maybe Bool) -> t -> Bool
partialReflexive (~~?) e = 
    trueOrNothing $ e ~~? e 
    

partialTransitive :: (t -> t -> Maybe Bool) -> t -> t -> t -> Bool
partialTransitive (~~?) e1 e2 e3 =
    (and $ map defined [e1 ~~? e2, e2 ~~? e3, e1 ~~? e3]) ===>
    transitive (assumeTotal2 (~~?)) e1 e2 e3

partialSymmetric :: (t -> t -> Maybe Bool) -> t -> t -> Bool
partialSymmetric (~~?) e1 e2 =
    (and $ map defined [e1 ~~? e2, e2 ~~? e1]) ===>
    symmetric (assumeTotal2 (~~?)) e1 e2

partialOrderExtrema (<=?) bottom top e =
    (trueOrNothing $ bottom <=? e) 
    && 
    (trueOrNothing $ e <=? top)
