{-|
    Module      :  Numeric.AERN.Basics.Laws.Relation
    Description :  common properties of binary operations 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common properties of binary operations.
-}
module Numeric.AERN.Basics.Laws.Operation
(
    idempotent, commutative, associative, 
    partialIdempotent, partialCommutative, partialAssociative, 
    modular, leftDistributive,  rightDistributive
)
where

import Numeric.AERN.Basics.Laws.Utilities

--import qualified Algebra.Laws as NP -- numeric-prelude

idempotent :: (Rel t) -> (Op t) -> t -> Bool
idempotent (==) (*) e = 
    (e * e) == e
    
commutative :: (Rel t) -> (Op t) -> t -> t -> Bool
commutative (==) (*) e1 e2 = 
    (e1 * e2) == (e2 * e1)

associative :: (Rel t) -> (Op t) -> t -> t -> t -> Bool
associative (==) (*) e1 e2 e3 = 
    ((e1 * e2) * e3) == (e1 * (e2 * e3))

modular :: (Rel t) -> (Op t) -> (Op t) -> t -> t -> t -> Bool
modular (==) (/\) (\/) e1 e2 e3 =
    ((e1 /\ e3) \/ (e2 /\ e3)) == (((e1 /\ e3) \/ e2) /\ e3)

leftDistributive :: (Rel t) -> (Op t) -> (Op t) -> t -> t -> t -> Bool
leftDistributive (==) (/\) (\/) e1 e2 e3 =
    (e1 \/ (e2 /\ e3)) == ((e1 \/ e2) /\ (e1 \/ e3))

rightDistributive :: (Rel t) -> (Op t) -> (Op t) -> t -> t -> t -> Bool
rightDistributive (==) (/\) (\/) e1 e2 e3 =
    ((e2 /\ e3) \/ e1) == ((e2 \/ e1) /\ (e3 \/ e1))

partialIdempotent :: (Rel t) -> (PartOp t) -> t -> Bool
partialIdempotent (==) (*?) e = 
    case e *? e of 
        Just r -> r == e
        _ -> True

partialCommutative :: (Rel t) -> (PartOp t) -> t -> t -> Bool
partialCommutative (==) (*?) e1 e2 = 
    case (e1 *? e2, e2 *? e1) of 
        (Just e12, Just e21) -> e12 == e21 
        _ -> True

partialAssociative :: (Rel t) -> (PartOp t) -> t -> t -> t -> Bool
partialAssociative (==) (*?) e1 e2 e3 = 
    case (e1 *? e2, e2 *? e3) of 
        (Just e12, Just e23) -> 
            case (e12 *? e3, e1 *? e23) of
                (Just eLR, Just eRL) -> eLR == eRL 
                _ -> True
        _ -> True

 