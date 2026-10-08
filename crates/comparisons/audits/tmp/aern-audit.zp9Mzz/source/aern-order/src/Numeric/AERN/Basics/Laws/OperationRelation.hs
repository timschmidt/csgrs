{-|
    Module      :  Numeric.AERN.Basics.Laws.OperationRelation
    Description :  common properties linking binary operations with relations 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common properties linking binary operations with relations.
-}
module Numeric.AERN.Basics.Laws.OperationRelation 
(
--    joinOfOrderedPairNonReflexive, meetOfOrderedPairNonReflexive, 
    joinOfOrderedPair, meetOfOrderedPair, 
    joinAboveOperands, meetBelowOperands,
    partialJoinOfOrderedPair, partialJoinAboveOperands,
    downRoundedJoinOfOrderedPair, upRoundedMeetOfOrderedPair,
    downRoundedPartialJoinOfOrderedPair
)
where

import Numeric.AERN.Basics.Laws.Utilities

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Maybe

--joinOfOrderedPairNonReflexive ::
--    (PRel t) -> (PRel t) -> (Op t) -> t -> t -> Bool    
--joinOfOrderedPairNonReflexive (==) (<=) (\/) e1 e2 =
--    (e1 <= e2) ===> ((e1 \/ e2) == e2)
--
--meetOfOrderedPairNonReflexive ::
--    (PRel t) -> (PRel t) -> (Op t) -> t -> t -> Bool    
--meetOfOrderedPairNonReflexive (==) (<=) (/\) e1 e2 =
--    (e1 <= e2) ===> ((e1 /\ e2) == e1)

joinOfOrderedPair ::
    (Rel t) -> (PRel t) -> (Op t) -> t -> t -> Bool    
joinOfOrderedPair (==) (<=?) (\/) e1 e2 =
    (defined (e1 <=? e2))
    ===>
    ((e1 <= e2) <===> ((e1 \/ e2) == e2))
    where
    (<=) = assumeTotal2 (<=?)

meetOfOrderedPair ::
    (Rel t) -> (PRel t) -> (Op t) -> t -> t -> Bool    
meetOfOrderedPair (==) (<=?) (/\) e1 e2 =
    (defined (e1 <=? e2)) 
    ===>
    ((e1 <= e2) <===> ((e1 /\ e2) == e1))
    where
    (<=) = assumeTotal2 (<=?)

joinAboveOperands ::
    (PRel t) -> (Op t) -> t -> t -> Bool
joinAboveOperands (<=?) (\/) e1 e2 =
    (defined (e1 <=? (e1 \/ e2)) && defined (e2 <=? (e1 \/ e2))) 
    ===>
    ((e1 <= (e1 \/ e2)) && (e2 <= (e1 \/ e2)))
    where
    (<=) = assumeTotal2 (<=?)

meetBelowOperands ::
    (PRel t) -> (Op t) -> t -> t -> Bool    
meetBelowOperands (<=?) (/\) e1 e2 =
    (defined ((e1 /\ e2) <=? e1) && defined ((e1 /\ e2) <=? e2)) 
    ===>
    (((e1 /\ e2) <= e1) && ((e1 /\ e2) <= e2))
    where
    (<=) = assumeTotal2 (<=?)

partialJoinOfOrderedPair ::
    (Rel t) -> (PRel t) -> (PartOp t) -> t -> t -> Bool    
partialJoinOfOrderedPair (==) (<=?) (\/?) e1 e2 =
    (defined (e1 \/? e2)) ===>
    joinOfOrderedPair (==) (<=?) (assumeTotal2 (\/?)) e1 e2 

partialJoinAboveOperands ::
    (PRel t) -> (PartOp t) -> t -> t -> Bool    
partialJoinAboveOperands (<=?) (\/?) e1 e2 =
    (defined (e1 \/? e2)) ===>
    joinAboveOperands (<=?) (assumeTotal2 (\/?)) e1 e2



downRoundedJoinOfOrderedPair ::
    (PRel t) -> (Op t) -> t -> t -> Bool    
downRoundedJoinOfOrderedPair (<=?) (\/.) e1 e2 =
    (defined (e1 <=? e2) && defined (e1 \/. e2 <=? e2)) 
    ===>
    ((e1 <= e2) ===> (e1 \/. e2 <= e2))
    where
    (<=) = assumeTotal2 (<=?)

upRoundedMeetOfOrderedPair ::
    (PRel t) -> (Op t) -> t -> t -> Bool    
upRoundedMeetOfOrderedPair (<=?) (/\^) e1 e2 =
    (defined (e1 <=? e2) && defined (e1 <=? (e1 /\^ e2))) 
    ===>
    ((e1 <= e2) ===> (e1 <= (e1 /\^ e2)))
    where
    (<=) = assumeTotal2 (<=?)

downRoundedPartialJoinOfOrderedPair ::
    (PRel t) -> (PartOp t) -> t -> t -> Bool    
downRoundedPartialJoinOfOrderedPair (<=?) (\/.?) e1 e2 =
    (defined (e1 \/.? e2)) ===>
    downRoundedJoinOfOrderedPair (<=?) (assumeTotal2 (\/.?)) e1 e2 
