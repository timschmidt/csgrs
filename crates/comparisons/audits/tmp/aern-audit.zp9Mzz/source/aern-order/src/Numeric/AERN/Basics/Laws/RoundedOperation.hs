{-|
    Module      :  Numeric.AERN.Basics.Laws.Relation
    Description :  common properties of rounded binary operations 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common properties of rounded binary operations.
-}

module Numeric.AERN.Basics.Laws.RoundedOperation where

import Numeric.AERN.Basics.Laws.Utilities
import Numeric.AERN.Basics.Exception

import Numeric.AERN.Misc.Maybe
import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Debug

partialRoundedIdempotent :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (PartOp t) -> (PartOp t) -> 
    t -> Bool
partialRoundedIdempotent (<=?) (*^?) (*.?) e =
    (defined (e *.? e) && defined (e *^? e))
    ===>
    (roundedIdempotent  (<=?) (*^) (*.) e)
    where
    (*.) = assumeTotal2 (*.?)
    (*^) = assumeTotal2 (*^?)

partialRoundedCommutative :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (PartOp t) -> (PartOp t) -> 
    t -> t -> Bool
partialRoundedCommutative (<=?) (*^?) (*.?) e1 e2 =
    (and $ map defined [e1 *.? e2, e2 *.? e1, e1 *^? e2, e2 *^? e1])
    ===>
    (roundedCommutative (<=?) (*^) (*.) e1 e2)
    where
    (*.) = assumeTotal2 (*.?)
    (*^) = assumeTotal2 (*^?)

partialRoundedAssociative :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (PartOp t) -> (PartOp t) -> 
    t -> t -> t -> Bool
partialRoundedAssociative (<=?) (*^?) (*.?) e1 e2 e3 =
    (and $ map defined [e1 *.? e2, (e1 *. e2) *.? e3, e2 *.? e3, e1 *.? (e2 *. e3), 
                        e1 *^? e2, (e1 *^ e2) *^? e3, e2 *^? e3, e1 *^? (e2 *^ e3)])
    ===>
    (roundedAssociative (<=?) (*^) (*.) e1 e2 e3)
    where
    (*.) = assumeTotal2 (*.?)
    (*^) = assumeTotal2 (*^?)

roundedUnit :: 
    (Show t, HasLegalValues t) => 
    t -> (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> Bool
roundedUnit unit =
    equalRoundingUpDn11 "roundedUnit" 
        (\(*) e -> e) (\(*) e -> unit * e)  

roundedIdempotent :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> Bool
roundedIdempotent =
    equalRoundingUpDn11 "roundedIdempotent" 
        (\(*) e -> e) (\(*) e -> e * e)  

roundedCommutative :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> t -> Bool
roundedCommutative = 
    equalRoundingUpDn12 "roundedCommutative" 
        (\(*) e1 e2 -> e1 * e2) (\(*) e1 e2 -> e2 * e1)  


roundedAssociative :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> t -> t -> Bool
roundedAssociative = 
    equalRoundingUpDn13 "roundedAssociative"
        (\(*) e1 e2 e3 -> (e1 * e2) * e3) 
        (\(*) e1 e2 e3 -> e1 * (e2 * e3))  

roundedModular :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> (Op t) -> (Op t) -> (Op t) -> (Op t) -> t -> t -> t -> Bool
roundedModular = 
    equalRoundingUpDn23 "roundedModular"
        (\(/\) (\/) e1 e2 e3 -> (e1 /\ e3) \/ (e2 /\ e3)) 
        (\(/\) (\/) e1 e2 e3 -> ((e1 /\ e3) \/ e2) /\ e3)  

roundedLeftDistributive :: 
    (Show t, HasLegalValues t) => 
    (PRel t) -> 
    (Op t) -> (Op t) -> (Op t) -> (Op t) -> 
    t -> t -> t -> Bool
roundedLeftDistributive = 
    equalRoundingUpDn23 "roundedLeftDistributive"
        (\(/\) (\/) e1 e2 e3 -> e1 \/ (e2 /\ e3)) 
        (\(/\) (\/) e1 e2 e3 -> (e1 \/ e2) /\ (e1 \/ e3))     

equalRoundingUpDn11 :: 
    (Show t, HasLegalValues t) => 
    String ->
    (Expr1Op1 t) -> (Expr1Op1 t) -> (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> Bool
equalRoundingUpDn11 contextDescription expr1 expr2  (<=?) (*^) (*.) e =
    leqIfDefined contextDescription (<=?) (expr1 (*.) e) (expr2 (*^) e) 
    && 
    leqIfDefined contextDescription (<=?) (expr2 (*.) e) (expr1 (*^) e) 
    
equalRoundingUpDn12 :: 
    (Show t, HasLegalValues t) => 
    String ->
    (Expr1Op2 t) -> (Expr1Op2 t) -> (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> t -> Bool
equalRoundingUpDn12 contextDescription expr1 expr2  (<=?) (*^) (*.) e1 e2 =
    leqIfDefined contextDescription (<=?) (expr1 (*.) e1 e2) (expr2 (*^) e1 e2) 
    && 
    leqIfDefined contextDescription (<=?) (expr2 (*.) e1 e2) (expr1 (*^) e1 e2) 
    
equalRoundingUpDn13 :: 
    (Show t, HasLegalValues t) => 
    String ->
    (Expr1Op3 t) -> (Expr1Op3 t) -> (PRel t) -> 
    (Op t) -> (Op t) -> 
    t -> t -> t -> Bool
equalRoundingUpDn13 contextDescription expr1 expr2  (<=?) (*^) (*.) e1 e2 e3 =
    leqIfDefined contextDescription (<=?) (expr1 (*.) e1 e2 e3) (expr2 (*^) e1 e2 e3)
    && 
    leqIfDefined contextDescription (<=?) (expr2 (*.) e1 e2 e3) (expr1 (*^) e1 e2 e3) 
    
equalRoundingUpDn23 :: 
    (Show t, HasLegalValues t) => 
    String ->
    (Expr2Op3 t) -> (Expr2Op3 t) -> (PRel t) -> 
    (Op t) -> (Op t) -> (Op t) -> (Op t) -> 
    t -> t -> t -> Bool
equalRoundingUpDn23 contextDescription expr1 expr2  (<=?) (*^) (**^) (*.) (**.) e1 e2 e3 =
    leqIfDefined contextDescription (<=?) (expr1 (*.) (**.) e1 e2 e3) (expr2 (*^) (**^) e1 e2 e3)
    && 
    leqIfDefined contextDescription (<=?) (expr2 (*.) (**.) e1 e2 e3) (expr1 (*^) (**^) e1 e2 e3) 
    
leqIfDefined contextDescription (<=?) val1 val2 
    | result == True = result
    | otherwise =
        unsafePrint
        (
            contextDescription ++ ": leqIfDefined: val1 <= val2 failed:"
            ++ "\n val1 = " ++ show val1
            ++ "\n val2 = " ++ show val2
        ) $
        result
    where
    result = (defined (val1OK <=? val2OK)) ===> (val1OK <= val2OK) 
    val1OK = check val1
    val2OK = check val2
    check = detectIllegalValues contextDescription
    (<=) = assumeTotal2 (<=?)
    
     