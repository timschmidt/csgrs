{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RankNTypes #-}
{-|
    Module      :  Numeric.AERN.Basics.Laws.Relation
    Description :  common properties of arithmetic operations arbitrarily-little rounded  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common properties of arithmetic operations when these operations
    are rounded but the rounding can be diminished arbitrarily 
    by increasing an effort indicator.
-}

module Numeric.AERN.RealArithmetic.Laws where

import Numeric.AERN.RealArithmetic.Measures

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Laws.Utilities

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Debug
import Numeric.AERN.Misc.List
import Numeric.AERN.Misc.Maybe
import Data.Maybe

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion

import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Exception
import Control.Exception

roundedRefinementIsotone1 ::
    (RefOrd.PartialComparison t, 
     RefOrd.ArbitraryOrderedTuple t, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr1Eff ei t) ->
    (Expr1Eff ei t) ->
    (RefOrd.LEPair t) -> 
    ei -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
roundedRefinementIsotone1 contextDescription exprUp exprDn (RefOrd.LEPair (e1L, e1H)) effort effortComp =
    case RefOrd.pLeqEff effortComp resDn resUp of
        Just b -> b
        _ -> True
    where
    resUp = check $ exprUp effort e1H
    resDn = check $ exprDn effort e1L
    check = detectIllegalValues $ contextDescription ++ " refinement isotone"

roundedRefinementIsotone2 ::
    (RefOrd.PartialComparison t, 
     RefOrd.ArbitraryOrderedTuple t, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr2Eff ei t) ->
    (Expr2Eff ei t) ->
    (RefOrd.TwoLEPairs t) -> 
    ei -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
roundedRefinementIsotone2 
        contextDescription exprUp exprDn 
        (RefOrd.TwoLEPairs ((e1L, e1H), (e2L, e2H))) 
        effort 
        effortComp 
        =
--    unsafePrint ("\nroundedRefinementIsotone2: " 
--      ++ "\n Up: op(" ++ show e1H ++ ", " ++ show e2H ++ ") = " ++ show resUp 
--      ++ "\n Dn: op(" ++ show e1L ++ ", " ++ show e2L ++ ") = " ++ show resDn
--      ++ "\n" 
--    ) $
    case RefOrd.pLeqEff effortComp resDn resUp of
        Just b -> b
        _ -> True
    where
    resUp = check $ exprUp effort e1H e2H
    resDn = check $ exprDn effort e1L e2L
    check = detectIllegalValues $ contextDescription ++ " refinement isotone"

roundedUnit ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    t -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
roundedUnit unit =
    equalRoundingUpDnBin1Var1 "roundedUnit" (\_ _ e -> e) expr2
    where
    expr2 opEff effort e = 
        unit * e
        where
        (*) = opEff effort

roundedReflexiveCollapse ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    t -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
roundedReflexiveCollapse unit =
    equalRoundingUpDnBin1Var1 "roundedReflexiveCollapse" (\_ _ e -> unit) expr2
    where
    expr2 opEff effort e = 
        e * e
        where
        (*) = opEff effort

roundedCommutative ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> t -> Bool
roundedCommutative =
    equalRoundingUpDnBin1Var2 "roundedCommutative" expr1 expr2
    where
    expr1 opEff effort e1 e2 = 
        e1 * e2
        where
        (*) = opEff effort
    expr2 opEff effort e1 e2 = 
        e2 * e1
        where
        (*) = opEff effort

roundedAssociative ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> t -> t -> Bool
roundedAssociative =
    equalRoundingUpDnBin1Var3 "roundedAssociative" expr1 expr2
    where
    expr1 opEff effort e1 e2 e3 = 
        (e1 * e2) * e3
        where
        (*) = opEff effort
    expr2 opEff effort e1 e2 e3 = 
        e1 * (e2 * e3)
        where
        (*) = opEff effort

roundedDistributive ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp1, Show eiOp1, 
     EffortIndicator eiOp2, Show eiOp2,
     HasAntiConsistency t, Show t, HasLegalValues t) =>
    (PRelEff eiRel t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (ConsistencyEffortIndicator t) ->
    (eiRel, (eiOp1, eiOp2)) -> 
    t -> t -> t -> Bool
roundedDistributive 
        pCompareEff
        op1UpEff op2UpEff 
        op1DnEff op2DnEff 
        effortConsistency
        initEffort 
        e1 e2 e3 =
--    unsafePrint
--    (
--        "property roundedDistributive: "
--        ++ "\n e1 = " ++ show e1
--        ++ "\n e2 = " ++ show e2
--        ++ "\n e3 = " ++ show e3
--    ) $        
    thinEqualConsLeqRoundingUpDnImprovement "roundedDistributive"
        -- cannot get equality when e1 is not thin 
        -- because e1 appears twice in expr1 (dependency error)
        [e1] expr1Up expr1Dn expr2Up expr2Dn 
        pCompareEff
        effortConsistency 
        initEffort
    where
    expr1Up eff = expr1 op1UpEff op2UpEff eff e1 e2 e3
    expr1Dn eff = expr1 op1DnEff op2DnEff eff e1 e2 e3
    expr2Up eff = expr2 op1UpEff op2UpEff eff e1 e2 e3
    expr2Dn eff = expr2 op1DnEff op2DnEff eff e1 e2 e3
    expr1 op1Eff op2Eff (effort1, effort2) e1 e2 e3 = 
        (e1 * e2) + (e1 * e3)
        where
        (*) = op1Eff effort1
        (+) = op2Eff effort2
    expr2 op1Eff op2Eff (effort1, effort2) e1 e2 e3 = 
        e1 * (e2 + e3)
        where
        (*) = op1Eff effort1
        (+) = op2Eff effort2


roundedNegSymmetric ::
    (Neg t,
     EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    (PRelEff eiRel t) -> 
    (UnaryOpEff eiOp t) -> (UnaryOpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
roundedNegSymmetric =
    equalRoundingUpDnUnary1Var1 "roundedNegSymmetric" expr1 expr2
    where
    expr1 opEff effort e = 
         opEff effort e
    expr2 opEff effort e = 
         opEff effort (neg e) 

roundedIdempotent ::
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    (PRelEff eiRel t) -> 
    (UnaryOpEff eiOp t) -> (UnaryOpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
roundedIdempotent =
    equalRoundingUpDnUnary1Var1 "roundedIdempotent" expr1 expr2
    where
    expr1 opEff effort e = 
         opEff effort e
    expr2 opEff effort e = 
         opEff effort (opEff effort e)


equalRoundingUpDnUnary1Var1 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr1UnaryOp1Eff eiOp t) -> 
    (Expr1UnaryOp1Eff eiOp t) -> 
    (PRelEff eiRel t) -> 
    (UnaryOpEff eiOp t) -> (UnaryOpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
equalRoundingUpDnUnary1Var1 contextDescription expr1 expr2 pCompareEff opUpEff opDnEff 
        initEffort e =
    equalRoundingUpDn contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 opUpEff eff e
    expr1Dn eff = expr1 opDnEff eff e
    expr2Up eff = expr2 opUpEff eff e
    expr2Dn eff = expr2 opDnEff eff e

equalRoundingUpDnBin1Var1 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr1Op1Eff eiOp t) -> 
    (Expr1Op1Eff eiOp t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> Bool
equalRoundingUpDnBin1Var1 
        contextDescription expr1 expr2 pCompareEff 
        opUpEff opDnEff initEffort e =
    equalRoundingUpDn 
        contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 opUpEff eff e
    expr1Dn eff = expr1 opDnEff eff e
    expr2Up eff = expr2 opUpEff eff e
    expr2Dn eff = expr2 opDnEff eff e

equalRoundingUpDnBin1Var2 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr1Op2Eff eiOp t) -> 
    (Expr1Op2Eff eiOp t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> t -> Bool
equalRoundingUpDnBin1Var2 contextDescription expr1 expr2 pCompareEff opUpEff opDnEff 
        initEffort e1 e2 =
    equalRoundingUpDn contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 opUpEff eff e1 e2
    expr1Dn eff = expr1 opDnEff eff e1 e2
    expr2Up eff = expr2 opUpEff eff e1 e2
    expr2Dn eff = expr2 opDnEff eff e1 e2

equalRoundingUpDnBin1Var3 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp, Show eiOp, 
     Show t, HasLegalValues t) =>
    String ->
    (Expr1Op3Eff eiOp t) -> 
    (Expr1Op3Eff eiOp t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp t) -> (OpEff eiOp t) -> 
    (eiRel, eiOp) -> 
    t -> t -> t -> Bool
equalRoundingUpDnBin1Var3 contextDescription expr1 expr2 pCompareEff opUpEff opDnEff 
        initEffort e1 e2 e3 =
    equalRoundingUpDn contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 opUpEff eff e1 e2 e3
    expr1Dn eff = expr1 opDnEff eff e1 e2 e3
    expr2Up eff = expr2 opUpEff eff e1 e2 e3
    expr2Dn eff = expr2 opDnEff eff e1 e2 e3

equalRoundingUpDnBin2Var3 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp1, Show eiOp1, 
     EffortIndicator eiOp2, Show eiOp2,
     Show t, HasLegalValues t) =>
    String ->
    (Expr2Op3Eff eiOp1 eiOp2 t) -> 
    (Expr2Op3Eff eiOp1 eiOp2 t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (eiRel, (eiOp1, eiOp2)) -> 
    t -> t -> t -> Bool
equalRoundingUpDnBin2Var3 contextDescription expr1 expr2 pCompareEff 
        op1UpEff op2UpEff 
        op1DnEff op2DnEff 
        initEffort e1 e2 e3 =
    equalRoundingUpDn contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 op1UpEff op2UpEff eff e1 e2 e3
    expr1Dn eff = expr1 op1DnEff op2DnEff eff e1 e2 e3
    expr2Up eff = expr2 op1UpEff op2UpEff eff e1 e2 e3
    expr2Dn eff = expr2 op1DnEff op2DnEff eff e1 e2 e3

thinEqualConsLeqRoundingUpDnImprovementBin2Var3 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp1, Show eiOp1, 
     EffortIndicator eiOp2, Show eiOp2,
     HasAntiConsistency t, Show t, HasLegalValues t) =>
    String ->
    (Expr2Op3Eff eiOp1 eiOp2 t) -> 
    (Expr2Op3Eff eiOp1 eiOp2 t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) ->
    (ConsistencyEffortIndicator t) -> 
    (eiRel, (eiOp1, eiOp2)) -> 
    t -> t -> t -> Bool
thinEqualConsLeqRoundingUpDnImprovementBin2Var3
        contextDescription
        expr1 expr2 pCompareEff 
        op1UpEff op2UpEff 
        op1DnEff op2DnEff 
        effortConsistency
        initEffort 
        e1 e2 e3 =
    thinEqualConsLeqRoundingUpDnImprovement
        contextDescription
        [e1,e2,e3] expr1Up expr1Dn expr2Up expr2Dn 
        pCompareEff
        effortConsistency 
        initEffort
    where
    expr1Up eff = expr1 op1UpEff op2UpEff eff e1 e2 e3
    expr1Dn eff = expr1 op1DnEff op2DnEff eff e1 e2 e3
    expr2Up eff = expr2 op1UpEff op2UpEff eff e1 e2 e3
    expr2Dn eff = expr2 op1DnEff op2DnEff eff e1 e2 e3



equalRoundingUpDnBin2Var2 :: 
    (EffortIndicator eiRel, Show eiRel, 
     EffortIndicator eiOp1, Show eiOp1, 
     EffortIndicator eiOp2, Show eiOp2,
     Show t, HasLegalValues t) =>
    String ->
    (Expr2Op2Eff eiOp1 eiOp2 t) -> 
    (Expr2Op2Eff eiOp1 eiOp2 t) -> 
    (PRelEff eiRel t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (OpEff eiOp1 t) -> (OpEff eiOp2 t) -> 
    (eiRel, (eiOp1, eiOp2)) -> 
    t -> t -> Bool
equalRoundingUpDnBin2Var2 contextDescription expr1 expr2 pCompareEff 
        op1UpEff op2UpEff 
        op1DnEff op2DnEff 
        initEffort e1 e2 =
    equalRoundingUpDn contextDescription expr1Up expr1Dn expr2Up expr2Dn pCompareEff initEffort
    where
    expr1Up eff = expr1 op1UpEff op2UpEff eff e1 e2
    expr1Dn eff = expr1 op1DnEff op2DnEff eff e1 e2
    expr2Up eff = expr2 op1UpEff op2UpEff eff e1 e2
    expr2Dn eff = expr2 op1DnEff op2DnEff eff e1 e2

thinEqualConsLeqRoundingUpDnImprovement :: 
    (EffortIndicator eiRel, EffortIndicator eiOp,
     Show eiOp, Show eiRel,
     HasAntiConsistency t, Show t, HasLegalValues t) =>
    String ->
    [t] -> 
    (eiOp -> t) {-^ left hand side expression UP -} -> 
    (eiOp -> t) {-^ left hand side expression DN -} -> 
    (eiOp -> t) {-^ right hand side expression UP -} -> 
    (eiOp -> t) {-^ right hand side expression DN -} ->
    (PRelEff eiRel t) -> 
    (ConsistencyEffortIndicator t) -> 
    (eiRel, eiOp) -> Bool
thinEqualConsLeqRoundingUpDnImprovement
        contextDescription
        parameters
        expr1Up expr1Dn expr2Up expr2Dn
        pCompareEff
        consistencyEffort 
        initEffort@(effComp, effOp)
    | allConsistent && allAntiConsistent =
        okIfThin
    | allConsistent =
        okIfConsistent
    | allAntiConsistent =
        okIfAntiConsistent
    where
    allConsistent =
        and $ map isConsistent parameters
    allAntiConsistent =
        and $ map isAntiConsistent parameters
    okIfThin =
            (equalRoundingUpDn contextDescription
                expr1Up expr1Dn expr2Up expr2Dn 
                pCompareEff initEffort)
    okIfConsistent =
            leqRoundingUpDn expr1Dn expr2Up
    okIfAntiConsistent = 
            leqRoundingUpDn expr2Dn expr1Up
    isConsistent a = 
        justButNot False $ isConsistentEff consistencyEffort a
    isAntiConsistent a = 
        justButNot False $ isAntiConsistentEff consistencyEffort a
    leqRoundingUpDn expr1Dn expr2Up =
        case pCompareEff effComp (expr1Dn effOp) (expr2Up effOp) of
            Just res -> res
            Nothing -> True


equalRoundingUpDn :: 
    (EffortIndicator eiRel, EffortIndicator eiOp,
     Show eiOp, Show eiRel,
     Show t, HasLegalValues t) =>
    String ->
    (eiOp -> t) {-^ left hand side expression UP -} -> 
    (eiOp -> t) {-^ left hand side expression DN -} -> 
    (eiOp -> t) {-^ right hand side expression UP -} -> 
    (eiOp -> t) {-^ right hand side expression DN -} -> 
    (PRelEff eiRel t) -> 
    (eiRel, eiOp) -> Bool
equalRoundingUpDn 
        contextDescription
        expr1Up expr1Dn expr2Up expr2Dn 
        pCompareEff initEffort =
--    unsafePrint 
--    (
--        "equalRoundingUpDn:"
--        ++ "\n  efforts executed = \n" ++ unlines (map show $ take (comparisonCount + 1) efforts)
--        ++ "\n  5 successes = \n" ++ unlines (map show relevantSuccesses)
--    ) 
--    $
    case evalCatchDomViolationExceptions "checking a property" result of
            Left e -> True
                -- ignore tests during which a domain violation exception arises 
            Right res -> res
                -- throw an exception if the result is an illegal values (eg NaN) 
    where
    result = 
        (andUnsafeReportFirstFalse relevantSuccesses)  
    relevantSuccesses = take 5 successes
    successes = map check efforts
    efforts =
        (initEffort : ) $ take 15 $ effortIncrementSequence initEffort
    check (effortRel, effortOp) =
        successWithMsg 
        where
        successWithMsg =
            (success,
              "failure for effortRel = " ++ show effortRel 
                ++ " effortOp = " ++ show effortOp
              ++ "\n val1Dn <=? val2Up is " 
                ++ show val1Dn ++ " <=? " ++ show val2Up ++ " = "
                ++ show (val1Dn <=? val2Up)
              ++ "\n val2Dn <=? val1Up is " 
                ++ show val2Dn ++ " <=? " ++ show val1Up ++ " = "
                ++ show (val2Dn <=? val1Up)
            )
        success =
            (defined (val1Dn <=? val2Up) ===> (val1Dn <= val2Up))
            &&
            (defined (val2Dn <=? val1Up) ===> (val2Dn <= val1Up))
        val1Dn = check $ expr1Dn effortOp
        val1Up = check $ expr1Up effortOp
        val2Dn = check $ expr2Dn effortOp
        val2Up = check $ expr2Up effortOp
        check = detectIllegalValues contextDescription
        (<=) = assumeTotal2 (<=?)
        (<=?) = pCompareEff effortRel


