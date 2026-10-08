module FunctionAbstraction where

import Numeric.AERN.MPFRBasis.Interval

type R = Int -> MI
    
class RF fn where
    build :: (Rational, Rational) -> Expression -> fn
    evalR :: fn -> R -> R
    evalMI :: fn -> MI -> MI
    integrate :: fn -> fn
    supportsIntegration :: fn -> Bool
        
data Expression =
      ExprLit Rational
    | ExprVar
    | ExprPlus Expression Expression
    | ExprTimes Expression Expression
    | ExprNegate Expression
    | ExprExp Expression
    | ExprSqrt Expression
    | ExprAbs Expression


