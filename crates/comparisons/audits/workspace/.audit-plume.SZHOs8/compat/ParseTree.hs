module ParseTree (
Exp (Digits, Assign, Exp1, Error, Exit, NoExp),
Exp1 (Plus, Minus, Term, NoVar),
Term (Times, Div, NFactor, Pow),
NFactor (Factor, Neg),
Factor (Num, Intnum, Var, Brack, Sin, Cos, Arctan, Ln, Exponential,
        Sqrt, Pi, Max, Min),
Fn1, Fn2, 
AnIntnum (An_Intnum),
AVar (A_Var),
Token (TokenNum, TokenIntnum, TokenVar, TokenAssign, TokenPlus,
       TokenMinus, TokenTimes, TokenDiv, TokenOB, TokenCB, TokenPow,
       TokenDigits, TokenSin, TokenCos, TokenArctan, TokenLn, TokenExp,
       TokenSqrt, TokenPi, TokenMax, TokenMin, TokenComma,
       TokenExit, Err)) where

import Alex


data Exp  
	= Digits String
        | Assign String Exp1
        | Exp1 Exp1
	| Error
	| Exit
        | NoExp
   deriving Show

data Exp1 
        = Plus Exp1 Term 
        | Minus Exp1 Term 
        | Term Term
	| NoVar			-- result of a database error
   deriving (Show, Eq)

data Term 
        = Times Term NFactor 
        | Div Term NFactor 
        | Pow NFactor NFactor
        | NFactor NFactor
   deriving (Show, Eq)


data NFactor 
	= Factor Factor
	| Neg Factor
   deriving (Show, Eq)

data Factor 
	= Num String 
	| Intnum AnIntnum
        | Var AVar
        | Brack Exp1
	| Sin Fn1
	| Cos Fn1
	| Arctan Fn1
        | Exponential Fn1
	| Ln Fn1
        | Sqrt Fn1
	| Max Fn2 
	| Min Fn2
	| Pi
   deriving (Show, Eq)

data AVar = A_Var String
   deriving (Show, Eq)

data AnIntnum = An_Intnum String
   deriving (Show, Eq)

type Fn1 = Exp1
type Fn2 = (Exp1,Exp1)


data Token
        = TokenNum String
	| TokenIntnum String
        | TokenVar String
        | TokenDigits
        | TokenAssign
        | TokenPlus
        | TokenMinus
        | TokenTimes
        | TokenDiv
        | TokenOB
        | TokenCB        
	| TokenPow
	| TokenSin
	| TokenCos
	| TokenArctan
	| TokenExp
	| TokenLn
        | TokenSqrt
	| TokenMax
        | TokenMin
	| TokenPi
	| TokenExit
	| TokenComma
	| Err Posn
   deriving (Show, Eq)
