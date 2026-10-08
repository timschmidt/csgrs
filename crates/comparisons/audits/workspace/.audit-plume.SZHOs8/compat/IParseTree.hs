module IParseTree (
Exp (Digits, Assign, Exp1, Error, Exit, NoExp),
Exp1 (Plus, Minus, Term, NoVar),
Term (Times, Div, NFactor),
NFactor (Factor, Neg),
Factor (Num, Intnum, Var, Brack, Sin, Cos, Arctan, Ln, Exponential, 
	Sqrt, Pi, Max, Min,
	Integrate, FMax, FMin),
Fn1, Fn2, Fn4, 
AnIntnum (An_Intnum),
AVar (A_Var),
Token (TokenNum, TokenIntnum, TokenVar, TokenAssign, TokenPlus,
       TokenMinus, TokenTimes, TokenDiv, TokenOB, TokenCB,
       TokenDigits, TokenSin, TokenCos, TokenArctan, TokenLn, TokenExp,
       TokenSqrt, TokenPi, TokenMax, TokenMin,
       TokenIntegr, TokenComma, TokenFMax,  TokenFMin, 
       TokenExit, Err)) where


import Alex


data Exp  
	= Digits String
	| Assign String Exp1
        | Exp1 Exp1
	| Error
	| Exit
	| NoExp
   deriving (Show, Eq)

data Exp1 
        = Plus Exp1 Term 
        | Minus Exp1 Term 
        | Term Term
	| NoVar
   deriving (Show, Eq)

data Term 
        = Times Term NFactor 
        | Div Term NFactor 
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
	| Integrate Fn4
	| FMax Fn4
	| FMin Fn4
	| Pi
   deriving (Show, Eq)

type Fn1 = Exp1
type Fn2 = (Exp1,Exp1)
type Fn4 = (Exp1, AVar, Exp1, Exp1)

data AVar = A_Var String
   deriving (Show, Eq)

data AnIntnum = An_Intnum String
   deriving (Show, Eq)

data Token
        = TokenNum String
        | TokenIntnum String
        | TokenVar String
        | TokenAssign
	| TokenDigits
        | TokenPlus
        | TokenMinus
        | TokenTimes
        | TokenDiv
        | TokenOB
        | TokenCB
	| TokenComma
	| TokenSin
	| TokenCos
	| TokenArctan
	| TokenExp
	| TokenLn
        | TokenSqrt
	| TokenMax
        | TokenMin
	| TokenIntegr
	| TokenFMax
	| TokenFMin
	| TokenPi
	| TokenExit
	| Err Posn
   deriving (Show, Eq)
