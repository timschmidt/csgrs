{
module IParser (
-- Types
Exp, Exp1, Term, Factor,
Token,

-- Functions
parse

  ) where

import IParseTree
import ILexer

}

%name parser
%tokentype { Token }

%token 
	digits		{ TokenDigits }
	bye		{ TokenExit }
	num             { TokenNum $$ }
	intnum          { TokenIntnum $$ }
	var             { TokenVar $$ }
        sin		{ TokenSin }
        cos		{ TokenCos }
        arctan		{ TokenArctan }
	ln		{ TokenLn }
	exp		{ TokenExp }
	sqrt		{ TokenSqrt }
	max		{ TokenMax }
	min		{ TokenMin }
	int		{ TokenIntegr }
	fmax		{ TokenFMax }
	fmin		{ TokenFMin }
	','		{ TokenComma } 
	'pi'		{ TokenPi }
	':='            { TokenAssign }
	'+'             { TokenPlus }
	'-'             { TokenMinus }
	'*'             { TokenTimes }
	'/'             { TokenDiv }
	'('             { TokenOB }
	')'             { TokenCB }
        errorToken	{ Err $$ }   -- This comes up as an unused terminal
				     -- but it allows the parser to recover
				     -- from errors non-tokens in them.

%%

Exp   : digits ':=' intnum	{ Digits $3 }	
      | var ':=' Exp1           { Assign $1 $3 }
      | Exp1                    { Exp1 $1 }
      | bye			{ Exit }
      | {- Empty -}		{ NoExp }

Exp1  : Exp1 '+' Term           { Plus $1 $3 }
      | Exp1 '-' Term           { Minus $1 $3 }
      | Term                    { Term $1 }

Term  : Term '*' NFactor        { Times $1 $3 }
      | Term '/' NFactor        { Div $1 $3 }
      | NFactor                 { NFactor $1 }

NFactor  
      : '-' Factor              { Neg $2 }
      | Factor                  { Factor $1 }

Factor                    
      : num                     { Num $1 }
      | AnIntnum		{ Intnum $1 }
      | AVar                    { Var $1 }
      | 'pi'                    { Pi }
      | '(' Exp1 ')'            { Brack $2 }
      | sin Fn1			{ Sin $2 }
      | cos Fn1                 { Cos $2 }
      | arctan Fn1              { Arctan $2 }
      | exp Fn1                 { Exponential $2 }
      | ln Fn1                  { Ln $2 }
      | sqrt Fn1                { Sqrt $2 }
      | min Fn2			{ Min $2 }
      | max Fn2			{ Max $2 }
      | fmax Fn4  		{ FMax $2 }
      | fmin Fn4  		{ FMin $2 }
      | int Fn4 		{ Integrate $2 }

Fn1
      : '(' Exp1 ')'         { $2 }

Fn2 
     : '(' Exp1 ',' Exp1 ')'	{ ($2, $4) }

Fn4
      : '(' Exp1 ',' AVar ',' Exp1 ',' Exp1 ')'	{ ($2, $4, $6, $8) } 



AVar
      : var 			{ A_Var $1 }

AnIntnum
      : intnum                  { An_Intnum $1 }

{

happyError :: [Token] -> Exp
happyError _ = Error        -- Add error token

parse = parser . lexer

}


