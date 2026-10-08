module Calc (evaluate, calc) where

import Parser
import ParseTree
import Database

import SBinFloat
import X_Repn
import Functions
import Integr

import SBinDec


type Poss_SBinFloat = Maybe SBinFloat  -- Monadic type to deal with
				       -- possibility that db lookup 
				       -- fails.

-- evaluate : Evaluate a parsed expression
evaluate :: Database -> Exp -> Int -> (Database, String, Int)
evaluate db (Digits d) _ = (db, "Digits = "++(show d'), d')
	 where d' = read d
evaluate db (Assign s val) d = (dbStore db s val, ("Okay."), d)
evaluate db (Exp1 exp)     d = 
	 (db, maybe "No such var" (\a -> sbfDecOut a d)
		    (evalExp1 db exp), d)

evaluate db (Error)        d = (db, "Parse Error", d)
evaluate db (NoExp)        d = (db, "", d)
evaluate db (Exit)         d = (emptyDB, "Exiting", -1)

evalExp1 :: Database -> Exp1 -> Poss_SBinFloat
evalExp1 db (Plus exp term)        = (evalExp1 db exp) >>= (\a ->
				     (evalTerm db term) >>= (\b ->
				      return (sbfAdd a b)))
evalExp1 db (Minus exp term)       = (evalExp1 db exp) >>= (\a ->
				     (evalTerm db term) >>= (\b ->
				      return (sbfSub a b)))
evalExp1 db (Term term)            = (evalTerm db term)

evalTerm :: Database -> Term -> Poss_SBinFloat
evalTerm db (Times term nfactor) = (evalTerm db term) >>= (\a ->
				   (evalNFactor db nfactor) >>= (\b ->
				    return (sbfMul a b)))
evalTerm db (Div term nfactor)   = (evalTerm db term) >>= (\a ->
	 			   (evalNFactor db nfactor) >>= (\b ->
				    return (sbfDiv a b)))
evalTerm db (Pow nf1 nf2)
				 = (evalNFactor db nf1) >>= (\a ->
				   (evalNFactor db nf2) >>= (\b ->
				    return (sbfExpon a b)))
evalTerm db (NFactor nfactor)    = evalNFactor db nfactor

evalNFactor :: Database -> NFactor -> Poss_SBinFloat
evalNFactor db (Factor factor)   = evalFactor db factor
evalNFactor db (Neg factor)      = (evalFactor db factor) >>= (\a ->
				    return (sbfNegate a))



evalFactor :: Database -> Factor -> Poss_SBinFloat
evalFactor _  (Num num)          = return (decSbf num)
evalFactor _  (Intnum intnum)    = evalAnIntnum intnum
evalFactor db (Var avar)         = evalAVar db avar
evalFactor db (Brack exp1)       = evalExp1 db exp1
evalFactor db (Sin exp1)         = (evalExp1 db exp1) >>= (\a -> 
				    return (sbfSin a))
evalFactor db (Cos exp1)         = (evalExp1 db exp1) >>= (\a ->  
				    return (sbfCos a))
evalFactor db (Arctan exp1)      = (evalExp1 db exp1) >>= (\a ->  
				    return (sbfArctan a))
evalFactor db (Exponential exp1) = (evalExp1 db exp1) >>= (\a ->  
				    return (sbfExpFn a))
evalFactor db (Ln exp1)          = (evalExp1 db exp1) >>= (\a ->  
				    return (sbfLn a))
evalFactor db (Sqrt exp1)        = (evalExp1 db exp1) >>= (\a ->  
				    return (sbfSqrt a))
evalFactor db (Max (exp1a, exp1b))  = (evalExp1 db exp1a) >>= (\a ->  
				   (evalExp1 db exp1b) >>= (\b ->  
				    return (sbfMax a b)))
evalFactor db (Min (exp1a, exp1b))  = (evalExp1 db exp1a) >>= (\a ->  
				   (evalExp1 db exp1b) >>= (\b ->  
				    return (sbfMin a b)))
evalFactor _  (Pi)               = return sbfPi



evalAVar :: Database -> AVar -> Poss_SBinFloat
evalAVar db (A_Var var)		   = if (fres == NoVar) then Nothing
				     else evalExp1 db fres
	 where fres = dbFetch db var

evalAnIntnum :: AnIntnum -> Poss_SBinFloat
evalAnIntnum (An_Intnum intnum)    = return (decSbf intnum)



calc db s d = evaluate db (parse s) d



