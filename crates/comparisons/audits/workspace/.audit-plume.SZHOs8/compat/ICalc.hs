module ICalc (icalc) where

import IParser
import IParseTree
import IDatabase

import SBinFloat
import X_Repn
import Functions
import Integr

import SBinDec


type Poss_SBinFloat = Maybe SBinFloat  -- Monadic type to deal with
				       -- possibility that db lookup 
				       -- fails.

type Poss_sbf_to_sbf = Maybe (SBinFloat -> SBinFloat)

-- evaluate : Evaluate a parsed expression
evaluate :: Database -> Exp -> Int -> (Database, String, Int)
evaluate db (Digits d)     _ = (db, "Digits = "++(show d'), d')
	 where d' = read d
evaluate db (Assign s val) d = (dbStore db s val, ("Okay."), d)
evaluate db (Exp1 exp)     d = 
	 (db, maybe "No such var" (\a -> "Binary: "++(sbfRawOut a d))
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

evalFactor db (FMax (exp1, avar, x, y)) = (ievalExp1 db avar exp1) >>= (\a ->
				     (evalExp1 db x) >>= (\b -> 
				     (evalExp1 db y) >>= (\c ->
				      return (fnmax a b c))))
evalFactor db (FMin (exp1, avar, x, y)) = (ievalExp1 db avar exp1) >>= (\a ->
				     (evalExp1 db x) >>= (\b -> 
				     (evalExp1 db y) >>= (\c ->
				      return (fnmin a b c))))
evalFactor db (Integrate (exp1, avar, x, y)) 
                                   = (ievalExp1 db avar exp1) >>= (\a ->
				     (evalExp1 db x) >>= (\b -> 
				     (evalExp1 db y) >>= (\c ->
				      return (realintegrate a b c))))
evalFactor _  (Pi)               = return sbfPi



evalAVar :: Database -> AVar -> Poss_SBinFloat
evalAVar db (A_Var var)		   = if (fres == NoVar) then Nothing
				     else evalExp1 db fres
	 where fres = dbFetch db var

evalAnIntnum :: AnIntnum -> Poss_SBinFloat
evalAnIntnum (An_Intnum intnum)    = return (decSbf intnum)


-- ievaluate : Ievaluate a parsed expression
ievalExp1 :: Database -> AVar -> Exp1 -> Poss_sbf_to_sbf
ievalExp1 db v (Plus exp term)        = (ievalExp1 db v exp) >>= (\a -> 
					(ievalTerm db v term) >>= (\b -> 
					 return (\x -> sbfAdd (a x) (b x))))
ievalExp1 db v (Minus exp term)       = (ievalExp1 db v exp) >>= (\a -> 
					(ievalTerm db v term) >>= (\b -> 
					 return (\x -> sbfSub (a x) (b x))))
ievalExp1 db v (Term term)            = (ievalTerm db v term)

ievalTerm :: Database -> AVar -> Term -> Poss_sbf_to_sbf
ievalTerm db v (Times term nfactor) = (ievalTerm db v term) >>= (\a ->
				      (ievalNFactor db v nfactor) >>= (\b ->
				       return (\x -> sbfMul (a x) (b x))))
ievalTerm db v (Div term nfactor)   = (ievalTerm db v term) >>= (\a ->
				      (ievalNFactor db v nfactor) >>= (\b ->
				       return (\x -> sbfDiv (a x) (b x))))
ievalTerm db v (NFactor nfactor)    = (ievalNFactor db v nfactor)

ievalNFactor :: Database -> AVar -> NFactor -> Poss_sbf_to_sbf
ievalNFactor db v (Factor factor) = (ievalFactor db v factor)
ievalNFactor db v (Neg factor)  = (ievalFactor db v factor) >>= (\a ->
				   return (\x -> sbfNegate (a x)))

ievalFactor :: Database -> AVar -> Factor -> Poss_sbf_to_sbf
ievalFactor _  _ (Num num)          = return (\x -> (decSbf num))
ievalFactor _  _ (Intnum intnum)    = (evalAnIntnum intnum) >>= (\a ->
				       return (\_ -> a))
ievalFactor db v (Var avar)         = ievalAVar db v avar
ievalFactor db v (Brack exp1)       = (ievalExp1 db v exp1)
ievalFactor db v (Sin exp1)         = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfSin (a x)))
ievalFactor db v (Cos exp1)         = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfCos (a x)))
ievalFactor db v (Arctan exp1)      = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfArctan (a x)))
ievalFactor db v (Exponential exp1) = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfExpFn (a x)))
ievalFactor db v (Ln exp1)          = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfLn (a x)))
ievalFactor db v (Sqrt exp1)        = (ievalExp1 db v exp1) >>= (\a ->
				       return (\x -> sbfSqrt (a x)))
ievalFactor db v (Max (exp1a, exp1b))  = (ievalExp1 db v exp1a) >>= (\a ->  
				      (ievalExp1 db v exp1b) >>= (\b ->  
				      return (\x -> sbfMax (a x) (b x))))
ievalFactor db v (Min (exp1a, exp1b))  = (ievalExp1 db v exp1a) >>= (\a ->  
				      (ievalExp1 db v exp1b) >>= (\b ->  
				      return (\x -> sbfMin (a x) (b x))))
ievalFactor db v (Pi)               = return (\_ -> sbfPi)

ievalAVar :: Database -> AVar -> AVar -> Poss_sbf_to_sbf
ievalAVar db (A_Var v) (A_Var avar)
	    | v==avar   = return id
	    | otherwise = if (fres == NoVar) then Nothing
			  else ievalExp1 db (A_Var v) fres
	 where fres = dbFetch db avar




icalc db s d = evaluate db (parse s) d
