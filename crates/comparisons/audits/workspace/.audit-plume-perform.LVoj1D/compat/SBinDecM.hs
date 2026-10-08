module SBinDecM (
sbfGen, sbfEval, sbfDec, decSbf,
sbfDecOut, 

sbRawOut, sbfRawOut,

sbfGenNL, 
decSbfNL,

  ) where

import Utils
import SBinStreamM
import SBinFloatM
import LookInt

{-
sbfGen x = (ligGlist m,e) where
       (m,e) = sbfGenNL x
decSbf x = (ligGlist m,e) where
       (m,e) = decSbfNL x
-}

decSbf x = decSbfNL x
sbfGen x = sbfGenNL x

-- sbGen : Naive generation of signed bin. streams using Doubles.
sbGen :: Double -> SBinStream
sbGen r | r == 0.0 = sbZero
sbGen r   = if (two_r >= 1.0) then (1:(sbGen (two_r-1.0))) else
	      if (two_r <= -1.0) then (-1:(sbGen (two_r+1.0))) else
                (0:(sbGen two_r))
	    where two_r = r*2.0;


-- sbEval' : Auxiliary for sbEval
sbEval' :: Double -> SBinStream -> Double -> Int -> Double
sbEval' res _     _   0 = res
sbEval' res (d:x) exp n = sbEval' (res + ((fromIntegral d)/
					   exp)) x (exp*2.0) (n-1)

-- sbEval : Naive evaluation of signed bin streams using Doubles.
sbEval :: SBinStream -> Int -> Double
sbEval x n = sbEval' 0.0 x 2.0 n 



-- sbDecs : Signed binary stream -> fake signed decimal stream
--          Fake because the digits 10 and -10 can appear. I've
--          only ever seen this at the head of the stream, although
--          it's not clear that this is the only possible case, so 
--          other sbDec_ functions are written with the assumption that
--          they may appear elsewhere, although if they do the resulting
--          stream is still in [-1,1].
sbDecs :: SBinStream -> Int -> [Int]
sbDecs sbs 0 = []
sbDecs sbs n = (toInt (8*a+4*b+2*c+d):(sbDecs x (n-1)))
	    where (a:b:c:d:x) = sbMul sbTen sbs 


-- sbDecu' : Fake Signed decimal -> unsigned decimal string. Note carry:
--   carry c contains overflow information from seeing signed decimal 10,
--   -10, in addition to carrying the result of seeing a negative digit.
--       Carry in fn : -2,-1,0,1.  (Probably only ever see -1,0 ...)
--       Carry out :  0 : okay
--                    1 : overflow
--       Carry out restricted as s always equals sign of first digit when
--         called from sbDecu.
sbDecu' :: Int -> [Int] -> (Int,String)
sbDecu' s []    = (0,[])
sbDecu' s (a:x) =  (c',(itoc (a'-10*c')):r)
	          where {(c,r) = sbDecu' s x;
			  a' = (s*a)+c;
			  c' = (a' `quot` 10) - (fromEnum (a' < 0))}



-- sbDecu : Fake signed decimal stream -> decimal string and extra info
--         (e:s) ... e ==  2: positive with overflow
--		     e ==  1: positive, 
--		     e ==  0: Zero.
--                   e == -1: negative,
--		     e == -2: negative with overflow
sbDecu :: [Int] -> (Int,String)
sbDecu []    = (0,[])
sbDecu (0:x) = (s,(('0'):x'))
	       where (s,x') = sbDecu x
sbDecu (d:x) = (signum(d)*(1+c),x')
	       where (c,x') = sbDecu' (signum d) (d:x)


-- sbDec : Signed binary stream -> decimal string and extra info
--         (e:s) ... 
--         (e:s) ... e ==  2: positive with overflow
--		     e ==  1: positive, 
--		     e ==  0: Zero.
--                   e == -1: negative,
--		     e == -2: negative with overflow
--         If abs(e)==2, kill string output (== "00000...")
sbDec :: SBinStream -> Int -> (Int,String)
sbDec x n = if (abs(e) == 2) then (e,"") else (e,"."++s)
      where (e,s) = sbDecu (sbDecs x n)



-- Pseudo-decimal digit constants. 
sbDecDigs  0 = (sbZero)
sbDecDigs  1 = (0:0:0:1:sbZero)
sbDecDigs  2 = (0:0:1:0:sbZero)
sbDecDigs  3 = (0:0:1:1:sbZero)
sbDecDigs  4 = (0:1:0:0:sbZero)
sbDecDigs  5 = (0:1:0:1:sbZero)
sbDecDigs  6 = (0:1:1:0:sbZero)
sbDecDigs  7 = (0:1:1:1:sbZero)
sbDecDigs  8 = (1:0:0:0:sbZero)
sbDecDigs  9 = (1:0:0:1:sbZero)


-- decSb' : Auxiliary for decSb
--  Note: 5 not 10 used because average is addition with result div 2.
decSb' :: String -> SBinStream

decSb' []    = sbZero
decSb' [h]   = sbIntDiv (sbDecDigs (ctoi h)) 10
decSb' (h:t) = sbIntDiv (sbAv (sbDecDigs (ctoi h)) (decSb' t)) 5


-- decSb : Decimal part of number after decimal point -> signed binary
--         stream. Note, this means you can't generate 1 or -1!
--         Support for signs removed as this is wrong when used with
--         decSbf. This is only required for debugging, etc.
decSb :: String -> SBinStream
--decSb ('-':x) = sbShl (sbNegate (decSb' x)) 4 
--decSb ('+':x) = sbShl (decSb' x) 4
decSb (x)     = sbShl (decSb' x) 4



-- decintSb' : Auxiliary for decintSb. Present digits in reverse.
--             This isn't very nice...
--             The integer is a shift of some description
decintSb' :: String -> (Integer, SBinStream)
decintSb' []    = (0, sbZero)
decintSb' [a]   = (4, (sbDecDigs (ctoi a)))
decintSb' (a:x) = (n+5, sbAv (sbShr (sbDecDigs (ctoi a)) n) 
		             (sbMul rest sbTen))
	where (n, rest) = (decintSb' x) 

-- decintSb : Finite list of decimal digits representing an integer
--	      -> signed binary stream + exponent.
decintSb :: String -> (Integer, SBinStream)
decintSb ('-':x) = (e, sbNegate m)
	where (e, m) = sbfNormNoNegExp (decintSb' (reverse x))
decintSb ('+':x) = sbfNormNoNegExp (decintSb' (reverse x))
decintSb x       = sbfNormNoNegExp (decintSb' (reverse x))


--decintSbNS : decintSb with no sign checking
decintSbNS :: String -> SBinFloat
decintSbNS x = sbfNormNoNegExp (decintSb' (reverse x))


--Splits either side of but not including first instance of element e.
--Fails if e is last element.
splitOver _ []                 = ([],[])
splitOver e [a]   | e==a       = undefined
splitOver e (a:x) | e==a       = ([],x)
splitOver e (a:x) | otherwise  = ((a:x'),y')
	where (x',y') = splitOver e x


-- Note: can parse stuff like "+" as 0.
decSbfNL ('+':x) = sbfAdd (decintSbNS x') (0, decSb y')
 	where (x', y') = splitOver '.' x
decSbfNL ('-':x) = sbfNegate (sbfAdd (decintSbNS x') (0, decSb y'))
 	where (x', y') = splitOver '.' x
decSbfNL x = sbfAdd (decintSbNS x') (0, decSb y')
	where (x', y') = splitOver '.' x








-- Signum of a signed binary int (finite length list)
sbintSignum []    = 0
sbintSignum (0:x) = sbintSignum x
sbintSignum (a:x) = a


-- Generate a (positive) power of two.
gen2e e = (sbHalf, e+1)




sbfGenNL a = if ((a > 1.0) || (a < -1.0)) then (e+1, m)
          			       else (0, sbGen a) 
          where (e,m) = sbfGenNL (a/2.0)




-- This can't evaluate zero because it won't ever normalise ...
-- Unless we change sbfNorm to only try a max no. of zeros for normalisation.
sbfEval num n = (sbEval m' n) * (pow 2 e')
  where (e',m') = sbfNormNoNegExp num
--  where (e',m') = num





-- Divides a *finite* signed binary number by an integer and returns (div,rem)
-- Third parameter is zero if div result was [0,...,0], otherwise 1.
sbintIntDiv' [] _ s                 = ([],s,0)
sbintIntDiv' (h:t) n s | (s' >= n)  = ( 1:d,r,1)
	where {s'=2*s+h;(d,r,_) = sbintIntDiv' t n (s'-n)}
sbintIntDiv' (h:t) n s | (s' <= -n) = (-1:d,r,1) 
	where {s'=2*s+h;(d,r,_) = sbintIntDiv' t n (s'+n)}
sbintIntDiv' (h:t) n s | otherwise = ( 0:d,r,w) 
	where {s'=2*s+h;(d,r,w) = sbintIntDiv' t n s'}

sbintIntDiv _ 0 = undefined
sbintIntDiv x n = sbintIntDiv' x n 0 



-- Converts signed binary int into signed decimal.
sbintDecs' x = if (w==1) then (r:sbintDecs' d) else [r]
	where (d,r,w) = sbintIntDiv x 10

sbintDecs x = reverse (sbintDecs' x)

sbintDec = sbDecu.sbintDecs


-- sb Integer, sub 1
sbintsubOne = reverse.sbintsubOne'.reverse 


sbintsubOne' []     = [-1]
sbintsubOne' (1:x)  = (0:x)
sbintsubOne' (0:x)  = (-1:x)
sbintsubOne' (-1:x) = (0:sbintsubOne' x)


-- sb Integer, add 1
sbintaddOne = reverse.sbintaddOne'.reverse 

sbintaddOne' []     = [1]
sbintaddOne' (-1:x)  = (0:x)
sbintaddOne' (0:x)  = (1:x)
sbintaddOne' (1:x) = (0:sbintaddOne' x)

-- sbffracDec : Converts fractional part of SBinFloat to decimal
sbffracDec (e, m) n | e<=0 = sbDec (sbShr m (-e)) n

-- If intres>0, fracres<0, need (1-fracres).
--   also: Case where fracres starts -1,1,1 ...
--         Fix this by implementing a rounding for sbDec...



-- correct_ipart: Correct integer part of number being converted on the
--                basis of possible overflow in the fractional part.       
correct_ipart :: Int -> [Int] -> [Int]
correct_ipart c    x | c > 1     = sbintaddOne x
		     | c < -1    = sbintsubOne x
		     | otherwise = x


sbfDec' :: Int -> Int -> [Int] -> SBinStream -> Int -> String
sbfDec' _ 0 ipart _ n = signstr ++ istr
	  where {(isign,istr) = sbintDec ipart;
                  signstr     = if (isign<0) then "-" else ""}

sbfDec' isign fsign ipart fpart n = signstr ++ istr ++ fstr
	  where {(fcarry, fstr) =
		  case isign of {
		     1 -> if (fsign== 1) then sbDec fpart n
				         else sbDec (one_plus_negx fpart) n;
		     0 -> sbDec fpart n;
		    -1 -> if (fsign== 1) then sbDec (mone_plus_posx fpart) n 
				         else sbDec fpart n};
		  (sign, istr) = sbintDec (correct_ipart 
					       ((fsign*fcarry)-isign)
					        ipart);
		  signstr      = if (sign<0) then "-" else ""}
			    



-- Note: This can't deal with exponents greater than maxint. But then I
--       really don't think that's a problem think that 

sbfDec :: SBinFloat -> Int -> String
sbfDec (e,m) n | e>0 = sbfDec' isign fsign ipart fpart n
	where {ipart = toIntList (take (fromInteger e) m);  
	       fpart = drop (fromInteger e) m; 
               isign = sbintSignum ipart;
               fsign = sbSignum fpart (4*n)}
sbfDec x     n | otherwise = case sign of
				  2  ->  "1"++fracres
				  1  ->  "0"++fracres
				  0  ->  "0"
				  -1 -> "-0"++fracres
				  -2 -> "-1"++fracres
        where (sign,fracres) = sbffracDec x n





-- Prototype of function to return signed decimal as computation proceeds, 
--   and decimal result after completion.

sbfDecOut :: SBinFloat -> Int -> String
sbfDecOut x@(e,m) n  =  sbfRawOut (e,map fromIntegral (sbDecs m (n+((fromInteger e) `quot` 4))))
				  (4*n+(fromInteger e)) ++ "\n= " ++ 
		       (sbfDec x n)



-- NB: These raw output routines will work fine with signed decimal

-- sbfRawOut : Raw signed binary float output, prettified slightly.
sbfRawOut :: SBinFloat -> Int -> String
sbfRawOut (e,m) n = "2^(" ++ show e ++ ") * 0. " ++ sbRawOut m n

-- sbRawOut : Raw signed binary output, prettified slightly.
sbRawOut :: SBinStream -> Int -> String
sbRawOut [] _ = ""    -- Signed decimal cases.
sbRawOut _  0 = ""   
--sbRawOut (a:x) n = (replicate (2 - length(s)) ' ') ++ s ++ sbRawOut x (n-1)
sbRawOut (a:x) n = " " ++ s ++ sbRawOut x (n-1)
         where s = show a 








