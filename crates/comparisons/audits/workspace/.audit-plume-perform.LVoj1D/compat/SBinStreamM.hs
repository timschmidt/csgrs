module SBinStreamM (

-- Types
SBinStream,

-- Constants
sbTen,
sbZero, sbOne, sbminusOne,
sbHalf, sbminusHalf,
sbQuarter, sbminusQuarter, 
sbThird, sbminusThird,   

sbTenL,
sbZeroL, sbOneL, sbminusOneL,
sbHalfL, sbminusHalfL,
sbQuarterL, sbminusQuarterL, 
sbThirdL, sbminusThirdL,   

-- Unary functions
one_plus_negx, mone_plus_posx, 
one_plus_twonegx, mone_plus_twoposx,
cappedDouble, p,

sbNorm, sbNormMax, sbNormExtra, sbNormMaxExtra,
sbSignum, sbAbs, sbAbsSign,

sbNegate, 

-- Shifts, etc.
sbShr, sbShl, sbShift, 

-- Binary arithmetic operations
sbDigitMul, sbAv, sbAvNy, sbMul,
sbAdd, sbSub,

sbAdd3, sbSub3, sbAv3, sbAv3Ny, sbMul,

sbIntDiv,
sbMax, sbMin,
sbGTE,
a,b -- Test

)

	where

import Utils
import LookInt

-- ********** Types **********
type SBinStream = [Lookint]



-- ********** Constants **********
sbTenL          = ligGlist sbTen     :: SBinStream -- equals (10 / 2^4)
sbOneL          = ligGlist sbOne     :: SBinStream
sbHalfL         = ligGlist sbHalf    :: SBinStream
sbThirdL        = ligGlist sbThird   :: SBinStream
sbQuarterL      = ligGlist sbQuarter :: SBinStream

sbZeroL         = ligGlist sbZero    :: SBinStream

sbminusQuarterL  = ligGlist sbminusQuarter :: SBinStream
sbminusThirdL    = ligGlist sbminusThird   :: SBinStream
sbminusHalfL     = ligGlist sbminusHalf    :: SBinStream
sbminusOneL      = ligGlist sbminusOne     :: SBinStream 


-- ********** Constants **********
sbTen          = ( 1: 0: 1: sbZero) :: SBinStream -- equals (10 / 2^4)
sbOne          = repeat 1             :: SBinStream
sbHalf         = ( 1: sbZero)       :: SBinStream
sbThird        = cycle [1, -1]        :: SBinStream
sbQuarter      = ( 0: 1: sbZero)    :: SBinStream

sbZero         = repeat 0             :: SBinStream

sbminusQuarter = ( 0: -1: sbZero)   :: SBinStream
sbminusThird   = cycle [-1,1]         :: SBinStream
sbminusHalf    = (-1:sbZero)        :: SBinStream
sbminusOne     = repeat (-1)         :: SBinStream 


-- ********** Unary Functions **********


-- p : (p d x) adjusts the input stream x so that (d:p d x) = x
--             if this is possible. 
p :: Lookint -> SBinStream -> SBinStream

p   a  ( b:x) | a==b = x
p   1  ( 0:x) = mone_plus_posx  x
p   0  ( 1:x) = one_plus_negx x
p   0  (-1:x) = mone_plus_posx  x
p (-1) ( 0:x) = one_plus_negx x
p   1  (-1:x) = sbminusOne
p (-1) ( 1:x) = sbOne



-- one_plus_negx : Returns (x+1) if x <  0
--                          1    if x >= 0
--                 Undefined case for x>=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (x+1).
one_plus_negx :: SBinStream -> SBinStream

one_plus_negx (a:x) | a ==  0 = (lig1 1 a:lignewl a (one_plus_negx x))
                    | a == -1 = (lig1 1 a:lignewl a x)
                    | a ==  1 = liglist 1 a
-- one_plus_negx  ( 1:x) = undefined  -- This case for test purposes only.



-- mone_plus_posx : Returns (x-1) if x >  0
--                           -1   if x <= 0
--                 Undefined case for x<=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (x+1).
mone_plus_posx :: SBinStream -> SBinStream

mone_plus_posx (a:x) | a ==  0 = (lig1 (-1) a:lignewl a (mone_plus_posx x))
		     | a ==  1 = (lig1 (-1) a:lignewl a x)
		     | a == -1 = liglist (-1) a
-- mone_plus_posx (-1:x) = undefined    -- This case for test purposes only.



-- one_plus_twonegx : Returns (2x+1) if x <  0
--                             1     if x >= 0
--                 Undefined case for x>=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (2x+1).
one_plus_twonegx :: SBinStream -> SBinStream

one_plus_twonegx  (a:x) | a ==  0 = (lig1 1 a:one_plus_twonegx x)
                        | a == -1 = x
                        | a ==  1 = liglist 1 a
-- one_plus_twonegx (1:x)  = undefined  -- This case for test purposes only.



-- mone_plus_twoposx : Returns (2x-1) if x >  0
--                              -1    if x <= 0
--                 Undefined case for x<=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (2x-1).
mone_plus_twoposx :: SBinStream -> SBinStream

mone_plus_twoposx  (a:x) | a ==  0 = (lig1 (-1) a:one_plus_twonegx x)
                         | a ==  1 = x
                         | a == -1 = liglist (-1) a
-- mone_plus_twoposx (-1:x) = undefined -- This case for test purposes only.




-- cappedDouble : Returns   1     if  x >=  1/2
--                          2x    if -1/2 < x < 1/2
--                         -1     if  x <= -1/2

-- May be able to improve lookahead here
cappedDouble :: SBinStream -> SBinStream
cappedDouble (a:x) | a ==  0 = x
                   | a ==  1 = (lig1 1 a :one_plus_twonegx x)  
                   | a == -1 = (lig1 (-1) a :mone_plus_twoposx x)  




-- sbNegate : Negate signed binary stream
sbNegate :: SBinStream -> SBinStream

sbNegate (a:x) | a ==  0 = (lig1 0 a:sbNegate x) 
	       | a ==  1 = (lig1 (-1) a:sbNegate x) 
	       | a == -1 = (lig1 1 a:sbNegate x) 


-- sbAbs : Returns absolute value of signed binary stream
sbAbs :: SBinStream -> SBinStream

sbAbs (a:x) | a ==  0 = (lig1 0 a:sbAbs x) 
	    | a ==  1 = (lig1 (1) a: x) 
	    | a == -1 = (lig1 1 a:sbNegate x) 


-- sbAbsSign : Returns absolute value and sign of original value
sbAbsSign :: SBinStream -> (Int, SBinStream)

sbAbsSign x@( 1:x') = ( 1, x)
sbAbsSign x@(-1:x') = (-1, sbNegate x)
sbAbsSign   ( a:x') = ( s, (a:x''))   where (s,x'') = sbAbsSign x'



-- Shift right by n places

sbShr :: SBinStream -> Integer -> SBinStream
sbShr sb 0 = sb
sbShr sb n = (0:(sbShr sb (n-1)))



-- Shift left (overflow poss)
sbShl :: SBinStream -> Integer -> SBinStream
sbShl x 0 = x
sbShl x n = sbShl (p 0 x) (n-1)


-- Shift right or left. Right = +ve, left = -ve.
sbShift :: SBinStream -> Integer -> SBinStream
sbShift x 0             = x
sbShift x n | n>0       = sbShr x n
sbShift x n | otherwise = sbShl x (-n)


-- Signum to a finite number of binary digits
sbSignum :: (Integral a) => SBinStream -> a -> Int
sbSignum _     0 = 0
sbSignum (0:x) n = sbSignum x (n-1)
sbSignum (a:x) _ = toInt a



-- Multiply signed binary stream by a signed bit.
sbDigitMul a     x  | a ==  1   = lignewl a x
		    | a ==  0   = liglist 0 a
		    | a == -1   = lignewl a (sbNegate x) 



choose n = if (n > 2)  then lig1 1 n else 
           if (n < -2) then lig1 (-1) n else lig1 0 n


sbAv'' (a1:x') (b1:y') digsum = (emit:(sbAv' (a1:x') (b1:y') c'))
	where {digsum2 = (2*digsum + a1 + b1);
	       emit = choose digsum2;
	       c' = digsum - (2*emit)}

sbAv' (a0:x) (b0:y) c = if (digsum `mod` 2 == 0) then
			   ((signum digsum):(sbAv' x y 0))
                        else sbAv'' x y digsum
	where digsum = (a0 + b0 + 2*c)

sbAv sbs1 sbs2 = sbAv' sbs1 sbs2  0 



sbAvNy'' (a1:x') (b1:y') digsum = (emit:(sbAvNy' (a1:x') (b1:y') c'))
	where {digsum2 = (2*digsum + a1 - b1);
               emit = choose digsum2;
	       c' = digsum - (2*emit)}


sbAvNy' (a0:x) (b0:y) c = if (digsum `mod` 2 == 0) then
			   ((signum digsum):(sbAvNy' x y 0))
                        else sbAvNy'' x y digsum
	where digsum = (a0 - b0 + 2*c)


sbAvNy sbs1 sbs2 = sbAvNy' sbs1 sbs2  0 


sbAdd x y =  p 0 (sbAv x y)    
sbSub x y =  p 0 (sbAvNy x y)




sbMul (a:  x) y      | a == 0  = ((lig1 0 a):sbMul x y)
sbMul x       (c: y) | c == 0  = ((lig1 0 c):sbMul x y)

sbMul ( a: b: x) y  | a==1 && b == -1  = sbMul (lig2 0 a b: lig2 1 a b: x) y
sbMul x ( c: d: y)  | c==1 && d == -1  = sbMul x (lig2 0 c d: lig2 1 c d: y)
sbMul ( a: b: x) y  | a== -1 && b == 1 = sbMul (lig2 0 a b: lig2 (-1) a b: x) y
sbMul x ( c: d: y)  | c== -1 && d == 1 = sbMul x (lig2 0 c d: lig2 (-1) c d: y)

sbMul (a:b:x) (c:d:y) 
	| a == 1 && b == 0 && c == -1 && d ==  -1 = 
	 sbAv (sbAv ((g (-1) l): (g 0 l): x') (sbAv x' y)) ((g (-1) l): (g 0 l): (g 0 l): sbMul x y)

	| a == 1 && b == 0 && c == -1 && d == 0 =
	 sbAv ((g (-1) l): sbAv (sbNegate x) y) ((g 0 l): (g 0 l): (g 0 l): sbMul x y)
	
	| a == 1 && b == 0 && c == 1 && d == 0 =
	 sbAv ((g 1 l): sbAv x y) ((g 0 l): (g 0 l): (g 0 l): sbMul x y)

	| a == 1 && b == 0 && c == 1 && d == 1 = 
	  sbAv (sbAv ((g 1 l): (g 0 l): x) (sbAv x y)) ((g 1 l): (g 0 l): (g 0 l): sbMul x y)

	| a == -1 && b == 0 && c == -1 && d == -1 = 
	  sbAv (sbAv ((g 1 l): (g 0 l): (sbNegate x)) (sbNegate (sbAv x y))) 
	       ((g 1 l): (g 0 l): (g 0 l): sbMul x y)
	| a == -1 && b == 0 && c == -1 && d == 0 = 
	  sbAv ( (g 1 l): sbNegate (sbAv x y)) ((g 0 l): (g 0 l): (g 0 l): sbMul x y)

	| a == -1 && b ==  0 && c == 1 && d == 0 =
	  sbAv ((g (-1) l): sbAv x (sbNegate y)) ((g 0 l): (g 0 l): (g 0 l): sbMul x y)

	| a == -1 && b ==  0 && c == 1 && d ==  1 = 
	sbAv (sbAv ( (g (-1) l): (g 0 l): x) (sbAv x (sbNegate y))) ((g (-1) l):(g 0 l):(g 0 l):sbMul x y)

	| a == 1 && b == 1 && c == 1 && d ==   1 =
	sbAv (sbAv ((g 1 l): xavy) xavy) ((g 1 l): (g 1 l): (g 1 l): sbMul x y)

	| a ==  1 && b ==  1 && c ==  1 && d ==  0 = 
	  sbAv (sbAv ((g 1 l): (g 0 l): y) (sbAv x y)) ((g 1 l): (g 0 l): (g 0 l): sbMul x y)

	| a ==  1 && b == 1 && c == -1 && d ==  0 = 
	sbAv (sbAv ( (g (-1) l): (g 0 l): y) (sbAv (sbNegate x) y)) ((g (-1) l):(g 0 l):(g 0 l):sbMul x y)

	| a == 1 && b == 1 && c == -1 && d ==  -1 =
	sbAv (sbAv ((g (-1) l): nxavy) nxavy) ((g (-1) l): (g (-1) l): (g (-1) l): sbMul x y)


	| a ==  -1 && b ==  -1 && c ==  -1 && d ==  -1 =
	sbAv (sbAv ((g 1 l): nxavny) nxavny) ((g 1 l): (g 1 l): (g 1 l): sbMul x y)
	
	| a == -1 && b ==  -1 && c == -1 && d ==  0 =
	 sbAv (sbAv ((g 1 l): (g 0 l): (sbNegate y)) (sbNegate (sbAv x y))) 
  	     ((g 1 l): (g 0 l): (g 0 l): sbMul x y)

	| a ==  -1 && b ==  -1 && c ==  1 && d ==  0 = 
  	 sbAv (sbAv ((g (-1) l): (g 0 l): y') (sbAv x y')) ((g (-1) l): (g 0 l): (g 0 l): sbMul x y)

	| a == -1 && b ==  -1 && c ==  1 && d ==  1 =
	sbAv (sbAv ((g (-1) l): xavny) xavny) ((g (-1) l): (g (-1) l): (g (-1) l): sbMul x y)
	where {x'     = sbNegate x;
	       y'     = sbNegate y;
	       xavy   = sbAv x y;
       	       xavny  = sbAv x (sbNegate y);
	       nxavy  = sbAv (sbNegate x) y;
	       nxavny = sbNegate (sbAv x y);
               l      = max (e a) (max (e b) (max (e c) (e d)))}

--sbMul (a:x) (b:y) | a==0 && b==0 = (lig2 0 a b: lig2 0 a b:(sbMul x y))
--sbMul (a:x) y     | a==0 = (lig1 0 a:sbMul x y)
--sbMul x    (b:y)  | b==0 = (lig1 0 b:sbMul x y)


sbMul3 (a:b:x) (c:d:y) = 
  sbAv3 (sbAv3 (a*d: (sbAv3 (sbDigitMul d x) (sbDigitMul b y))) 
       (sbAv3 (sbDigitMul c x) (sbDigitMul a y))) (a*c: b*c: b*d: sbMul3 x y)



-- Test
sbMul2 (a:b:x) (c:y) = 
  sbAv (sbAv (sbDigitMul a y) (sbAv (sbDigitMul c x) (sbDigitMul b y)))
       (a*c: b*c: sbMul2 x y)


------------- Alex's average. Used to test worse lookahead properties ----

carryav :: Lookint -> SBinStream -> SBinStream -> SBinStream

carryav d (d1:r1) (d2:r2) =
  let c = (2*d + d1 + d2) in
    if c >= 2 then (lig1 1 c): (carryav (c-4) r1 r2)
    else if c <= (-2) then (lig1 (-1) c) : (carryav (c+4) r1 r2)
    else (lig1 0 c) : carryav c r1 r2

sbAv3 (d1:r1) (d2:r2) = carryav (d1+d2) r1 r2

sbAv3Ny x y = sbAv3 x (sbNegate y)



sbAdd3 x y =  p 0 (sbAv3 x y)    
sbSub3 x y =  p 0 (sbAv3Ny x y)


-------------------------------------------------------------------------

sbIntDiv' (h:t) n s = if (s' >= n)  then (lig1 1 h:sbIntDiv' t n (s'-n)) else
	              if (s' <= -n) then (lig1 (-1) h:sbIntDiv' t n (s'+n)) 
  			            else (lig1 0 h:sbIntDiv' t n s')
	where s' = 2*s+h


sbIntDiv _ 0 = undefined
sbIntDiv x n = sbIntDiv' x n 0







-- sbNorm : Normalise a signed binary stream
sbNorm :: SBinStream -> (Integer, SBinStream)
sbNorm x = sbNorm' x 0


-- sbNorm : Normalise a signed binary stream to max no. of places.
sbNormMax :: (Integral a) => SBinStream -> a -> (Integer, SBinStream)
sbNormMax x max = sbNormMax' x 0 max


-- sbNormExtra' : Normalise signed binary stream with counter and 
--                detecting 1,-1 and -1,1 cases.
sbNormExtra' :: SBinStream -> Integer -> (Integer, SBinStream)
sbNormExtra' ( 0:  x')    n = sbNorm' x' (n+1)
sbNormExtra' ( 1: -1:x') n = sbNorm' (1:x') (n+1)
sbNormExtra' (-1:  1:x') n = sbNorm' (-1:x') (n+1)
sbNormExtra' x         n = (n, x)

-- sbNormMaxExtra : Normalise a signed binary stream to max no. of places, 
--                detecting 1,-1 and -1,1 cases.
sbNormMaxExtra :: (Integral a) => SBinStream -> a -> (Integer, SBinStream)
sbNormMaxExtra x max = sbNormMaxExtra' x 0 max

-- sbNormExtra : Normalise a signed binary stream, and
--                detecting 1,-1 and -1,1 cases.
sbNormExtra :: SBinStream -> (Integer, SBinStream)
sbNormExtra x = sbNormExtra' x 0


-- sbNormMaxExtra' : Normalise signed binary stream with counter and max,
--                detecting 1,-1 and -1,1 cases.
sbNormMaxExtra' :: (Integral a) => SBinStream -> Integer -> a -> 
			  (Integer, SBinStream)
sbNormMaxExtra' x     n 0   = (n,x)
sbNormMaxExtra' (0:t) n max = sbNormMax' t (n+1) (max-1)
sbNormMaxExtra' ( 1: -1:t) n max = sbNormMax' (1:t) (n+1) (max-1)
sbNormMaxExtra' (-1:  1:t) n max = sbNormMax' (-1:t) (n+1) (max-1)
sbNormMaxExtra' x     n _   = (n,x)



-- sbNorm' : Normalise signed binary stream with counter
sbNorm' :: SBinStream -> Integer -> (Integer, SBinStream)
sbNorm' (0:x') n = sbNorm' x' (n+1)
sbNorm' x      n = (n, x)


-- sbNormMax' : Normalise signed binary stream with counter and max
sbNormMax' :: (Integral a) => SBinStream -> Integer -> a -> 
			  (Integer, SBinStream)
sbNormMax' x     n 0   = (n,x)
sbNormMax' (0:t) n max = sbNormMax' t (n+1) (max-1)
sbNormMax' x     n _   = (n,x)




-- Alex's Max 
sbMax a@(da : ra) b@(db : rb)  =
  case  da - db  of
    2  ->  lignewl db a
    1  ->  ligu da db : sbMax  ra (mone_plus_posx rb)
    0  ->  ligu da db : sbMax  ra rb
    -1 ->  ligu db da : sbMax  (mone_plus_posx ra) rb
    _  ->  lignewl da b        


-- Alex's
sbMin a@(da : ra) b@(db : rb)  =
  case  da - db  of
    2  ->  lignewl da b
    1  ->  ligu db da : sbMin  (one_plus_negx ra) rb
    0  ->  ligu db da : sbMin  ra rb
    -1 ->  ligu da db : sbMin  ra (one_plus_negx rb)
    _  ->  lignewl db a   



-- Not computable...
sbGTE (a0:x)     (b0:y) | a0==b0 = sbGTE x y
sbGTE (1:x)      (-1:y)          = True
sbGTE (-1:x)     (1:y)           = False
sbGTE (1: 1:x)   (0:y)	         = True
sbGTE (1: 0:x)   (0:1:y)         = sbGTE (0:x) (-1:y)
sbGTE (1: 0:x)   (0:y)           = True
sbGTE (1: -1:x)  (0:y)           = sbGTE (1:x) y
sbGTE (-1: -1:x) (0:y)	         = False
sbGTE (-1: 0:x)  (0: -1:y)       = sbGTE (0:x) (1:y)
sbGTE (-1: 0:x)  (0:y)           = False
sbGTE (-1: 1:x)  (0:y)           = sbGTE (-1:x) y
sbGTE x y                        = sbGTE y x




-- Test

a = ligGlist ([1, 0, -1, 0, 0, -1, 1, 0, 0, -1, 0, 1, 0, 1, 0, 1, -1, 0, 1, 0,
 0, -1, 0, 1, 0, -1, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, -1, 1, 0, -1, 0,
 0, 0, 0, -1] ++ a)

b = ligGlist ([1, 1, 0, -1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 0, 1, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 1, 0, -1, 0, -1, -1, 1, 0, -1, 1, -1, 0, 0, 0, -1, 0, 1, 0, 0, 0, 1, 0] ++ b)
