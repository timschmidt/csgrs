module SBinStream (

-- Types
SBinStream,

-- Constants
sbTen,
sbZero, sbOne, sbminusOne,
sbHalf, sbminusHalf,
sbQuarter, sbminusQuarter, 
sbThird, sbminusThird,   

-- Unary functions
one_plus_negx, mone_plus_posx, 
one_plus_twonegx, mone_plus_twoposx,
cappedDouble, p,

sbNorm, sbNormMax,
sbNormExtra, sbNormMaxExtra,

sbSignum, sbAbs, sbAbsSign,

sbNegate, 

-- Shifts, etc.
sbShr, sbShl, sbShift, 

-- Binary arithmetic operations
sbDigitMul, sbAv, sbAvNy, sbMul, sbIntDiv,
sbAdd, sbSub,


sbMax, sbMin,
sbGTE,

)

	where

import Utils


-- ********** Types **********
type SBinStream = [Int]



-- ********** Constants **********
sbTen          = ( 1: 0: 1: sbZero) :: SBinStream -- equals (10 / 2^4)
sbOne          = ( 1: sbOne)        :: SBinStream
sbHalf         = ( 1: sbZero)       :: SBinStream
sbThird        = ( 1: -1: sbThird)  :: SBinStream
sbQuarter      = ( 0: 1: sbZero)    :: SBinStream

sbZero         = ( 0: sbZero)       :: SBinStream

sbminusQuarter = ( 0: -1: sbZero) :: SBinStream
sbminusThird    = (-1: 1:sbminusThird) :: SBinStream
sbminusHalf    = (-1:sbZero) :: SBinStream
sbminusOne     = ( -1:sbminusOne)  :: SBinStream 


-- ********** Unary Functions **********


-- p : (p d x) adjusts the input stream x so that (d:p d x) = x
--             if this is possible. 
p :: Int -> SBinStream -> SBinStream

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

one_plus_negx     ( 0:x) = (1:one_plus_negx x)
one_plus_negx     (-1:x) = (1:x)
one_plus_negx     ( 1:x) = sbOne
-- one_plus_negx  ( 1:x) = undefined  -- This case for test purposes only.



-- mone_plus_posx : Returns (x-1) if x >  0
--                           -1   if x <= 0
--                 Undefined case for x<=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (x+1).
mone_plus_posx :: SBinStream -> SBinStream

mone_plus_posx    ( 0:x) = (-1:mone_plus_posx x)
mone_plus_posx    ( 1:x) = (-1:x)
mone_plus_posx    (-1:x) = sbminusOne
-- mone_plus_posx (-1:x) = undefined    -- This case for test purposes only.



-- one_plus_twonegx : Returns (2x+1) if x <  0
--                             1     if x >= 0
--                 Undefined case for x>=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (2x+1).
one_plus_twonegx :: SBinStream -> SBinStream

one_plus_twonegx  ( 0:x) = (1:one_plus_twonegx x)
one_plus_twonegx  (-1:x) = x
one_plus_twonegx  ( 1:x) = sbOne
-- one_plus_twonegx (1:x)  = undefined  -- This case for test purposes only.



-- mone_plus_twoposx : Returns (2x-1) if x >  0
--                              -1    if x <= 0
--                 Undefined case for x<=0 is incorrect, but can be used to
--                   detect calls with pos. +ve x where result not (2x-1).
mone_plus_twoposx :: SBinStream -> SBinStream

mone_plus_twoposx ( 0:x) = (-1:mone_plus_twoposx x)
mone_plus_twoposx ( 1:x) = x
mone_plus_twoposx (-1:x) = sbminusOne
-- mone_plus_twoposx (-1:x) = undefined -- This case for test purposes only.




-- cappedDouble : Returns   1     if  x >=  1/2
--                          2x    if -1/2 < x < 1/2
--                         -1     if  x <= -1/2
cappedDouble :: SBinStream -> SBinStream

cappedDouble ( 0: x') = (x')
cappedDouble ( 1: x') = ( 1:one_plus_twonegx x')  
cappedDouble (-1: x') = (-1:mone_plus_twoposx x')




-- sbNegate : Negate signed binary stream
sbNegate :: SBinStream -> SBinStream

sbNegate (0:x) = (0:(sbNegate x)) 
sbNegate (1:x) = (-1:(sbNegate x)) 
sbNegate (-1:x) = (1:(sbNegate x)) 




-- sbAbs : Returns absolute value of signed binary stream
sbAbs :: SBinStream -> SBinStream

sbAbs ( 0:x) = (0:sbAbs x)
sbAbs ( 1:x) = (1:x)
sbAbs (-1:x) = (1:sbNegate x)



-- sbAbsSign : Returns absolute value and sign of original value
sbAbsSign :: SBinStream -> (Int, SBinStream)

sbAbsSign ( 0:x) = ( s, 0:x')   where (s,x') = sbAbsSign x
sbAbsSign ( 1:x) = ( 1, 1:x)
sbAbsSign (-1:x) = (-1, 1:sbNegate x)



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
sbSignum (a:x) _ = a



-- Multiply signed binary stream by a signed bit.
sbDigitMul :: Int -> SBinStream -> SBinStream
sbDigitMul 0  _ = sbZero
sbDigitMul 1  x = x
sbDigitMul _  x = sbNegate x



-- sbAv : Signed binary stream average operation
sbAv :: SBinStream -> SBinStream -> SBinStream
sbAv x y = sbAv' x y 0 

-- sbAv' : Auxiliary function for sbAv
sbAv' :: SBinStream -> SBinStream -> Int -> SBinStream
sbAv' (a0:x) (b0:y) c = if (d' `mod` 2 == 0) then
			   ((signum d'):(sbAv' x y 0))
                        else sbAv'' x y d'
	where d' = (a0 + b0 + 2*c)


-- sbAv'' : Auxiliary function for sbAv
sbAv'' :: SBinStream -> SBinStream -> Int -> SBinStream
sbAv'' (a1:x') (b1:y') d' = (e : sbAv' (a1:x') (b1:y') c')
	where {d = (2*d' + a1 + b1);
	       e = if (d >  2) then  1 else 
	           if (d < -2) then -1 else 0;
	       c' = d' - (2*e)}




-- sbAvNy : Signed binary stream average, negating second argument.
sbAvNy :: SBinStream -> SBinStream -> SBinStream
sbAvNy x y = sbAvNy' x y 0 

-- sbAvNy' : Auxiliary function for sbAvNy
sbAvNy' :: SBinStream -> SBinStream -> Int -> SBinStream
sbAvNy' (a0:x) (b0:y) c = if (d' `mod` 2 == 0) then
			   ((signum d'):(sbAvNy' x y 0))
                        else sbAvNy'' x y d'
	where d' = (a0 - b0 + 2*c)

-- sbAvNy'' : Auxiliary function for sbAvNy
sbAvNy'' :: SBinStream -> SBinStream -> Int -> SBinStream
sbAvNy'' (a1:x') (b1:y') d' = (e:(sbAvNy' (a1:x') (b1:y') c'))
	where {d = (2*d' + a1 - b1);
               e = if (d >  2) then  1 else 
		   if (d < -2) then -1 else 0;
	       c' = d' - 2*e}



-- sbAdd : Signed binary stream addition. 
--         Result undefined if not in [-1,1].
sbAdd :: SBinStream -> SBinStream -> SBinStream
sbAdd x y =  p 0 (sbAv x y)    

-- sbSub : Signed binary stream addition. 
--         Result undefined if not in [-1,1].
sbSub :: SBinStream -> SBinStream -> SBinStream
sbSub x y =  p 0 (sbAvNy x y)





-- sbMul : Signed binary stream multiplication.
sbMul :: SBinStream -> SBinStream -> SBinStream

sbMul (0:  x) (0:  y) = (0:0:(sbMul x y))
sbMul (0:  x) y       = (0:sbMul x y)
sbMul x       (0: y)  = (0:sbMul x y)

sbMul ( 1: -1: x) y   = sbMul (0: 1: x) y
sbMul x ( 1: -1: y)   = sbMul x (0: 1: y)
sbMul ( -1: 1: x) y   = sbMul (0: -1: x) y
sbMul x ( -1: 1: y)   = sbMul x (0: -1: y)

sbMul ( 1: 0: x)  (-1: -1: y) = 
	sbAv (sbAv (-1: 0: x') (sbAv x' y)) (-1: 0: 0: sbMul x y)
	where x' = sbNegate x
sbMul ( 1: 0: x)  (-1: 0: y) = sbAv (-1: sbAv (sbNegate x) y) (0: 0: 0: sbMul x y)
sbMul ( 1: 0: x)  ( 1: 0: y) = sbAv (1: sbAv x y) (0: 0: 0: sbMul x y)
sbMul ( 1: 0: x)  ( 1: 1: y) = 
  sbAv (sbAv (1: 0: x) (sbAv x y)) (1: 0: 0: sbMul x y)

sbMul (-1: 0: x)  (-1: -1: y) = 
  sbAv (sbAv (1: 0: (sbNegate x)) (sbNegate (sbAv x y))) (1: 0: 0: sbMul x y)
sbMul (-1: 0: x)  (-1: 0: y) = 
  sbAv ( 1: sbNegate (sbAv x y)) (0: 0: 0: sbMul x y)
sbMul (-1: 0: x)  ( 1: 0: y) =
  sbAv (-1: sbAv x (sbNegate y)) (0: 0: 0: sbMul x y)
sbMul (-1: 0: x)  ( 1: 1: y) = 
	sbAv (sbAv ( -1: 0: x) (sbAv x (sbNegate y))) (-1:0:0:sbMul x y)

sbMul ( 1: 1: x) ( 1:  1: y) =
	sbAv (sbAv (1: xavy) xavy) (1: 1: 1: sbMul x y)
	where xavy = sbAv x y
sbMul ( 1: 1: x)  ( 1: 0: y) = 
  sbAv (sbAv (1: 0: y) (sbAv x y)) (1: 0: 0: sbMul x y)
sbMul ( 1: 1: x)  (-1: 0: y) = 
	sbAv (sbAv ( -1: 0: y) (sbAv (sbNegate x) y)) (-1:0:0:sbMul x y)
sbMul ( 1:  1: x) (-1: -1: y) =
	sbAv (sbAv (-1: nxavy) nxavy) (-1: -1: -1: sbMul x y)
	where nxavy = sbAv (sbNegate x) y

sbMul ( -1: -1: x) ( -1: -1: y) =
	sbAv (sbAv (1: nxavny) nxavny) (1: 1: 1: sbMul x y)
	where nxavny = sbNegate (sbAv x y)
sbMul (-1: -1: x)  (-1: 0: y) = sbAv (sbAv (1: 0: (sbNegate y)) 
				           (sbNegate (sbAv x y))) 
				     (1: 0: 0: sbMul x y)
sbMul ( -1: -1: x)  ( 1: 0: y) = 
	sbAv (sbAv (-1: 0: y') (sbAv x y')) (-1: 0: 0: sbMul x y)
	where y' = sbNegate y
sbMul (-1: -1: x) ( 1: 1: y) =
	sbAv (sbAv (-1: xavny) xavny) (-1: -1: -1: sbMul x y)
	where xavny = sbAv x (sbNegate y)



-- sbMul2 : Multiplication of two signed binary streams
--          Unoptimised version.
sbMul2 :: SBinStream -> SBinStream -> SBinStream
sbMul2 (0:  x) (0:  y) = (0:0:(sbMul2 x y))
sbMul2 (0:  x) y       = (0:sbMul2 x y)
sbMul2 x       (0: y)  = (0:sbMul2 x y)
sbMul2 (a:b:x) (c:d:y) = 
  sbAv (sbAv (a*d: (sbAv (sbDigitMul d x) (sbDigitMul b y))) 
             (sbAv (sbDigitMul c x) (sbDigitMul a y))) 
       (a*c: b*c: b*d: sbMul2 x y)



-- sbIntDiv' : Auxiliary to sbIntDiv
sbIntDiv' :: SBinStream -> Int -> Int -> SBinStream 
sbIntDiv' (a:x) n s = if (s' >= n)  then (  1:sbIntDiv' x n (s'-n)) else
	              if (s' <= -n) then ( -1:sbIntDiv' x n (s'+n)) 
  			            else (  0:sbIntDiv' x n s')
	where s' = 2*s+a

-- sbIntDiv : Division of a signed binary stream by an integer
sbIntDiv :: SBinStream -> Int -> SBinStream 
sbIntDiv _ 0    = undefined
sbIntDiv x 1    = x
sbIntDiv x (-1) = sbNegate x
sbIntDiv x n    = sbIntDiv' x n 0



-- sbNorm : Normalise a signed binary stream
sbNorm :: SBinStream -> (Integer, SBinStream)
sbNorm x = sbNorm' x 0

-- sbNormExtra : Normalise a signed binary stream, and
--                detecting 1,-1 and -1,1 cases.
sbNormExtra :: SBinStream -> (Integer, SBinStream)
sbNormExtra x = sbNormExtra' x 0


-- sbNormMax : Normalise a signed binary stream to max no. of places.
sbNormMax :: (Integral a) => SBinStream -> a -> (Integer, SBinStream)
sbNormMax x max = sbNormMax' x 0 max

-- sbNormMaxExtra : Normalise a signed binary stream to max no. of places, 
--                detecting 1,-1 and -1,1 cases.
sbNormMaxExtra :: (Integral a) => SBinStream -> a -> (Integer, SBinStream)
sbNormMaxExtra x max = sbNormMaxExtra' x 0 max



-- sbNorm' : Normalise signed binary stream with counter
sbNorm' :: SBinStream -> Integer -> (Integer, SBinStream)
sbNorm' (0:x') n = sbNorm' x' (n+1)
sbNorm' x      n = (n, x)

-- sbNormExtra' : Normalise signed binary stream with counter and 
--                detecting 1,-1 and -1,1 cases.
sbNormExtra' :: SBinStream -> Integer -> (Integer, SBinStream)
sbNormExtra' ( 0:  x')    n = sbNorm' x' (n+1)
sbNormExtra' ( 1: -1:x') n = sbNorm' (1:x') (n+1)
sbNormExtra' (-1:  1:x') n = sbNorm' (-1:x') (n+1)
sbNormExtra' x         n = (n, x)

-- sbNormMax' : Normalise signed binary stream with counter and max
sbNormMax' :: (Integral a) => SBinStream -> Integer -> a -> 
			  (Integer, SBinStream)
sbNormMax' x     n 0   = (n,x)
sbNormMax' (0:t) n max = sbNormMax' t (n+1) (max-1)
sbNormMax' x     n _   = (n,x)

-- sbNormMaxExtra' : Normalise signed binary stream with counter and max,
--                detecting 1,-1 and -1,1 cases.
sbNormMaxExtra' :: (Integral a) => SBinStream -> Integer -> a -> 
			  (Integer, SBinStream)
sbNormMaxExtra' x     n 0   = (n,x)
sbNormMaxExtra' (0:t) n max = sbNormMax' t (n+1) (max-1)
sbNormMaxExtra' ( 1: -1:t) n max = sbNormMax' (1:t) (n+1) (max-1)
sbNormMaxExtra' (-1:  1:t) n max = sbNormMax' (-1:t) (n+1) (max-1)
sbNormMaxExtra' x     n _   = (n,x)


-- sbMax : Return max of two signed binary streams
sbMax :: SBinStream -> SBinStream -> SBinStream
sbMax (a0:x)    (b0:y) | a0==b0 = (a0: (sbMax x y))
sbMax ( 1:x)     (-1:y)         = (1:x)
sbMax (-1:x)     ( 1:y)         = (1:y)
sbMax ( 1:x)     ( 0:y)	        = (1:sbMax x (mone_plus_posx y))
sbMax ( 0:x)     ( 1:y)	        = (1:sbMax (mone_plus_posx x) y)
sbMax (-1:x)     ( 0:y)	        = (0:sbMax (mone_plus_posx x) y)
sbMax ( 0:x)     (-1:y)	        = (0:sbMax x (mone_plus_posx y))

-- sbMin : Return min of two signed binary streams
sbMin :: SBinStream -> SBinStream -> SBinStream
sbMin (a0:x)     (b0:y) | a0==b0 = (a0: (sbMin x y))
sbMin (1:x)      (-1:y)          = (-1:y)
sbMin (-1:x)     (1:y)           = (-1:x)
sbMin ( 1:x)     ( 0:y)	         = ( 0:sbMin (one_plus_negx x) y)
sbMin ( 0:x)     ( 1:y)	         = ( 0:sbMin x (one_plus_negx y))
sbMin (-1:x)     ( 0:y)	         = (-1:sbMin x (one_plus_negx y))
sbMin ( 0:x)     (-1:y)	         = (-1:sbMin (one_plus_negx x) y)


-- sbMin : Return true if x >= y
--         Not convergent. May not terminate.
sbGTE :: SBinStream -> SBinStream -> Bool
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


