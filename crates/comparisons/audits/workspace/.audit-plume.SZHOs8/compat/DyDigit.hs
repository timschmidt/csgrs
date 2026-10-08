module DyDigit

(
-- Types
DyDigit,

-- Constants
dyd_One, dyd_Half, dyd_Quarter, dyd_Eighth,        
dyd_Zero,          
dyd_minusHalf, dyd_minusQuarter, dyd_minusEighth, dyd_minusOne,      

-- Unary Operations
dyd, dydNegate, dydDouble, 

-- Binary Arithmetic Operations
dydAv, dydAvNy, 
dydAdd, dydSub,
dydAddRC, dydSubRC,
dydMul,
dydShr, dydShl, 

-- Relationships
dydEq, 
dydGTE, dydGT,
dydLEQ, dydLT

   ) where

-- ********** Types **********
-- Dyadic digit type
-- DyDigit : (a,b) represents digit a/2^b
--           Digits always stored in normal form ... 
--           Most operations assume this.
type DyDigit = (Integer, Integer) 




-- ********** Constants **********
dyd_One           = ( 1,0) :: DyDigit
dyd_Half          = ( 1,1) :: DyDigit
dyd_Quarter       = ( 1,2) :: DyDigit
dyd_Eighth        = ( 1,3) :: DyDigit
dyd_Zero          = ( 0,0) :: DyDigit
dyd_minusHalf     = (-1,1) :: DyDigit
dyd_minusQuarter  = (-1,2) :: DyDigit
dyd_minusEighth   = (-1,3) :: DyDigit
dyd_minusOne      = (-1,0) :: DyDigit




-- ********** Unary Operations **********


-- dyd : Make dyadic digit
--       Checks for validity of digit.
dyd :: (Integral a) => (a,a) -> DyDigit
dyd (a,b) | b < 0        = undefined 
          | abs(a) > 2^b = undefined 
          | otherwise    = (toInteger a,toInteger b)     


-- dydNegate : Negate digit
dydNegate :: DyDigit -> DyDigit
dydNegate (a,b) = (-a,b)


-- dydDouble : Double digit (May go out of range)
dydDouble :: DyDigit -> DyDigit
dydDouble (a,0) = (2*a,0)
dydDouble (a,b) = (a,b-1)

-- dydNormalise : Normalise dyadic digits
dydNormalise :: DyDigit -> DyDigit
dydNormalise (0,b) = (0,0)
dydNormalise (a,0) = (a,0)
dydNormalise (a,b) | odd a     = (a,b)
		   | otherwise = dydNormalise (a `div` 2, b - 1)


-- ********** Binary Arithmetic Operations **********

-- dydAv : Average two dyadic digits
--         Assume digits in normal form first.

dydAv :: DyDigit -> DyDigit -> DyDigit
dydAv (a,b) (c,d) | b == d    = dydNormalise (a + c,b + 1)
		  | b > d     = (a + 2^(b-d)*c, b+1)
		  | otherwise = (2^(d-b)*a + c, d+1)


-- dydAvNy : Average two dyadic digits x and y, negating y
--           Assume digits in normal form first.

dydAvNy :: DyDigit -> DyDigit -> DyDigit
dydAvNy (a,b) (c,d) | b == d    = dydNormalise (a - c,b + 1)
	  	    | b > d     = (a - 2^(b-d)*c, b+1)
		    | otherwise = (2^(d-b)*a - c, d+1)


-- dydAdd : Add two dyadic digits
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydAdd :: DyDigit -> DyDigit -> DyDigit
dydAdd (a,b) (c,d) | b == d      = dydNormalise (a + c,b) 
		   | b > d       = (a + 2^(b-d)*c, b)
		   | otherwise   = (2^(d-b)*a + c, d)



-- dydSub : Dyadic digits, subtract y from x
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydSub :: DyDigit -> DyDigit -> DyDigit
dydSub (a,b) (c,d) | b == d      = dydNormalise (a - c,b) 
		   | b > d       = (a - 2^(b-d)*c, b)
		   | otherwise   = (2^(d-b)*a - c, d)



-- dydAddRC : Dyadic digits, add x to y
--            Assume digits in normal form first.
--            Output result mod 1, and remainder.
dydAddRC :: DyDigit -> DyDigit -> (DyDigit, DyDigit)
dydAddRC x y = if (a > two_e_b)  then (dyd_One, dydSub r dyd_One) else
	       if (a < -two_e_b) then (dyd_minusOne, dydAdd r dyd_One) 
	       else (r, dyd_Zero)
	 where {r@(a,b)   = dydAdd x y;
	        two_e_b   = 2^b}

-- dydSubRC : Dyadic digits, subtract y from x
--            Assume digits in normal form first.
--            Output result mod 1, and remainder.
dydSubRC :: DyDigit -> DyDigit -> (DyDigit, DyDigit)
dydSubRC x y = if (a > two_e_b)  then (dyd_One, dydSub r dyd_One) else
	       if (a < -two_e_b) then (dyd_minusOne, dydAdd r dyd_One) 
	       else (r, dyd_Zero)
	 where {r@(a,b)   = dydSub x y;
	        two_e_b   = 2^b}


-- dydMul : Multiply two dyadic digits
--          Assume digits in normal form first.
dydMul :: DyDigit -> DyDigit -> DyDigit
dydMul _     (0,_) = dyd_Zero
dydMul (0,_) _     = dyd_Zero
dydMul (a,b) (c,d) = (a*c, b+d)



-- dydShr : Dyadic digit shift right (divide by 2^n)
--          Assume digits in normal form first.
dydShr :: (Integral a) => DyDigit -> a -> DyDigit
dydShr (a,b) n = (a,b + (toInteger n))


-- dydShl : Dyadic digit shift left (multiply by 2^n)
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydShl :: (Integral a) => DyDigit -> a -> DyDigit
dydShl (a,b) n = (a,b -(toInteger n))

-- ********** Relationships **********


-- dydEq : Test for equality of dyadic digits
--         Assume digits in normal form first.
dydEq :: DyDigit -> DyDigit -> Bool
dydEq (a,b) (c,d) = ((a==c) && (b==d))



-- dydGTE : Dyadic digit greater than or equal to: (a >= b)?
--          Assume digits in normal form first.
dydGTE :: DyDigit -> DyDigit -> Bool
dydGTE (a,b) (c,d) | b == d    = (a >= c)
 	           | b > d     = (a >= c*2^(b-d)) 
 	           | otherwise = (a*2^(d-b) >= c) 


-- dydGT : Dyadic digit strictly greater than: (a > b)?
--          Assume digits in normal form first.
dydGT :: DyDigit -> DyDigit -> Bool
dydGT (a,b) (c,d) | b == d    = (a > c)
 	          | b > d     = (a > c*2^(b-d)) 
 	          | otherwise = (a*2^(d-b) > c) 



-- dydLEQ : Dyadic digit less than or equal to: (a <= b)?
--          Assume digits in normal form first.
dydLEQ :: DyDigit -> DyDigit -> Bool
dydLEQ (a,b) (c,d) | b == d    = (a <= c)
 	           | b > d     = (a <= c*2^(b-d)) 
 	           | otherwise = (a*2^(d-b) <= c) 


-- dydLT : Dyadic digit strictly less than: (a < b)?
--          Assume digits in normal form first.
dydLT :: DyDigit -> DyDigit -> Bool
dydLT (a,b) (c,d) | b == d    = (a < c)
 	          | b > d     = (a < c*2^(b-d)) 
 	          | otherwise = (a*2^(d-b) < c) 
