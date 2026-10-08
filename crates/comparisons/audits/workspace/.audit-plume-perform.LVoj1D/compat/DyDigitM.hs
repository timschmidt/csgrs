module DyDigitM

(
-- Types
DyDigit, DyDigit',

-- Constants
dyd_One', dyd_Half', dyd_Quarter', dyd_Eighth',        
dyd_Zero',          
dyd_minusHalf', dyd_minusQuarter', dyd_minusEighth', dyd_minusOne',   
   
dyd_One, dyd_Half, dyd_Quarter, dyd_Eighth,        
dyd_Zero,          
dyd_minusHalf, dyd_minusQuarter, dyd_minusEighth, dyd_minusOne,   
-- Unary Operations
dyd, dydNegate, dydDouble, 
dyd_Look,dyd_LMax, dyd_LMax2,
-- Binary Arithmetic Operations
dydAv, dydAvNy, 
dydAdd, dydSub,
dydAddRC, dydSubRC,
dydMul,
dydShr, dydShl, 

-- Relationships
dydEq, 
dydGTE, dydGT,
dydLEQ, dydLT,

-- Lookahead tests

dydList', dydList,
dydListCL', dydListCL,
dydLMax,

dyex_Lookahead

   ) where

-- ********** Types **********
-- Dyadic digit type
-- DyDigit : (a,b) represents digit a/2^b
--           Digits always stored in normal form ... 
--           Most operations assume this.

type DyDigit = (Integer, Integer, Int) 
type DyDigit' = (Integer, Integer) 



data PREDEFS = Mone | Zero | One

mkdyd0 :: PREDEFS -> DyDigit
mkdyd0 Mone = dyd_minusOne
mkdyd0 Zero = dyd_Zero
mkdyd0 One  = dyd_One

mkdyd1 :: PREDEFS -> DyDigit -> DyDigit
mkdyd1 One  (_,_,a) = (1,0,a)
mkdyd1 Zero (_,_,a) = (0,0,a)
mkdyd1 Mone (_,_,a) = (-1,0,a)

mkdyd2 :: PREDEFS -> DyDigit -> DyDigit -> DyDigit
mkdyd2 One  (_,_,a) (_,_,b) = (1,0,max a b)
mkdyd2 Zero (_,_,a) (_,_,b) = (0,0,max a b)
mkdyd2 Mone (_,_,a) (_,_,b) = (-1,0,max a b)


-- ********** Constants **********
dyd_One           = ( 1,0,0) :: DyDigit
dyd_Half          = ( 1,1,0) :: DyDigit
dyd_Quarter       = ( 1,2,0) :: DyDigit
dyd_Eighth        = ( 1,3,0) :: DyDigit
dyd_Zero          = ( 0,0,0) :: DyDigit
dyd_minusHalf     = (-1,1,0) :: DyDigit
dyd_minusQuarter  = (-1,2,0) :: DyDigit
dyd_minusEighth   = (-1,3,0) :: DyDigit
dyd_minusOne      = (-1,0,0) :: DyDigit

-- ********** Constants **********
dyd_One'           = ( 1,0) :: DyDigit'
dyd_Half'          = ( 1,1) :: DyDigit'
dyd_Quarter'       = ( 1,2) :: DyDigit'
dyd_Eighth'        = ( 1,3) :: DyDigit'
dyd_Zero'          = ( 0,0) :: DyDigit'
dyd_minusHalf'     = (-1,1) :: DyDigit'
dyd_minusQuarter'  = (-1,2) :: DyDigit'
dyd_minusEighth'   = (-1,3) :: DyDigit'
dyd_minusOne'      = (-1,0) :: DyDigit'




-- ********** Unary Operations **********


-- dyd : Make dyadic digit
--       Checks for validity of digit.
dyd :: (Integral a) => (a,a) -> DyDigit
dyd (a,b) | b < 0        = undefined 
          | abs(a) > 2^b = undefined 
          | otherwise    = (toInteger a,toInteger b,0)   


-- Make dyadic digit with specified lookahead
dyd_Look :: DyDigit -> Int -> DyDigit
dyd_Look (a,b,_) n = (a,b,n)

-- Return lookahead of digit
dyd_LMax :: DyDigit -> Int
dyd_LMax (_,_,m) = m 

-- Return max lookahead of 2 digits
dyd_LMax2 :: DyDigit -> DyDigit -> Int
dyd_LMax2 (_,_,m) (_,_,n) = max m n

-- dydNegate : Negate digit
dydNegate :: DyDigit -> DyDigit
dydNegate (a,b, x) = (-a,b, x)


-- dydDouble : Double digit (May go out of range)
dydDouble :: DyDigit -> DyDigit
dydDouble (a,0, x) = (2*a,0, x)
dydDouble (a,b, x) = (a,b-1, x)

-- dydNormalise : Normalise dyadic digits
dydNormalise :: DyDigit -> DyDigit
dydNormalise (0,b,x) = (0,0,x)
dydNormalise (a,0,x) = (a,0,x)
dydNormalise (a,b,x) | odd a     = (a,b,x)
		     | otherwise = dydNormalise (a `div` 2, b - 1, x)


-- ********** Binary Arithmetic Operations **********

-- dydAv : Average two dyadic digits
--         Assume digits in normal form first.

dydAv :: DyDigit -> DyDigit -> DyDigit
dydAv (a,b,x) (c,d,y) | b == d    = dydNormalise (a + c,b + 1, max x y)
		      | b > d     = (a + 2^(b-d)*c, b+1, max x y)
		      | otherwise = (2^(d-b)*a + c, d+1, max x y)


-- dydAvNy : Average two dyadic digits x and y, negating y
--           Assume digits in normal form first.

dydAvNy :: DyDigit -> DyDigit -> DyDigit
dydAvNy (a,b,x) (c,d,y) | b == d    = dydNormalise (a - c,b + 1, max x y)
	  	        | b > d     = (a - 2^(b-d)*c, b+1, max x y)
		        | otherwise = (2^(d-b)*a - c, d+1, max x y)


-- dydAdd : Add two dyadic digits
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydAdd :: DyDigit -> DyDigit -> DyDigit
dydAdd (a,b,x) (c,d,y) | b == d      = dydNormalise (a + c,b, max x y) 
		       | b > d       = (a + 2^(b-d)*c, b, max x y)
		       | otherwise   = (2^(d-b)*a + c, d, max x y)



-- dydSub : Dyadic digits, subtract y from x
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydSub :: DyDigit -> DyDigit -> DyDigit
dydSub (a,b,x) (c,d,y) | b == d      = dydNormalise (a - c,b, max x y) 
		       | b > d       = (a - 2^(b-d)*c, b, max x y)
		       | otherwise   = (2^(d-b)*a - c, d, max x y)



-- dydAddRC : Dyadic digits, add x to y
--            Assume digits in normal form first.
--            Output result mod 1, and remainder.
dydAddRC :: DyDigit -> DyDigit -> (DyDigit, DyDigit)
dydAddRC x y = 
    if (a > two_e_b)  then ((mkdyd2 One x y), dydSub r (mkdyd2 One x y)) else
    if (a < -two_e_b) then ((mkdyd2 Mone x y), dydAdd r (mkdyd2 One x y)) 
	       else (r, (mkdyd2 Zero x y))
	 where {r@(a,b,_)   = dydAdd x y;
	        two_e_b   = 2^b}

-- dydSubRC : Dyadic digits, subtract y from x
--            Assume digits in normal form first.
--            Output result mod 1, and remainder.
dydSubRC :: DyDigit -> DyDigit -> (DyDigit, DyDigit)
dydSubRC x y = 
    if (a > two_e_b)  then ((mkdyd2 One x y), dydSub r (mkdyd2 One x y)) else
    if (a < -two_e_b) then ((mkdyd2 Mone x y), dydAdd r (mkdyd2 One x y)) 
	       else (r, (mkdyd2 Zero x y))
	 where {r@(a,b,_)   = dydSub x y;
	        two_e_b   = 2^b}


-- dydMul : Multiply two dyadic digits
--          Assume digits in normal form first.
dydMul :: DyDigit -> DyDigit -> DyDigit
dydMul (_,_,x) (0,_,y) = (0,0,max x y)
dydMul (0,_,x) (_,_,y) = (0,0,max x y)
dydMul (a,b,x) (c,d,y) = (a*c, b+d, max x y)



-- dydShr : Dyadic digit shift right (divide by 2^n)
--          Assume digits in normal form first.
dydShr :: (Integral a) => DyDigit -> a -> DyDigit
dydShr (a,b,x) n = (a,b + (toInteger n),x)


-- dydShl : Dyadic digit shift left (multiply by 2^n)
--          Assume digits in normal form first.
--          No range checking performed. Digits may go out of range
dydShl :: (Integral a) => DyDigit -> a -> DyDigit
dydShl (a,b,x) n = (a,b -(toInteger n),x)

-- ********** Relationships **********


-- dydEq : Test for equality of dyadic digits
--         Assume digits in normal form first.
dydEq :: DyDigit -> DyDigit -> Bool
dydEq (a,b,_) (c,d,_) = ((a==c) && (b==d))



-- dydGTE : Dyadic digit greater than or equal to: (a >= b)?
--          Assume digits in normal form first.
dydGTE :: DyDigit -> DyDigit -> Bool
dydGTE (a,b,_) (c,d,_) | b == d    = (a >= c)
 	               | b > d     = (a >= c*2^(b-d)) 
 	               | otherwise = (a*2^(d-b) >= c) 


-- dydGT : Dyadic digit strictly greater than: (a > b)?
--          Assume digits in normal form first.
dydGT :: DyDigit -> DyDigit -> Bool
dydGT (a,b,_) (c,d,_) | b == d    = (a > c)
 	              | b > d     = (a > c*2^(b-d)) 
 	              | otherwise = (a*2^(d-b) > c) 



-- dydLEQ : Dyadic digit less than or equal to: (a <= b)?
--          Assume digits in normal form first.
dydLEQ :: DyDigit -> DyDigit -> Bool
dydLEQ (a,b,_) (c,d,_) | b == d    = (a <= c)
 	               | b > d     = (a <= c*2^(b-d)) 
 	               | otherwise = (a*2^(d-b) <= c) 


-- dydLT : Dyadic digit strictly less than: (a < b)?
--          Assume digits in normal form first.
dydLT :: DyDigit -> DyDigit -> Bool
dydLT (a,b,_) (c,d,_) | b == d    = (a < c)
 	              | b > d     = (a < c*2^(b-d)) 
 	              | otherwise = (a*2^(d-b) < c) 




-- DyDigit' list -> DyDigit list ...
dydList' :: DyDigit -> [DyDigit'] -> [DyDigit]
dydList' (_,_,x) ((a,b):r) = ((a,b,x):dydList' (0,0,x+1) r)

dydList :: [DyDigit'] -> [DyDigit]
dydList = dydList' (0,0,0)




-- DyDigit' list -> DyDigit list ... constant lookahead
dydListCL' :: DyDigit -> [DyDigit'] -> [DyDigit]
dydListCL' (_,_,x) ((a,b):r) = ((a,b,x):dydListCL' (0,0,x) r)

dydListCL :: [DyDigit'] -> [DyDigit]
dydListCL = dydListCL' (0,0,0)


dydLMax :: DyDigit -> [DyDigit] -> [DyDigit]
dydLMax (_,_,x) ((a,b,y):r) | y >= x    = ((a,b,y):r)
			    | otherwise = ((a,b,x):dydLMax (0,0,x) r)


dyex_Lookahead :: [DyDigit] -> String
dyex_Lookahead ((_,_,a):x) = show a ++", "++(dyex_Lookahead x)