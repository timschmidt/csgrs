module DyFloat (

-- Types
DyFloat,


-- Constants
dyf_One, dyf_Half, dyf_Third, dyf_Quarter,
dyf_Zero,
dyf_minusQuarter, dyf_minusThird, dyf_minusHalf,  dyf_minusOne,


-- Unary Operations
dyf,
dyfNegate, 
dyfShl, dyfShr,


-- Binary Arithmetic Operations
dyfAv, dyfAvNy,
dyfAdd, dyfSub,
dyfMul,


-- Misc
dyfNorm, dyfNormAllowNegExp, dyfNormNoNegExp
) where


import DyDigit
import DyStream



-- Types

type DyFloat = (Integer, DyStream)    --    (Exponent, Mantissa)


-- Constants

dyf_One            = (1, dys_Half)         :: DyFloat
dyf_Half           = (0, dys_Half)	   :: DyFloat
dyf_Third          = (0, dys_Third)	   :: DyFloat
dyf_Quarter        = (0, dys_Half)	   :: DyFloat
dyf_Zero           = (0, dys_Zero)         :: DyFloat
dyf_minusQuarter   = (0, dys_minusHalf)	   :: DyFloat
dyf_minusThird     = (0, dys_minusThird)   :: DyFloat
dyf_minusHalf      = (0, dys_minusHalf)	   :: DyFloat
dyf_minusOne       = (1, dys_minusHalf)    :: DyFloat



-- ********** Unary Operations **********

-- dyf : Make a dyadic float from mantissa, exponent
dyf :: (Integral a) => (a, DyStream) -> DyFloat
dyf (e,m) = (toInteger e, m)


-- dyfNegate : Negate a dyadic float
dyfNegate :: DyFloat -> DyFloat
dyfNegate (e,m) = (e, dysNegate m)



-- dyfShl : Dyadic float shift left
dyfShl :: (Integral a) => DyFloat -> a -> DyFloat
dyfShl (e,m) n = (e + (toInteger n), m)


-- dyfShr : Dyadic float shift right
dyfShr :: (Integral a) => DyFloat -> a -> DyFloat
dyfShr (e,m) n = (e - (toInteger n), m)





-- ********** Binary Arithmetic Operations **********


-- dyfMatchExps : Match the exponents of two dyadic floats by
--                shifting the mantissa of the smaller one right 
--                by n places, and adding that to it's exponent
dyfMatchExps :: DyFloat -> DyFloat -> (DyFloat, DyFloat)
dyfMatchExps x@(ex, mx) y@(ey,my)  
	  | ex == ey   = (x,y)
	  | ex > ey    = (x,(ex, dysShr my (ex - ey)))
	  | otherwise  = ((ey, dysShr mx (ey - ex)), y) 




-- dyfAv : Average two dyadic floats
dyfAv :: DyFloat -> DyFloat -> DyFloat
dyfAv x y = (ex', dysAv mx' my')
	where ((ex',mx'),(ey',my')) = dyfMatchExps x y



-- dyfNyAv : Average two dyadic floats x and y, negating y first.
dyfAvNy :: DyFloat -> DyFloat -> DyFloat
dyfAvNy x y = (ex', dysAvNy mx' my')
	where ((ex',mx'),(ey',my')) = dyfMatchExps x y



-- dyfAdd : Add two dyadic floats
dyfAdd :: DyFloat -> DyFloat -> DyFloat 
dyfAdd x y = (ex'+1, dysAv mx' my')
	where ((ex',mx'),(ey',my')) = dyfMatchExps x y



-- dyfSub : Subtract a dyadic float y from another x
dyfSub :: DyFloat -> DyFloat -> DyFloat 
dyfSub x y = (ex'+1, dysAvNy mx' my')
	where ((ex',mx'),(ey',my')) = dyfMatchExps x y


-- dyfMul : Multiply two dyadic floats
dyfMul :: DyFloat -> DyFloat -> DyFloat
dyfMul (ex,mx) (ey,my) = (ex+ey, dysMul mx my)




-- dyfNorm
dyfNorm = dyfNormAllowNegExp

-- dyfNormAllowNegExp : Normalise, allowing negative exponent
dyfNormAllowNegExp :: DyFloat -> DyFloat

dyfNormAllowNegExp (e, m) = (e-s, m')
   where (s,m') = dysNorm m    -- Neg. Exp. allowed, could set max.



-- dyfNormNoNegExp : Normalise but don't allow negative exponent
dyfNormNoNegExp :: DyFloat -> DyFloat

dyfNormNoNegExp (e, m) = (e-s, m')
   where (s,m') = dysNormMax m e   -- No Neg exp



