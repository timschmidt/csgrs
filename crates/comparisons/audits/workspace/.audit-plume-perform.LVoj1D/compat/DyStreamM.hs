module DyStreamM (
-- Types
DyStream,

-- Constants
dys_One, dys_Half,
dys_Third, dys_Quarter,
dys_Zero, 
dys_minusThird, dys_minusQuarter,
dys_minusHalf, dys_minusOne,


dys_OneL, dys_HalfL,
dys_ThirdL, dys_QuarterL,
dys_ZeroL, 
dys_minusThirdL, dys_minusQuarterL,
dys_minusHalfL, dys_minusOneL,


-- Unary Operations
dysNegate, dysShl, dysShr, 

-- Binary Arithmetic Operations
dysAv, dysAvNy, dysDigitMul, dysMul,  dysAdd, dysSub,

-- Misc
dysNorm, dysNormMax
) where

import DyDigitM


type DyStream' = [DyDigit']
type DyStream = [DyDigit]

-- Constants 
dys_One'           = (repeat dyd_One')                           :: DyStream'
dys_Half'          = ( dyd_One': repeat dyd_Zero')               :: DyStream'
dys_Third'         = ( dyd_One': dyd_minusOne': dys_Third')      :: DyStream'
dys_Quarter'       = (dyd_Zero': dys_Half')                      :: DyStream'
dys_Zero'          = (dyd_Zero': repeat dyd_Zero')               :: DyStream'
dys_minusQuarter'  = (dyd_Zero': dys_minusHalf')                 :: DyStream'
dys_minusThird'    = ( dyd_One': dyd_minusOne': dys_Third')      :: DyStream'
dys_minusHalf'     = (dyd_minusOne': dys_Zero')                  :: DyStream'
dys_minusOne'      = (dyd_minusOne': dys_minusOne')              :: DyStream'


dys_OneL           = dydList dys_One'		:: DyStream 
dys_HalfL          = dydList dys_Half' 		:: DyStream 
dys_ThirdL         = dydList dys_Third' 	:: DyStream 
dys_QuarterL       = dydList dys_Quarter' 	:: DyStream 
dys_ZeroL          = dydList dys_Zero' 		:: DyStream 
dys_minusQuarterL  = dydList dys_minusQuarter'  :: DyStream 
dys_minusThirdL    = dydList dys_minusThird'   	:: DyStream 
dys_minusHalfL     = dydList dys_minusHalf' 	:: DyStream 
dys_minusOneL      = dydList dys_minusOne'	:: DyStream 


dys_One           = dydListCL dys_One'		:: DyStream 
dys_Half          = dydListCL dys_Half' 	:: DyStream 
dys_Third         = dydListCL dys_Third' 	:: DyStream 
dys_Quarter       = dydListCL dys_Quarter' 	:: DyStream 
dys_Zero          = dydListCL dys_Zero' 	:: DyStream 
dys_minusQuarter  = dydListCL dys_minusQuarter' :: DyStream 
dys_minusThird    = dydListCL dys_minusThird'   :: DyStream 
dys_minusHalf     = dydListCL dys_minusHalf' 	:: DyStream 
dys_minusOne      = dydListCL dys_minusOne'	:: DyStream 



-- ********** Unary Operations **********

-- dysNegate : Negate a dyadic stream
dysNegate :: DyStream -> DyStream
dysNegate (a:x) = (dydNegate a:dysNegate x)



-- dysDigAdd : Add a digit to a stream (or emit -1,1 stream if not poss). 
dysDigAdd :: DyDigit -> DyStream -> DyStream

dysDigAdd dyd_Zero x = x
dysDigAdd d    (h:x) = (a:dysDigAdd b x)
     where (a,b) = dydAddRC d h


-- dysP : Adjust stream as if you had just emitted digit d from it.
dysP :: DyDigit -> DyStream -> DyStream

dysP d (h:x) = if (ba==0) then (dydLMax d x) else (a:dysDigAdd b x)
     where (a,b@(ba,_,_)) = dydSubRC h d



-- dysNorm : Normalise a stream
dysNorm :: DyStream -> (Integer, DyStream)
dysNorm x = dysNorm' x 0


-- dysNormMax : Normalise a stream with max
dysNormMax :: (Integral a) => DyStream -> a -> (Integer, DyStream)
dysNormMax x max = dysNormMax' x 0 max



-- dysNorm' : Normalise a stream, with counter 
dysNorm' :: DyStream -> Integer -> (Integer, DyStream)
dysNorm' x@(a:b:x') n = if (dx==0) then dysNorm' (c:x') (n+1) 
	                  else (n,dydLMax c x)
	where (c,(dx,_,_)) = dydAddRC (dydDouble a) b


-- dysNormMax' : Normalise a stream, with counter and max
dysNormMax' :: (Integral a) => DyStream -> Integer -> a -> (Integer, DyStream)
dysNormMax' x n 0            = (n,x)
dysNormMax' x@(a:b:x') n max = if (dx==0) then 
				 dysNormMax' (c:x') (n+1) max
		              else (n,dydLMax c x)
	where (c,(dx,_,_)) = dydAddRC (dydDouble a) b




-- dysShl : Shift a dyadic stream left by specified no. of digits
dysShl :: (Integral a) => DyStream -> a -> DyStream
dysShl x 0 = x
dysShl x n = dysShl (dysP dyd_Zero x) (n-1)


-- dysShr : Shift a dyadic stream right by specified no. of digits
dysShr :: (Integral a) => DyStream -> a -> DyStream
dysShr x 0 = x
dysShr x n = (dyd_Zero: dysShr x (n-1))



-- ********** Binary Operations **********

-- dysAv : Average two dyadic streams
dysAv :: DyStream -> DyStream -> DyStream 
dysAv (a:x) (b:y) = ((a `dydAv` b):dysAv x y)


-- dysAvNy : Average two dyadic streams x and y, negating y
dysAvNy :: DyStream -> DyStream -> DyStream 
dysAvNy (a:x) (b:y) = ((a `dydAvNy` b):dysAvNy x y)


-- dysAdd : Add two dyadic streams (overflow poss)
dysAdd :: DyStream -> DyStream -> DyStream 
dysAdd x y = dysShl (dysAv x y) 1

-- dysSub : Add two dyadic streams (overflow poss)
dysSub :: DyStream -> DyStream -> DyStream 
dysSub x y = dysShl (dysAvNy x y) 1

-- dysDigitMul : Multiply a dyadic stream by a dyadic digit
dysDigitMul :: DyDigit -> DyStream -> DyStream
dysDigitMul d (a:x) = (a `dydMul` d:dysDigitMul d x)


-- dysMul : Multiply two dyadic streams
dysMul :: DyStream -> DyStream -> DyStream
dysMul (a:x) (b:y) = dysAv (a `dydMul` b : dysMul x y) 
                           (dysAv (dysDigitMul b x) (dysDigitMul a y))


