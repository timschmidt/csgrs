module SBinFloatM (

-- Types
SBinFloat,

-- Constants
sbfTen, sbfFour, sbfTwo,   
sbfZero, sbfOne, sbfminusOne,
sbfHalf, sbfminusHalf,
sbfThird, sbfminusThird, 
sbfEighth, sbfminusEighth,

sbfTenL, sbfFourL, sbfTwoL,   
sbfZeroL, sbfOneL, sbfminusOneL,
sbfHalfL, sbfminusHalfL,
sbfThirdL, sbfminusThirdL, 
sbfEighthL, sbfminusEighthL,


sbfSignum, sbfNegate, sbfAbs, sbfAbsSign,  
sbfAv, sbfAdd, sbfSub, sbfMul, sbfIntDiv,
sbfMin, sbfMax,
sbfShr, sbfShl,
sbfNorm, sbfNormMax, sbfNormNoNegExp, sbfNormAllowNegExp,
sbfNormAllowNegExpExtra, sbfNormMaxOrExp_lte_0,
sbfMant, sbfExp

)	where

import Utils
import LookInt
import SBinStreamM


type SBinFloat = (Integer, SBinStream)

sbfTen          = (4, sbTen)             :: SBinFloat
sbfFour         = (2, sbHalf)            :: SBinFloat
sbfTwo          = (2, sbHalf)            :: SBinFloat

sbfZero         = (0, sbZero)            :: SBinFloat
sbfOne          = (1, sbHalf)            :: SBinFloat
sbfminusOne     = (1, sbminusHalf)       :: SBinFloat

sbfHalf         = (0, sbHalf)            :: SBinFloat
sbfminusHalf    = (0, sbminusHalf)       :: SBinFloat

sbfThird        = (0, sbThird)           :: SBinFloat
sbfminusThird   = (0, sbminusThird)      :: SBinFloat

sbfQuarter      = (-1, sbHalf)           :: SBinFloat
sbfminusQuarter = (-1, sbminusHalf)      :: SBinFloat

sbfEighth       = (-2, sbHalf)           :: SBinFloat
sbfminusEighth  = (-2, sbminusHalf)      :: SBinFloat


sbfTenL          = (4, sbTenL)             :: SBinFloat
sbfFourL         = (2, sbHalfL)            :: SBinFloat
sbfTwoL          = (2, sbHalfL)            :: SBinFloat

sbfZeroL         = (0, sbZeroL)            :: SBinFloat
sbfOneL          = (1, sbHalfL)            :: SBinFloat
sbfminusOneL     = (1, sbminusHalfL)       :: SBinFloat

sbfHalfL         = (0, sbHalfL)            :: SBinFloat
sbfminusHalfL    = (0, sbminusHalfL)       :: SBinFloat

sbfThirdL        = (0, sbThirdL)           :: SBinFloat
sbfminusThirdL   = (0, sbminusThirdL)      :: SBinFloat

sbfQuarterL      = (-1, sbHalfL)           :: SBinFloat
sbfminusQuarterL = (-1, sbminusHalfL)      :: SBinFloat

sbfEighthL       = (-2, sbHalfL)           :: SBinFloat
sbfminusEighthL  = (-2, sbminusHalfL)      :: SBinFloat



-- sbfSignum : Sign of result to a finite number of digits.
sbfSignum :: (Integral a) => SBinFloat -> a -> Int
sbfSignum  (e,m) n  = sbSignum m n


sbfNegate (e,m) = (e, sbNegate m)

sbfOrderByExp (d1@(e1,m1)) (d2@(e2,m2)) = if (e1>e2) then (d2,d1) else (d1,d2)


-- sbfExpCorrect : Takes two SBinFloats, and returns a three tuple
--                 with the two floats represented using the same
--                 exponent, and the value of that exponent
sbfExpCorrect :: SBinFloat -> SBinFloat -> (SBinFloat, SBinFloat, Integer)
sbfExpCorrect  (d1@(e1,m1)) (d2@(e2,m2)) =
	if (e1>e2) then (d1,(e2,(sbShr m2 diff)), e1)
                   else ((e1,(sbShr m1 diff)),d2, e2)
     where diff = abs (e2-e1)



sbfAv sbf1 sbf2 = if (diff==0) then (e2, sbAv m1 m2) 
                else (e2, sbAv (sbShr m1 diff) m2)
      where {((e1,m1),(e2,m2)) = sbfOrderByExp sbf1 sbf2;
             diff = e2-e1}



-- sbfAdd : Signed binary float addition
sbfAdd :: SBinFloat -> SBinFloat -> SBinFloat

sbfAdd sbf1 sbf2 = (maxexp+1, sbAv m1 m2)
      where ((_,m1),(_,m2),maxexp) = sbfExpCorrect sbf1 sbf2


-- sbfSub : Signed binary float subtraction
sbfSub :: SBinFloat -> SBinFloat -> SBinFloat

sbfSub sbf1 sbf2 = (maxexp+1, sbAvNy m1 m2)
      where ((_,m1),(_,m2),maxexp) = sbfExpCorrect sbf1 sbf2



{-
-- Don't think this is any better.
sbfMul (e1,m1) (e2,m2) = if (new_e > 250) then sbfNorm (e1+e2,sbMul m1 m2)
                                         else (e1+e2, sbMul m1 m2)
      where new_e = e1+e2
-}

sbfMul (e1,m1) (e2,m2) | e1+e2 < 500 = (e1 + e2, sbMul m1 m2)
sbfMul (e1,m1) (e2,m2) = sbfNorm (e1 + e2, sbMul m1 m2)


sbfMax :: SBinFloat -> SBinFloat -> SBinFloat
sbfMax x y = (maxexp, sbMax m1 m2)
      where ((_,m1),(_,m2),maxexp) = sbfExpCorrect x y

sbfMin :: SBinFloat -> SBinFloat -> SBinFloat
sbfMin x y = (maxexp, sbMin m1 m2)
      where ((_,m1),(_,m2),maxexp) = sbfExpCorrect x y




-- sbfAbs : Returns absolute value
sbfAbs (e,m) = (e, sbAbs m)

-- sbfAbsSign : Returns absolute value and sign of original value
sbfAbsSign (e,m) = (s,(e,m')) where (s,m') = sbAbsSign m


sbfMant x = snd x
sbfExp  x = fst x


sbfIntDiv :: SBinFloat -> Lookint -> SBinFloat
-- Do division by powers of two using exponent.
sbfIntDiv x     d | d == 1           = x 
sbfIntDiv (e,m) d | (d `mod` 2) == 0 = sbfIntDiv (e-1, m) (d `div` 2)
		  | otherwise        = (e, sbIntDiv m d)



sbfShr :: SBinFloat -> Integer -> SBinFloat
sbfShr (e,m) n = (e-n, m)

sbfShl :: SBinFloat -> Integer -> SBinFloat
sbfShl (e,m) n = (e+n, m)



-- Normalisation:
--
--  Should have option to normalise to some default max so won't loop
--  trying to normalise zero ...


-- sbfNorm : Shorthand for normalising, allowing neg exp.
sbfNorm = sbfNormAllowNegExp



-- sbfNormAllowNegExp : Normalise, allowing negative exponent
sbfNormAllowNegExp :: SBinFloat -> SBinFloat

sbfNormAllowNegExp (e, m) = (e-s, m')
   where (s,m') = sbNorm m    -- Neg. Exp. allowed, could set max.




-- sbfNormMaxOrExp_lte_0 : Normalisation used by exponential function
--                           limits iterator. Normalises up to a specified
--		 	     maximum number of digits provided the 
--			     exponential <= 0. If max. reached and e > 0,
--			     then continue until e=0.
--                         Note: Dodgy for very large e.
sbfNormMaxOrExp_lte_0 :: SBinFloat -> Int -> SBinFloat
sbfNormMaxOrExp_lte_0 x@(e,m) n = sbfNormMax x (max (fromInteger e) n)


-- sbfNormMax : Normalise with a maximum number of places
sbfNormMax :: SBinFloat -> Int -> SBinFloat
sbfNormMax (e, m) n = (e-s, m')
   where (s,m') = sbNormMax m n   


-- sbfNormNoNegExp : Normalise but don't allow negative exponent
sbfNormNoNegExp :: SBinFloat -> SBinFloat

sbfNormNoNegExp (e, m) = (e-s, m')
   where (s,m') = sbNormMax m e   -- No Neg exp


-- sbfNormAllowNegExpExtra : Normalise, allowing negative exponent,
--                detecting 1,-1 and -1,1 cases.
sbfNormAllowNegExpExtra :: SBinFloat -> SBinFloat

sbfNormAllowNegExpExtra (e, m) = (e-s, m')
   where (s,m') = sbNormExtra m    -- Neg. Exp. allowed, could set max.





-- *************************************************** --
-- Conversion sbin -> decimal.


-- Incidental ...
sbfIntPow :: SBinFloat -> Integer -> SBinFloat
sbfIntPow _ 0 = sbfOne 
sbfIntPow x 1 = x
sbfIntPow x n | even n    = sbfIntPow (sbfMul x x) (n `div` 2)
sbfIntPow x n | otherwise = sbfMul x (sbfIntPow (sbfMul x x) (n `div` 2))


