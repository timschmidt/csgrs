module IReal (
-- Types
SbInterval,
IReal,

-- Functions
sbfLimit, 
sbfBetween

) where

import Utils
import SBinStream
import SBinFloat


type SbInterval = (SBinFloat, SBinFloat)
type SbIntervalx = (SBinStream, SBinStream)

type IReal = [SbInterval]
type IRealx = [SbIntervalx]



-- sbBB : Signed binary stream `banana bracket' operation
--          (Simplified call. Could use general call instead).
sbBB :: SBinStream -> SBinStream -> SBinStream -> SBinStream
sbBB x@(a:x') y z@(c:z')
-- | r_lo > s_hi = undefined     -- Debug only   
 | (r_lo' >= -4) && (s_hi' <= 4) 
		 = ( 0: sbBB (p 0 x) (p 0 y) (p 0 z))
 | r_lo' >= 0    = ( 1: sbBB (p 1 x) (p 1 y) (p 1 z))
 | s_hi' <= 0    = (-1: sbBB (p (-1) x) (p (-1) y) (p (-1) z))
 | r_hi' < s_lo' = y
 | otherwise   = sbBB' 2 r' s' [a] [c] x' y z'
     where {r'@(r_lo', r_hi') = (-4 + a*4, 4 + a*4);
	    s'@(s_lo', s_hi') = (-4 + c*4, 4 + c*4)}


-- sbBB' : Signed binary stream `banana bracket' operation
--           (General call).
--         r and s represent the range of the lower and upper
--           bounds x and z from digits evaluated so far.
--           eg. [-8,0] -> [-1,0],  [2,6] -> [1/4,3/4].
--         sig represents the amount by which the next digit may
--           vary r and s.
--         outx and outz are digits from x and z seen so far.
--         x and z are lower and upper bounds (x <= z)
--         y is the number being `banana bracketed', (x <= y <= z).
--
--         The choice of [-8,8] to represent the range [-1,1] is
--           based on the fact that after examining 3 digits we
--           can either emit a digit, or determine that x <> z.
--         r' and s' are the new ranges of the lower/upper bounds
--           after the digits a and c are considered.
sbBB' :: Int -> (Int, Int) -> (Int, Int) -> [Int] -> [Int] ->
         SBinStream -> SBinStream -> SBinStream -> SBinStream      
sbBB' sig (r_lo,r_hi) (s_lo,s_hi) outx outz x@(a:x') y z@(c:z') 
-- | sig == 0    = undefined     -- Debug only
-- | r_lo > s_hi = undefined     -- Debug only   
 | (r_lo' >= -4) && (s_hi' <= 4) 
	      = ( 0: sbBB (p 0 (outx++x)) (p 0 y) (p 0 (outz++z)))
 | r_lo' >= 0 = ( 1: sbBB (p 1 (outx++x)) (p 1 y) (p 1 (outz++z)))
 | s_hi' <= 0 = (-1: sbBB (p (-1) (outx++x)) (p (-1) y) (p (-1) (outz++z)))
 | r_hi' < s_lo' = y
 | otherwise   = sbBB' (sig `div` 2) r' s' (outx++[a]) (outz++[c]) x' y z'

     where {r'@(r_lo', r_hi') = (r_lo + (a+1)*sig, r_hi + (a-1)*sig);
	    s'@(s_lo', s_hi') = (s_lo + (c+1)*sig, s_hi + (c-1)*sig)}




-- sbfBB : `Banana Bracket' operation on signed binary floats.
--          Note: exponent specified by parameter e, and absent from 
--                output.
sbfBB :: Integer -> SBinFloat -> SBinFloat -> SBinFloat -> SBinStream
sbfBB e (ex, mx) (ey,my) (ez, mz) = 
	sbBB (sbShift mx (e-ex)) (sbShift my (e-ey)) (sbShift mz (e-ez))



-- limitx : Convert a sequence of nested intervals in [-1,1] into a
--          signed binary stream
limitx :: IRealx -> SBinStream
limitx ((x, z): y) = sbBB x (limitx y) z


-- sbfLimit' : Convert a sequence of nested intervals into an SBinFloat
--             with specified exponent
sbfLimit' :: Integer -> IReal -> SBinFloat
sbfLimit' e ((x, z): y) = (e, sbfBB e x (sbfLimit' e y) z)

-- sbfLimit : Convert a sequence of nested intervals into an SBinFloat
--            by first finding max exponent required, and
--            then using sbfLimit'.
--            We use sbfNormMax with 500 (an arbitrary number) so that 
--            if we see zero it won't fall over. In fact it is only for
--            efficiency that we bother normalising at all, so this doesn't
--            really matter.
sbfLimit :: IReal -> SBinFloat
sbfLimit ((x, z): y) = sbfLimit' (max ex ez) ((x,z):y)
	where {(ex, _) = sbfNormMax x 500;
	       (ez, _) = sbfNormMax z 500}
		

{-
Take the exponent of the upper bound. Ideally the first and
subsequent approximations used should have upper and lower bounds with
the same exponent. But: 

1. Normalise everything first.
2. Use sbShift to correct mantissas so that they have the right significance.
-}

{- y :: SBinFloat
   Only need mantissa provided can specify exponent in recursive call.
-}



sbfBetween' :: Integer -> IReal -> SBinFloat
sbfBetween' e ((x, z): y) = 
	(e, sbfBB e a (sbfMin b (sbfMax a (sbfBetween' e y))) b)
	where {a   = sbfMin x z;
	       b   = sbfMax x z}


sbfBetween :: IReal -> SBinFloat
sbfBetween ((x, z): y) = sbfBetween' (ez) ((x,(ez,mz)):y)
	where (ez, mz) = sbfNorm z


		

