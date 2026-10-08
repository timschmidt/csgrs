module X_Repn (xsbMul, xsbfMul, sbDiv, sbfDiv) where

import SBinStream
import SBinFloat
import DyDigit
import DyStream
import DyFloat
import Convert
---

-- xsbMul : Signed binary stream multiplication via dyadics.
xsbMul :: SBinStream -> SBinStream -> SBinStream
xsbMul x y = dysToSbs (dysMul  (sbsToDys x) (sbsToDys y))

-- xsbfMul : Signed binary float multiplication via dyadics.
xsbfMul :: SBinFloat -> SBinFloat -> SBinFloat
xsbfMul (e1,m1) (e2,m2) | e1+e2 < 500 = (e1+e2, xsbMul m1 m2)
                        | otherwise   = sbfNorm (e1+e2, xsbMul m1 m2)



-- sbDiv : Division of two (corrected) signed binary streams. This
--         function is used by the sbfDiv function. Note that negative
--         denominator streams result in the negation of both
--         denominator and numberator so the denominator is positive.
sbDiv :: SBinStream -> SBinStream -> DyStream

sbDiv x ( 1:y) = sbDiv' x (1:y)
sbDiv x (-1:y) = sbDiv' (sbNegate x) (1:sbNegate y)
sbDiv _ ( 0:_)      = undefined
sbDiv _ ( 1: -1: _) = undefined
sbDiv _ (-1:  1: _) = undefined


-- sbDiv_emit : Emit specified dyadic digit and make the appropriate
--              recursive call. This function makes the code more
--              readable. Used by sbDiv'.
sbDiv_emit :: Int -> SBinStream -> SBinStream -> DyStream

sbDiv_emit ( 4) x y = (dyd_One          : sbDiv' x y)
sbDiv_emit ( 2) x y = (dyd_Half         : sbDiv' x y)
sbDiv_emit ( 1) x y = (dyd_Quarter      : sbDiv' x y)
sbDiv_emit ( 0) x y = (dyd_Zero         : sbDiv' x y)
sbDiv_emit (-1) x y = (dyd_minusQuarter : sbDiv' x y)
sbDiv_emit (-2) x y = (dyd_minusHalf    : sbDiv' x y)
sbDiv_emit (-4) x y = (dyd_minusOne     : sbDiv' x y)


-- sbDiv' : The body of the division algorithm, used by sbDiv.
sbDiv' :: SBinStream -> SBinStream -> DyStream

sbDiv'   ( 1: -1:x') y = (sbDiv_emit 0 ( 1:x') y)
sbDiv'   (-1:  1:x') y = (sbDiv_emit 0 (-1:x') y)
sbDiv'   ( 0:x') y     = (sbDiv_emit 0 x' y)

sbDiv' x@( 1:x') y     = case a of { 
	(-1) -> if (a'==1) then  (sbDiv_emit 2 (-1:r'') y) else
  	                         (sbDiv_emit 1 (p 0 (sbSub x (0:y))) y);
	  0  -> sbDiv_emit 2 r' y;
	  1  -> if (a'== -1) then  (sbDiv_emit 2 (1:r'') y) else
				   (sbDiv_emit 4 (p 0 (sbSub r y)) y)}    

	where r@(a:r'@(a':r'')) = sbSub x y

sbDiv' x@(-1:x') y = case a of {
	  1  -> if (a'== -1) then  (sbDiv_emit (-2) (1:r'') y) else
  	                           (sbDiv_emit (-1) (p 0 (sbAdd x (0:y))) y);
	  0  -> sbDiv_emit (-2) r' y;
  	(-1) -> if (a'== 1) then  (sbDiv_emit (-2) (-1:r'') y) else
				  (sbDiv_emit (-4) (p 0 (sbAdd r y)) y)}
	
	where r@(a:r'@(a':r'')) = sbAdd x y




-- fixinput' : Auxiliary function used by fixinput.
--             Fixes a number so that it always start with a non-zero digit.
fixinput' :: SBinStream -> Integer -> SBinFloat
fixinput' ( 0:  x)    n = fixinput'     x  (n+1)
fixinput' ( 1: -1: x) n = fixinput' ( 1:x) (n+1)
fixinput' (-1:  1: x) n = fixinput' (-1:x) (n+1)
fixinput' x           n = (n,x)

fixinput :: SBinStream -> SBinFloat
fixinput x = fixinput' x 0


-- sbfDiv : Top level division function.
sbfDiv :: SBinFloat -> SBinFloat -> SBinFloat
sbfDiv (ex, mx) (ey,my) = dyfToSbf (dyf (ex-ey+ey_fix+2, sbDiv mx my'))
	where (ey_fix, my') = fixinput my
