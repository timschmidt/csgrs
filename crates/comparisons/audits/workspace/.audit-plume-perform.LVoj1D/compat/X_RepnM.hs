module X_RepnM (xsbMul, xsbfMul, sbDiv, sbfDiv) where

import LookInt
import SBinStreamM
import SBinFloatM
import DyDigitM
import DyStreamM
import DyFloatM
import ConvertM
---


xsbMul x y = dysToSbs (dysMul  (sbsToDys x) (sbsToDys y))

xsbfMul (e1,m1) (e2,m2) | e1+e2 < 500 = (e1+e2, xsbMul m1 m2)
                        | otherwise   = sbfNorm (e1+e2, xsbMul m1 m2)



sbDiv :: SBinStream -> SBinStream -> DyStream

sbDiv _ ( 0:_)      = undefined
sbDiv _ ( 1: -1: _) = undefined
sbDiv _ (-1:  1: _) = undefined
sbDiv x y@(1:y') = sbDiv' x y
sbDiv x y@(-1:y') = sbDiv' (sbNegate x) (sbNegate y)



sbDiv_emit :: Int -> Int -> SBinStream -> SBinStream -> DyStream

sbDiv_emit a look x y 
	   | a == 4  = (dyd_Look dyd_One look : sbDiv' x y)
	   | a == 2  = (dyd_Look dyd_Half look : sbDiv' x y)
	   | a == 1  = (dyd_Look dyd_Quarter look : sbDiv' x y)
	   | a == 0  = (dyd_Look dyd_Zero look : sbDiv' x y)
	   | a == -1 = (dyd_Look dyd_minusQuarter look : sbDiv' x y)
	   | a == -2 = (dyd_Look dyd_minusHalf look : sbDiv' x y)
	   | a == -4 = (dyd_Look dyd_minusOne look : sbDiv' x y)

sbDiv' :: SBinStream -> SBinStream -> DyStream


sbDiv' x@(a:x'@(b:x'')) y 
       | (a == 0) =
	     (sbDiv_emit 0 a_look x' y)
       | (a == 1) && (b == -1) = 
	     (sbDiv_emit 0 b_look ( g 1 b_look : x'') y)
       | (a == -1) && (b == 1) = 
	     (sbDiv_emit 0 b_look ( g (-1) b_look : x'') y)
       | (a == 1) =
	 let {r@(c:r'@(d:r'')) = sbSub x y;
	      c_look = e c;
	      d_look = e2 c d}
	 in  case c of { 
	(-1) -> if (d==1) then  
	     (sbDiv_emit 2 d_look (g (-1) d_look :r'') y) else
  	     (sbDiv_emit 1 d_look (p 0 (sbSub x (g 0 d_look:y))) y);
	  0  -> sbDiv_emit 2 c_look r' y;
	  1  -> sbDiv_emit 4 c_look (p 0 (sbSub r y)) y} 
       | (a == -1) =
	 let {r@(c:r'@(d:r'')) = sbAdd x y;
	      c_look = e c;
	      d_look = e2 c d}
	 in  case c of { 
	  1 -> if (d== -1) then  
	     (sbDiv_emit (-2) d_look (g 1 d_look :r'') y) else
  	     (sbDiv_emit (-1) d_look (p 0 (sbAdd x (g 0 d_look:y))) y);
	  0  -> sbDiv_emit (-2) c_look r' y;
	(-1) -> sbDiv_emit (-4) c_look (p 0 (sbSub r y)) y} 
       where {a_look = e a;
	      b_look = e2 a b}



fixinput' (a:b:x) n 
	  | a == 0 = fixinput' (ligu b a:x)  (n+1)
	  | (a == 1) && (b == -1) = fixinput' (lig2 1 a b:x) (n+1)
	  | (a == -1) && (b == 1) = fixinput' (lig2 (-1) a b:x) (n+1)
	  | otherwise = ((a:b:x),n)


fixinput :: SBinStream -> (SBinStream, Integer)
fixinput x = fixinput' x 0



sbfDiv :: SBinFloat -> SBinFloat -> SBinFloat
sbfDiv (ex, mx) (ey,my) = dyfToSbf (dyf (ex-ey+ey_fix+2, sbDiv mx my'))
	where (my', ey_fix) = fixinput my
