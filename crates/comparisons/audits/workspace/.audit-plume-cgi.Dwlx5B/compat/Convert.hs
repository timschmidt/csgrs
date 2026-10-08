module Convert (

sbsToDys, dysToSbs,
sbfToDyf, dyfToSbf

  ) where

import SBinStream
import SBinFloat
import DyDigit
import DyStream
import DyFloat


-- sbsToDys : Convert signed binary stream -> dyadic stream
sbsToDys :: SBinStream -> DyStream
sbsToDys ( 1: x)  = (dyd_One      : sbsToDys x)
sbsToDys ( 0: x)  = (dyd_Zero     : sbsToDys x)
sbsToDys (-1: x)  = (dyd_minusOne : sbsToDys x)




-- dysToSbs : Convert dyadic stream -> signed binary stream
dysToSbs :: DyStream -> SBinStream
dysToSbs (a:b:x) 
	| dydGTE c dyd_Quarter      =
		 ( 1:dysToSbs (dydShl (c `dydSub` dyd_Half) 2: x))
 	| dydLEQ c dyd_minusQuarter = 
		 (-1:dysToSbs (dydShl (c `dydAdd` dyd_Half) 2: x))
	| otherwise  = (0:dysToSbs (dydShl c 2: x))
 	where c = (a `dydAv` (dydShr b 1))



dyfToSbf :: DyFloat -> SBinFloat
dyfToSbf (e,m) = (e, dysToSbs m)



sbfToDyf :: SBinFloat -> DyFloat
sbfToDyf (e,m) = (e, sbsToDys m)











{- Demo function for performing multiplication of two SbinStreams and 
   getting the result as a DyS.


xsbMul :: SbinStream -> SbinStream -> DyS
xsbMul x y = DyS (xsbMul' x y)

xsbDigitMul 1    x = sbToDy x
xsbDigitMul 0    x = dys_Zero
xsbDigitMul (-1) x = sbToDy (sbNegate x)

xsbMul'  (0:x)  (0:y) = (dyadic_Zero:dyadic_Zero:xsbMul' x y)
xsbMul'  (0:x)     y  = (dyadic_Zero:xsbMul' x y)
xsbMul'     x   (0:y) = (dyadic_Zero:xsbMul' x y)
xsbMul'  (1:x)  (1:y) = dysAv' (dyadic_One:xsbMul' x y)
                          (dysAv' (sbToDy' x) (sbToDy' y))
xsbMul' (-1:x) (-1:y) = dysAv' (dyadic_One:xsbMul' x y)
                        (dysAv' (sbToDy' (sbNegate x)) (sbToDy' (sbNegate y)))
xsbMul' (-1:x)  (1:y) = dysAv' (dyadic_minusOne:xsbMul' x y)
                        (dysAv' (sbToDy' x) (sbToDy' (sbNegate y)))
xsbMul'  (1:x) (-1:y) = dysAv' (dyadic_minusOne:xsbMul' x y)
                          (dysAv' (sbToDy' (sbNegate x)) (sbToDy' y))




--sbMul x y = dyToSb (xsbMul x y)

-}




