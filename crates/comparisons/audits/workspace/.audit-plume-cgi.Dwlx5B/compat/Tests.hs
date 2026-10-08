module Tests (tabulate) where

import SBinStream
import SBinFloat
import SBinDec
import DyStream
import DyFloat
import Convert
import X_Repn


-- log_mapSB' : Signed binary logistic map
log_mapSB' :: SBinFloat -> SBinFloat

log_mapSB' x = sbfShl (sbfMul x (sbfSub sbfOne x)) 2

-- log_mapSB : Signed binary logistic map, iterated
log_mapSB :: (Integral a) => SBinFloat -> a -> SBinFloat

log_mapSB x 0 = x
log_mapSB x n = log_mapSB' (sbfNorm (log_mapSB x (n-1)))




-- log_mapX' : Cross representation logistic map
log_mapX' :: SBinFloat -> SBinFloat
log_mapX' x =  sbfShl (xsbfMul x (sbfSub sbfOne x)) 2

-- log_mapX : Signed binary logistic map, iterated
log_mapX :: (Integral a) => SBinFloat -> a -> SBinFloat

log_mapX x 0 = x
log_mapX x n = log_mapX' (sbfNorm (log_mapX x (n-1)))




-- log_mapDy : Dyadic logistic map
log_mapDy' :: DyFloat -> DyFloat

log_mapDy' x = dyfShl (dyfMul x (dyfSub dyf_One x)) 2

-- log_mapSB : Signed binary logistic map, iterated
log_mapDy :: (Integral a) => DyFloat -> a -> DyFloat

log_mapDy x 0 = x
log_mapDy x n = log_mapDy' (dyfNorm (log_mapDy x (n-1)))

-- log_mapSB : Signed binary logistic map, iterated
log_mapDyWrap :: (Integral a) => SBinFloat -> a -> SBinFloat

log_mapDyWrap x n = sbfNorm (dyfToSbf (log_mapDy (sbfToDyf x) n))


-- Shortcuts

stest x n d = sbfDec (log_mapSB x n) d
xtest x n d = sbfDec (log_mapX x n) d
dtest x n d = sbfDec (log_mapDyWrap x n) d

stest' = log_mapSB
xtest' = log_mapX
dtest' = log_mapDyWrap

------------------------------------------------------------

-- Test values

a = decSbf "0.5467"
b = decSbf "0.1"

a' = sbfToDyf a
b' = sbfToDyf b



------------------------------------------------------------


tabulate :: SBinFloat -> Int -> Int -> Int -> String
tabulate x d 0 i = (show i)++"  ->  "++(sbfDec x d)++"\n"++"... Done\n"
tabulate x d n i = (show i)++"  ->  "++(sbfDec x d)++"\n"++(tabulate x' d (n-1) (i+1))
	where x' = log_mapSB' x






-- log_mapSBs' : Signed binary logistic map
log_mapSBs' :: SBinStream -> SBinStream

log_mapSBs' x = sbShl (sbMul x (sbSub sbOne x)) 2

-- log_mapSBs : Signed binary logistic map, iterated
log_mapSBs :: (Integral a) => SBinStream -> a -> SBinStream

log_mapSBs x 0 = x
log_mapSBs x n = log_mapSBs' (log_mapSBs x (n-1	))




-- log_mapXs' : Cross representation logistic map
log_mapXs' :: SBinStream -> SBinStream
log_mapXs' x =  sbShl (xsbMul x (sbSub sbOne x)) 2

-- log_mapXs : Signed binary logistic map, iterated
log_mapXs :: (Integral a) => SBinStream -> a -> SBinStream

log_mapXs x 0 = x
log_mapXs x n = log_mapXs' (log_mapXs x (n-1))




-- log_mapDys' : Dyadic logistic map
log_mapDys' :: DyStream -> DyStream

log_mapDys' x = dysShl (dysMul x (dysSub dys_One x)) 2

-- log_mapDys : Signed binary logistic map, iterated
log_mapDys :: (Integral a) => DyStream -> a -> DyStream

log_mapDys x 0 = x
log_mapDys x n = log_mapDys' (log_mapDys x (n-1))

-- log_mapDysWrap : 
log_mapDysWrap :: (Integral a) => SBinStream -> a -> SBinStream

log_mapDysWrap x n = (dysToSbs (log_mapDys (sbsToDys x) n))


