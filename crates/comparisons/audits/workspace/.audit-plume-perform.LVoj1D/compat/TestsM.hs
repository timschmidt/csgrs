module TestsM where

import SBinStreamM
import SBinFloatM
import DyFloatM
import DyStreamM
import DyDigitM
import LookInt


-- log_mapSBs' : Signed binary logistic map
log_mapSBs' :: SBinStream -> SBinStream

log_mapSBs' x = sbShl (sbMul x (sbSub3 sbOneL x)) 2
--log_mapSBs' x = sbShl (sbMul x (sbSub sbOneL x)) 2
 
-- log_mapSBs : Signed binary logistic map, iterated
log_mapSBs :: SBinStream -> Int -> SBinStream

log_mapSBs x 0 = x
log_mapSBs x n = log_mapSBs' (log_mapSBs x (n-1	))

t1' = [0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0] ++ [0, 0..] :: [Int]

t1 = ligilist t1'


-- extract_Lookahead (log_mapSBs a 5)


-- log_mapDys' : Signed binary logistic map
log_mapDys' :: DyStream -> DyStream


log_mapDys' x = (dysMul x (dysSub dys_OneL x))

 
-- log_mapDys : Signed binary logistic map, iterated
log_mapDys :: DyStream -> Int -> DyStream

log_mapDys x 0 = x
log_mapDys x n = log_mapDys' (log_mapDys x (n-1))

-- dyex_Lookahead (log_mapDys dys_HalfL 5)

-- ************************************************************


-- log_mapSbf' : Signed binary logistic map
log_mapSbf' :: SBinFloat -> SBinFloat

log_mapSbf' x = sbfShl (sbfMul x (sbfSub sbfOne x)) 2
 
-- log_mapSbf : Signed binary logistic map, iterated
log_mapSbf :: SBinFloat -> Int -> SBinFloat

log_mapSbf x 0 = x
log_mapSbf x n = log_mapSbf' (log_mapSbf x (n-1))

af = (0,a) :: SBinFloat

-- eg. 
-- extract_Lookahead (sbfMant (sbfNorm (log_mapSbf af 5)))



-- log_mapDyf' : Dyadic logistic map
log_mapDyf' :: DyFloat -> DyFloat

--log_mapDyf' x = dyfMul (dyfMul x (dyfSub dyf_One x)) (0, dys_One)
log_mapDyf' x = dyfShl (dyfMul x (dyfSub dyf_One x)) 2

 
-- log_mapDyf : Dyadic logistic map, iterated
log_mapDyf :: DyFloat -> Int -> DyFloat

log_mapDyf x 0 = x
log_mapDyf x n = log_mapDyf' (log_mapDyf x (n-1))


-- eg. 
-- extract_Lookahead ( (sbfNorm (log_mapDyf dyf_HalfL 5)))





stest x n = extract_Lookahead (log_mapSBs x n) 
--xtest x n = extract_Lookahead (log_mapX x n) 
dtest x n = dyex_Lookahead (log_mapDys x n) 
