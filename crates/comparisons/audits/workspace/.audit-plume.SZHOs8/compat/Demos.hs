module Demos (tabulate) where

import SBinFloat
import SBinDec


-- log_mapSB' : Signed binary logistic map
log_mapSB' :: SBinFloat -> SBinFloat

log_mapSB' x = sbfShl (sbfMul x (sbfSub sbfOne x)) 2

tabulate :: SBinFloat -> Int -> Int -> Int -> String
tabulate x d 0 i = (show i)++"  ->  "++(sbfDec x d)++"\n"++"... Done\n"
tabulate x d n i = (show i)++"  ->  "++(sbfDec x d)++"\n"++(tabulate x' d (n-1) (i+1))
	where x' = log_mapSB' x

