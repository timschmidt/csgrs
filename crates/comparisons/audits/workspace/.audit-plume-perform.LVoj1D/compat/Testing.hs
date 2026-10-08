module Testing where
import Utils
import LookInt
import SBinStreamM



lmap x = sbShl (sbMul x (sbSub sbOneL x)) 2



it_lmap x 0 = x
it_lmap x n = lmap (it_lmap x (n-1))







