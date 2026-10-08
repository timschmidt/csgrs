-- module Logistic  where 

import System.Environment
import Data.Number.IReal
import Data.Number.IReal.IReal
import Data.Number.IReal.IntegerInterval
import Test.QuickCheck

{-

Iterating the logistic map for IReals.

iterLogistic c n k x0 computes the n'th iterate of logistic c starting from x0, to 
k decimals. It is crucial for efficiency to iterate (prec d . logistic c) rather than
logistic c itself. 

To get required precision, d must be at least n * log_10 c + k; this follows easily 
from estimating the derivative of logistic c.

main takes n and k as command line arguments and gives n'th iterate for selected values of c and x0.

-}

logistic c x = c * x * (1-x) 

iterLogistic c n k x0 = iterate (prec d . logistic c) x0 !! n
  where  d = fromInteger $ upperI (appr (fromIntegral n*logBase 10 c  + fromIntegral k) 0)

main = do ns : ks : _ <- getArgs
          let n = read ns
              k = read ks
          iterLogistic 3.75 n k (recip pi) ? k

-- Closed form solution for c=4
sol n x0 = (1 - cos (scale y0 n))/2
  where y0 =  acos (1 - 2 * x0)

propLogistic = forAll (uniformNum (0,1)) $ \x0 -> 
                 iterLogistic 4 100 100 x0 =?= sol 100 x0 `atDecimals` 100

