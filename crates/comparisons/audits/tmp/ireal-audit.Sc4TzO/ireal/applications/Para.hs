
import Control.Parallel.Strategies
import Data.Number.IReal
 
{- 

Simple experiment in parallelism. 

sumFun d f a b computes sum (map f [a..b-1]) to d decimals by
- splitting into nrSparks separate sums over subranges, which are individually sparked.
- using function isumN' for summation over each range. 
- using a simple strategy to force enough precision in the computation of each spark.

Has only been tested on a dual-core machine, where speedup is almost perfect:

> time ./Para +RTS -N2
1.036927755143369926331355720881

real	0m0.768s
user	0m1.514s
sys	0m0.011s

Note: As an approximation of the infinite sum zeta(5), this is very naive (only 22 correct
decimals). See file Zeta.hs in this directory for a much more sophisticated approximation.

-}

nrSparks = 100

sumFun :: Int -> (Integer -> IReal) -> Integer -> Integer -> IReal
sumFun d f a b = isumN' nrSparks (zipWith s ks (tail ks) `using` parList (rseq . prec p))
   where s a b = isumN' (b-a) (map f [a..b-1])
         ks = [a +  i * (b-a) `div` nrSparks | i <- [0..nrSparks] ]
         p = d + ceiling (logBase 10 (fromIntegral nrSparks))

main = sumFun 31 (recip . fromInteger . (^5)) 1 400001 ? 30


