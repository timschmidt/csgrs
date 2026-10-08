module BinSplit where

import Control.Parallel.Strategies
import Control.DeepSeq

import Data.Ratio

divConq :: (NFData b) => (a -> b)
        -> a
        -> (a -> Bool)
        -> (b -> b -> b)
        -> (a -> Maybe (a,a))
        -> b
divConq f arg threshold conquer divide = go arg
    where
      go arg = 
          case divide arg of
            Nothing -> f arg
            Just (l0,r0) -> conquer l1 r1 `using` strat
                where
                  l1 = go l0
                  r1 = go r0
                  strat x = do r l1; r r1; return x
                      where r | threshold arg = rdeepseq
                              | otherwise     = rpar

pqCombine :: (Integer, Integer) -> (Integer, Integer) -> (Integer, Integer)
pqCombine (pl, ql) (pr, qr) = (pl*qr+pr, ql*qr)

pq :: (Integer, Integer) -> (Integer, Integer)
pq (a, b)  = (\t -> (sum t, last t)) $ scanl1 (*) [b,b-1..a+1]

euler :: Integer -> Rational
euler n =
    let (p,q) = divConq pq
                        (0,n)
                        (\(a,b) -> b-a < 10000)
                        pqCombine
                        (\(a,b) -> if b-a > 5
                                   then let m = (a+b+1) `div` 2 in Just ((a,m), (m, b))
                                   else Nothing)
    in p%q

main = print $ euler 320000 `seq` ()
