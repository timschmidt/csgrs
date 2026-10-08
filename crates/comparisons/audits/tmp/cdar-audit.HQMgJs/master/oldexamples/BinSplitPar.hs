module BinSplitPar where

import Control.DeepSeq
import Control.Monad.Par hiding (parMap)
import Data.Ratio

pqCombine :: (Integer, Integer) -> (Integer, Integer) -> (Integer, Integer)
pqCombine (pl, ql) (pr, qr) = (pl*qr+pr, ql*qr)

pq :: (Integer, Integer) -> (Integer, Integer)
pq (a, b)
    | d >   5 = let m = (a+b+1) `div` 2
                    pql = pq (a, m)
                    pqr = pq (m, b)
                in pqCombine pql pqr
    | otherwise = (sum $ scanl1 (*) [b,b-1..a+1], product [a+1..b])
  where d = b - a

main = print . (`deepseq` ()) . runPar $ do
  i1 <- new; i2 <- new; i3 <- new; i4 <- new
  fork (put i1 (pq (0,80000)))
  fork (put i2 (pq (80000,160000)))
  fork (put i3 (pq (160000,240000)))
  fork (put i4 (pq (240000,320000)))
  a1 <- get i1; a2 <- get i2; a3 <- get i3; a4 <- get i4
  return $ (a1,a2,a3,a4) --pqCombine (pqCombine a1 a2) (pqCombine a3 a4)
