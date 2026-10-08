import Control.DeepSeq
import Control.Monad.Par hiding (parMap)
import Criterion.Main
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

t1 n = runPar $ do
  i1 <- new; i2 <- new; i3 <- new; i4 <- new
  fork (put i1 (pq (0,n1)))
  fork (put i2 (pq (n1,2*n1)))
  fork (put i3 (pq (2*n1,3*n1)))
  fork (put i4 (pq (3*n1,4*n1)))
  a1 <- get i1; a2 <- get i2; a3 <- get i3; a4 <- get i4
  return $ pqCombine (pqCombine a1 a2) (pqCombine a3 a4)
  where n1 = n `div` 4

suite =
  [ bgroup "BinSplitPar"
    [ bench "parallel" $ nf t1 320000
    , bench "sequential" $ nf pq (0,320000)
    ]
  ]

main = defaultMain suite
