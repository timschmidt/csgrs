module BinarySplit where

import Control.Monad.Par hiding (parMap)
import Control.Parallel.Strategies
import Control.DeepSeq
import Data.Bits
import Data.Ratio

import Data.CDAR

divConq :: (a -> b)
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
                      where r | threshold arg = rseq
                              | otherwise     = rpar

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

fudge :: Approx -> Approx -> Approx
fudge (Approx m e s) (Approx m' e' s') =
    let m'' = 1 + (abs m' + e') `shift` (s' - s + 1)
    in Approx m (e+m'') s

{-
euler n =
    let t = n `div` 16
        pqs = parMap rpar pq $ zip [0,t..15*t] [t,2*t..16*t]
        [(p,q)] = contract . contract . contract . contract $ pqs
        a = (fromIntegral p :: Approx) / fromIntegral q
--        e = (1 :: Approx) / fromIntegral (q*(n+1))
    in boundErrorTerm $ a --fudge a e
-}

pairUp :: [a] -> [(a,a)]
pairUp [] = []
pairUp [a] = undefined
pairUp (a:b:as) = (a,b):pairUp as

contract :: [(Integer, Integer)] -> [(Integer, Integer)]
contract as = map {-parMap rpar-} (\(a,b) -> pqCombine a b) (pairUp as) --`using` evalList rpar

--{-
euler n =
    let (p,q) = divConq pq
                        (0,n)
                        (\(a,b) -> b-a < 100000)
                        pqCombine
                        (\(a,b) -> if b-a > 5000
                                   then let m = (a+b+1) `div` 2 in Just ((a,m), (m, b))
                                   else Nothing)
        a = (fromIntegral p :: Approx) / fromIntegral q
        e = (1 :: Approx) / fromIntegral (q*(n+1))
    in boundErrorTerm $ a --fudge a e
-- -}

pqToApprox :: (Integer,Integer) -> Approx
pqToApprox (p,q) =
  let a = (fromIntegral p :: Approx) / fromIntegral q
--      e = (1 :: Approx) / fromIntegral (q*(n+1))
  in boundErrorTerm a

main = print $ euler 300000 `seq` ()

test = pqToApprox $ divConq pq --(\(a,b) -> product [a..b-1])
                       (0,300000)
                       (\(a,b) -> b-a < 10000)
                       pqCombine
                       (\(a,b) -> if b-a > 1000
                                  then let m = (a+b) `div` 2 in Just ((a,m),(m,b))
                                  else Nothing)

----

                    -- ((pam, qam), (pmb, qmb)) = runPar $ do
                    --                              s <- new
                    --                              t <- new
                    --                              fork (put s (pq a m))
                    --                              fork (put t (pq m b))
                    --                              u <- get s
                    --                              v <- get t
                    --                              return (u,v)

{-
    let m = n `div` 4
        (a,e) = runPar $ do
                  s <- new
                  t <- new
                  u <- new
                  v <- new
                  fork (put s (pq 0 m))
                  fork (put t (pq (m+1) (2*m)))
                  fork (put u (pq (2*m+1) (3*m)))
                  fork (put v (pq (3*m+1) n))
                  (p0,q0) <- get s
                  (p1,q1) <- get t
                  (p2,q2) <- get u
                  (p3,q3) <- get v
                  x <- new
                  y <- new
                  z <- new
                  fork (put x (p0*q1+p1,q0*q1))
                  fork (put y (p2*q3+p3,q2*q3))
                  (p',q') <- get x
                  (p'',q'') <- get y
                  fork (put z (p'*q''+p'',q'*q''))
                  (p,q) <- get z
                  return (((fromIntegral p :: Approx) / fromIntegral q)
                         ,((1 :: Approx) / fromIntegral (q*(n+1))))
-}
