{-# LANGUAGE NoMonomorphismRestriction #-}
--module Logistic where

import Data.CDAR
import Data.Bits
import Data.Ratio
import System.Environment (getArgs)
import Text.Printf

main :: IO ()
main = do
  args <- getArgs
  let n = read $ args !! 0 :: Int
  let c = if length args > 1 then
            let (i,f) = break (== '.') $ args !! 1
                f' = tail f
                l = length f'
            in (read (i++f') :: CR) / (10^l)
          else scale 1961 (-9)
  printf "Constant is %s.\n" $ showA . require 10 $ c  
  printf "\nComputing %d iterations of the map.\nIt may take some seconds...\n" n
  printf "\nAfter %d iterations the value is %s\n" n $ showA . limitSize 34 . require 34 $ (orbitC2 c)!!n

logMap :: (Fractional a) => a -> a
logMap x = 4*x*(1-x)

logMap2 :: Approx -> Approx
logMap2 Bottom = Bottom
logMap2 (Approx m e s) =
    let one = bit (-s)
        t = abs (bit (-s-1) - m)
    in boundErrorTerm $
       if t >= e
       then Approx (m*(one-m)-e^2) (e * abs (one-2*m)) (2*s)
       else Approx (bit (-2*s - 2)) ((t + e)^2) (2*s)

orbit :: Fractional a => [a]
orbit = iterate logMap (fromRational (1%8))

orbit1 :: [CR]
orbit1 = iterate (polynomial [0,4,4]) (fromRational (1%8))

orbit2 :: [CR]
orbit2 = iterate (CR . fmap ((4*) . logMap2) . unCR) (fromRational (1%8))

logMapC :: Fractional a => a -> a -> a
logMapC c x = c*x*(1-x)

orbitC :: CR -> [CR]
orbitC c = iterate (logMapC c) (1/8)

orbitC1 :: CR -> [CR]
orbitC1 c = iterate ((c*) . polynomial [0,1,-1]) (1/8)

orbitC2 :: CR -> [CR]
orbitC2 c = iterate ((c*) . CR . fmap logMap2 . unCR) (1/8)

{-

-- Using orbitC2:

jens@cspcjb:~/cdar> time stack exec logistic 10 3.82    
Constant is 3.82000000000000000000000~.

Computing 10 iterations of the map.
It may take some seconds...

After 10 iterations the value is 0.1650561779~

real    0m0.114s
user    0m0.094s
sys     0m0.022s
jens@cspcjb:~/cdar> time stack exec logistic 100 3.82
Constant is 3.82000000000000000000000~.

Computing 100 iterations of the map.
It may take some seconds...

After 100 iterations the value is 0.4309357855~

real    0m0.113s
user    0m0.087s
sys     0m0.033s
jens@cspcjb:~/cdar> time stack exec logistic 1000 3.82
Constant is 3.82000000000000000000000~.

Computing 1000 iterations of the map.
It may take some seconds...

After 1000 iterations the value is 0.8687524058~

real    0m0.121s
user    0m0.093s
sys     0m0.030s
jens@cspcjb:~/cdar> time stack exec logistic 10000 3.82
Constant is 3.82000000000000000000000~.

Computing 10000 iterations of the map.
It may take some seconds...

After 10000 iterations the value is 0.9216483032~

real    0m0.342s
user    0m0.307s
sys     0m0.037s
jens@cspcjb:~/cdar> time stack exec logistic 50000 3.82
Constant is 3.82000000000000000000000~.

Computing 50000 iterations of the map.
It may take some seconds...

After 50000 iterations the value is 0.2527618912~

real    0m6.753s
user    0m6.679s
sys     0m0.075s
jens@cspcjb:~/cdar> time stack exec logistic 100000 3.82
Constant is 3.82000000000000000000000~.

Computing 100000 iterations of the map.
It may take some seconds...

After 100000 iterations the value is 0.6195746474~

real    0m41.659s
user    0m41.501s
sys     0m0.159s

---

-- Using orbitC:

jens@cspcjb:~/cdar> time stack exec logistic 10 3.82    
Constant is 3.82000000000000000000000~.

Computing 10 iterations of the map.
It may take some seconds...

After 10 iterations the value is 0.1650561779~

real    0m0.110s
user    0m0.093s
sys     0m0.023s
jens@cspcjb:~/cdar> time stack exec logistic 100 3.82
Constant is 3.82000000000000000000000~.

Computing 100 iterations of the map.
It may take some seconds...

After 100 iterations the value is 0.4309357855~

real    0m0.113s
user    0m0.089s
sys     0m0.031s
jens@cspcjb:~/cdar> time stack exec logistic 1000 3.82
Constant is 3.82000000000000000000000~.

Computing 1000 iterations of the map.
It may take some seconds...

After 1000 iterations the value is 0.8687524058~

real    0m0.128s
user    0m0.081s
sys     0m0.049s
jens@cspcjb:~/cdar> time stack exec logistic 10000 3.82
Constant is 3.82000000000000000000000~.

Computing 10000 iterations of the map.
It may take some seconds...

After 10000 iterations the value is 0.9216483032~

real    0m1.044s
user    0m1.009s
sys     0m0.036s
jens@cspcjb:~/cdar> time stack exec logistic 50000 3.82
Constant is 3.82000000000000000000000~.

Computing 50000 iterations of the map.
It may take some seconds...

After 50000 iterations the value is 0.2527618912~

real    0m36.025s
user    0m35.848s
sys     0m0.176s
jens@cspcjb:~/cdar> time stack exec logistic 100000 3.82
Constant is 3.82000000000000000000000~.

Computing 100000 iterations of the map.
It may take some seconds...

After 100000 iterations the value is 0.6195746474~

real    3m47.789s
user    3m46.463s
sys     0m1.324s

---

model name      : Intel(R) Core(TM) i7-4790 CPU @ 3.60GHz

-}
