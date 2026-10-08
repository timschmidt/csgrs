import Control.Concurrent
import Control.Monad ()
import Criterion.Main

import Data.Bits
import Data.Ratio
import Data.CDAR

logMap :: (Fractional a) => Rational -> a -> a
logMap c x = (fromRational c)*x*(1-x)

logMap2 :: Approx -> Approx
logMap2 Bottom = Bottom
logMap2 (Approx m e s) =
    let one = bit (-s)
        t = abs (bit (-s-1) - m)
    in boundErrorTerm $
       if t >= e
       then Approx (m*(one-m)-e^2) (e * abs (one-2*m)) (2*s)
       else Approx (bit (-2*s - 2)) ((t + e)^2) (2*s)

orbit :: Fractional a => Rational -> [a]
orbit c = iterate (logMap c) (fromRational (1%8))

orbit2 :: Rational -> [CR]
orbit2 c = iterate (((fromRational c)*) . CR . fmap logMap2 . unCR) (fromRational (1%8))

suite :: [Benchmark]
suite = [
    bgroup "Logistic 4" [
      bench "Double" $ nf (!! 10000) (orbit 4 :: [Double])
    , bench "CR" $ nf (require 10 . (!! 10000)) (orbit 4 :: [CR])
    , bench "CR (2)" $ nf (require 10 . (!! 10000)) (orbit2 4 :: [CR])
    ],
    bgroup "Logistic 3.5" [
      bench "Double" $ nf (!! 10000) (orbit (7%2) :: [Double])
    , bench "CR" $ nf (require 10 . (!! 10000)) (orbit (7%2) :: [CR])
    , bench "CR (2)" $ nf (require 10 . (!! 10000)) (orbit2 (7%2) :: [CR])
    ],
    bgroup "Logistic 3" [
      bench "Double" $ nf (!! 10000) (orbit 3 :: [Double])
    , bench "CR" $ nf (require 10 . (!! 10000)) (orbit 3 :: [CR])
    , bench "CR (2)" $ nf (require 10 . (!! 10000)) (orbit2 3 :: [CR])
    ]
  ]

newSuite :: [Benchmark]
newSuite =
  [ bgroup "expappr"
    [ bench "double" $ nf exp (1 :: Double)
    , bench "B40" $ nf (expBinarySplittingA 40) 1
    , bench "T40" $ nf (expTaylorA 40) 1
    , bench "T'40" $ nf (expTaylorA' 40) 1
    , bench "CR40" $ nf (require 40 . expCR) 1
    , bench "B400" $ nf (expBinarySplittingA 400) 1
    , bench "T400" $ nf (expTaylorA 400) 1
    , bench "T'400" $ nf (expTaylorA' 400) 1
    , bench "CR400" $ nf (require 400 . expCR) 1
    , bench "B4000" $ nf (expBinarySplittingA 4000) 1
    , bench "T4000" $ nf (expTaylorA 4000) 1
    , bench "T'4000" $ nf (expTaylorA' 4000) 1
    , bench "CR4000" $ nf (require 4000 . expCR) 1
    ]
  , bgroup "logappr"
    [ bench "doubleLog" $ nf log (1.5 :: Double)
    , bench "40B" $ nf (logBinarySplittingA 40) (Approx 3 0 (-1))
    , bench "40T" $ nf (logTaylorA 40) (Approx 3 0 (-1))
    , bench "40agm" $ nf (logAgmA (-40)) (Approx 3 0 (-1))
    , bench "400B" $ nf (logBinarySplittingA 400) (Approx 3 0 (-1))
    , bench "400T" $ nf (logTaylorA 400) (Approx 3 0 (-1))
    , bench "400agm" $ nf (logAgmA (-400)) (Approx 3 0 (-1))
    , bench "4000B" $ nf (logBinarySplittingA 4000) (Approx 3 0 (-1))
    , bench "4000T" $ nf (logTaylorA 4000) (Approx 3 0 (-1))
    , bench "4000agm" $ nf (logAgmA (-4000)) (Approx 3 0 (-1))
    ]
  , bgroup "exp"
    [ bench "double" $ nf exp (1 :: Double)
    , bench "40" $ nf (require 40 . exp) 1
    , bench "400" $ nf (require 400 . exp) 1
    ]
  , bgroup "log"
    [ bench "double" $ nf log (2 :: Double)
    , bench "40" $ nf (require 40 . log) 2
    , bench "400" $ nf (require 400 . log) 2
    ]
  , bgroup "sin"
    [ bench "double" $ nf sin (1 :: Double)
    , bench "40" $ nf (require 40 . sin) 1
    , bench "40T" $ nf (sinTaylorA 40) 1
    , bench "40CR" $ nf (require 40 . sinCR) 1
    , bench "400" $ nf (require 400 . sin) 1
    , bench "400T" $ nf (sinTaylorA 400) 1
    , bench "400CR" $ nf (require 400 . sinCR) 1
--    , bench "4000" $ nf (require 4000 . sin) 1
    , bench "4000T" $ nf (sinTaylorA 4000) 1
    , bench "4000CR" $ nf (require 4000 . sinCR) 1
    ]
  , bgroup "cos"
    [ bench "double" $ nf cos (1 :: Double)
    , bench "40" $ nf (require 40 . cos) 1
    , bench "400" $ nf (require 400 . cos) 1
    ]
  , bgroup "atan"
    [ bench "double" $ nf atan (1 :: Double)
    , bench "40" $ nf (require 40 . atan) 1
    , bench "400" $ nf (require 400 . atan) 1
    ]
  , bgroup "pi"
    [ bench "double" $ nf (\_ -> pi :: Double) (1 :: Double)
    , bench "40" $ nf (\_ -> require 40 $ pi) 1
    , bench "400" $ nf (\_ -> require 400 $ pi) 1
    ]
  , env setupEnvPi $ \ ~(pi40,pi400,pi4000) ->
    bgroup "ccc2015"
    [ bench "pi D" $ nf (\_ -> pi :: Double) (1 :: Double)
    , bench "pi 40" $ nf (\_ -> require 40 $ pi) 1
    , bench "pi 400" $ nf (\_ -> require 400 $ pi) 1
    , bench "pi 4000" $ nf (\_ -> require 4000 $ pi) 1
    , bench "+ D" $ nf (\x -> x+x) (pi :: Double)
    , bench "+ 40" $ nf (\x -> x+x) pi40
    , bench "+ 400" $ nf (\x -> x+x) pi400
    , bench "+ 4000" $ nf (\x -> x+x) pi4000
    , bench "* D" $ nf (\x -> x*x) (pi :: Double)
    , bench "* 40" $ nf (\x -> x*x) pi40
    , bench "* 400" $ nf (\x -> x*x) pi400
    , bench "* 4000" $ nf (\x -> x*x) pi4000
    , bench "* D" $ nf (1/) (pi :: Double)
    , bench "* 40" $ nf (1/) pi40
    , bench "* 400" $ nf (1/) pi400
    , bench "* 4000" $ nf (1/) pi4000
    , bench "exp D" $ nf exp (1 :: Double)
    , bench "exp 40" $ nf (require 40 . exp) 1
    , bench "exp 400" $ nf (require 400 . exp) 1
    , bench "exp 4000" $ nf (require 4000 . exp) 1
    , bench "log D" $ nf log (2 :: Double)
    , bench "log 40" $ nf (require 40 . log) 2
    , bench "log 400" $ nf (require 400 . log) 2
    , bench "log 4000" $ nf (require 4000 . log) 2
    , bench "sin D" $ nf sin (1 :: Double)
    , bench "sin 40" $ nf (require 40 . sinCR) 1
    , bench "sin 400" $ nf (require 400 . sinCR) 1
    , bench "sin 4000" $ nf (require 4000 . sinCR) 1
    , bench "cos D" $ nf cos (1 :: Double)
    , bench "cos 40" $ nf (require 40 . cosCR) 1
    , bench "cos 400" $ nf (require 400 . cosCR) 1
    , bench "cos 4000" $ nf (require 4000 . cosCR) 1
    , bench "atan D" $ nf atan (1 :: Double)
    , bench "atan 40" $ nf (require 40 . atanCR) 1
    , bench "atan 400" $ nf (require 400 . atanCR) 1
    , bench "atan 4000" $ nf (require 4000 . atanCR) 1
    ]
  , env setupEnv $ \ ~(pi1,pi2) ->
    bgroup "elementary Approx"
    [ bench "+ double" $ nf (\x -> x+x) (pi :: Double)
    , bench "* double" $ nf (\x -> x*x) (pi :: Double)
    , bench "rec double" $ nf (1/) (pi :: Double)
    , bench "sqrt double" $ nf (sqrt) (pi :: Double)
    , bench "+ 50" $ nf (\x -> x+x) pi1
    , bench "* 50" $ nf (\x -> x*x) pi1
    , bench "rec 50" $ nf (recipA 50) pi1
    , bench "sqrt 50" $ nf (sqrtA 50) pi1
    , bench "log 50" $ nf (logA 50) pi1
    , bench "+ 1000" $ nf (\x -> x+x) pi2
    , bench "* 1000" $ nf (\x -> x*x) pi2
    , bench "rec 1000" $ nf (recipA 1000) pi2
    , bench "sqrt 1000" $ nf (sqrtA 1000) pi2
    , bench "sqrtRec 1000" $ nf (sqrtRecA 1000) pi2
    , bench "log 1000" $ nf (logA 1000) pi2
    ]
  ]

setupEnv :: IO (Approx, Approx)
setupEnv = return . (\a -> (limitAndBound 50 a, a)) . limitAndBound 1000 . require 1000 $ pi
setupEnvPi :: IO (Approx, Approx, Approx)
setupEnvPi = return . (\a -> (limitAndBound 40 a, limitAndBound 400 a, limitAndBound 4000 a)) . require 4000 $ pi

threadSuite :: MVar Approx -> MVar Approx -> [Benchmark]
threadSuite u v =
  [ bgroup "thread"
    [ bench "communicate" $ nfIO (do putMVar u (Approx 145324626 123 (-30)); a <- takeMVar v; return a)
    ]
  ]

main :: IO ()
main = defaultMain $ newSuite -- ++ suite

{- Are threads making criterion confused, times seem ok, but reported as unreliable.

threadSuite :: MVar Approx -> MVar Approx -> [Benchmark]
threadSuite u v =
  [ bgroup "thread"
    [ bench "communicate" $ nfIO (do putMVar u pi2; a <- takeMVar v; return a)
    ]
  ]

main :: IO ()
main = do
  u <- newEmptyMVar
  v <- newEmptyMVar
  forkIO $ server u v
  defaultMain $ threadSuite u v
  where server u v = forever $ do
          a <- takeMVar u
          putMVar v (a + 1)
-}
