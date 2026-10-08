import Control.Concurrent
import Control.Monad
import Criterion.Main

import Data.Bits
import Data.Ratio
import Data.CDAR

pi1 = limitAndBound 40 . require 40 $ pi
pi2 = limitAndBound 400 . require 400 $ pi
pi3 = limitAndBound 4000 . require 4000 $ pi

threadSuite :: MVar Approx -> MVar Approx -> [Benchmark]
threadSuite u v =
  [ bgroup "thread"
    [ bench "communicate40" $ nfIO (do putMVar u pi1; a <- takeMVar v; return a)
    , bench "communicate400" $ nfIO (do putMVar u pi2; a <- takeMVar v; return a)
    , bench "communicate4000" $ nfIO (do putMVar u pi3; a <- takeMVar v; return a)
    ]
  , bgroup "nonthread"
    [ bench "40" $ nf (sqrA) pi1
    , bench "400" $ nf (sqrA) pi2
    , bench "4000" $ nf (sqrA) pi3
    ]
  ]

main :: IO ()
main = do
  u <- newEmptyMVar
  v <- newEmptyMVar
  forkOS $ server u v
  defaultMain $ threadSuite u v
  where server u v = forever $ do
          a <- takeMVar u
          putMVar v (sqrA a)
