import Criterion.Main

import Data.CDAR

suite :: [Benchmark]
suite =
    [ bgroup "exp 1"
      [ bench "200" $ whnf (require 200) (exp 1)
      , bench "2000" $ whnf (require 2000) (exp 1)
      , bench "20000" $ whnf (require 20000) (exp 1)
      ]
    , bgroup "exp ~1"
      [ bench "200" $ whnf (require 200) (exp (1 + BR (repeat (Approx 0 1 (-250)))))
      , bench "2000" $ whnf (require 2000) (exp (1 + BR (repeat (Approx 0 1 (-2050)))))
--      , bench "20000" $ whnf (require 20000) (exp $ 1 + (BR . repeat $ Approx 0 1 (-25000)))
      ]
    , bgroup "exp n"
      [ bench "200" $ whnf (require 200) (exp 200)
      , bench "2000" $ whnf (require 200) (exp 2000)
      , bench "20000" $ whnf (require 200) (exp 20000)
      ]
    ]

main :: IO ()
main = defaultMain suite
