module Main where

import Criterion.Main
import Data.CReal.Internal

main :: IO ()
main = defaultMain [ bgroup "pi" [ bench "0"    $ whnf (pi `atPrecision`) 0
                                 , bench "4"    $ whnf (pi `atPrecision`) 4
                                 , bench "16"   $ whnf (pi `atPrecision`) 16
                                 , bench "64"   $ whnf (pi `atPrecision`) 64
                                 , bench "256"  $ whnf (pi `atPrecision`) 256
                                 , bench "1024" $ whnf (pi `atPrecision`) 1024
                                 ]
                   , bgroup "sin 1" [ bench "0"    $ whnf (sin 1 `atPrecision`) 0
                                    , bench "4"    $ whnf (sin 1 `atPrecision`) 4
                                    , bench "16"   $ whnf (sin 1 `atPrecision`) 16
                                    , bench "64"   $ whnf (sin 1 `atPrecision`) 64
                                    , bench "256"  $ whnf (sin 1 `atPrecision`) 256
                                    , bench "1024" $ whnf (sin 1 `atPrecision`) 1024
                                 ]
                   , bgroup "sin (π/4)" [ bench "0"    $ whnf (sin (pi/4) `atPrecision`) 0
                                        , bench "4"    $ whnf (sin (pi/4) `atPrecision`) 4
                                        , bench "16"   $ whnf (sin (pi/4) `atPrecision`) 16
                                        , bench "64"   $ whnf (sin (pi/4) `atPrecision`) 64
                                        , bench "256"  $ whnf (sin (pi/4) `atPrecision`) 256
                                        , bench "1024" $ whnf (sin (pi/4) `atPrecision`) 1024
                                 ]
                   , bgroup "asin (π/4)" [ bench "0"    $ whnf (asin (pi/4) `atPrecision`) 0
                                         , bench "4"    $ whnf (asin (pi/4) `atPrecision`) 4
                                         , bench "16"   $ whnf (asin (pi/4) `atPrecision`) 16
                                         , bench "64"   $ whnf (asin (pi/4) `atPrecision`) 64
                                         , bench "256"  $ whnf (asin (pi/4) `atPrecision`) 256
                                         , bench "1024" $ whnf (asin (pi/4) `atPrecision`) 1024
                                 ]
                                 ]

