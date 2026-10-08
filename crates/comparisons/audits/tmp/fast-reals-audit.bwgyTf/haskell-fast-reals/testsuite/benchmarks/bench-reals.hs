{-# LANGUAGE BangPatterns #-}

-- |
{--
in rational        took    0.000573552881ms (2097151 eval in 1202.827ms)
in real            took    0.002196396248ms (524287 eval in 1151.542ms) -- but we guessed the prec
in irram way 1ms 

-}

module Main where

import Data.Approximate.ApproximateField
import Data.Ratio
import Data.Time.Clock
import Numeric

import Data.Approximate.Floating.MPFR
import Data.Reals.Reals

run :: (() -> a) -> Int -> Int
run f 0 = 0
run f n =
      let !z = f () in run f (n-1)

runAndMeasure :: UTCTime -> NominalDiffTime -> IO Int -> (() -> t) -> IO Int
runAndMeasure startTime timeLimit n f =
    do n2 <- n
       let !r = run f n2
       end <- getCurrentTime
       if diffUTCTime end startTime < timeLimit
           then do
                  n3 <- runAndMeasure startTime timeLimit (return $ 2 * n2) f
                  return $ n2 + n3
           else return n2

showTime start end n =
    showFFloat Nothing (fromRational . toRational $ diffUTCTime end start * 1000 / fromInteger ( toInteger n )) "ms"

measure f s =
  do
    start  <- getCurrentTime
    ncalls <- runAndMeasure start 1 (return 1) f
    end <- getCurrentTime
    putStrLn $ s ++ " took    " ++ showTime start end ncalls ++
      " (" ++ show ncalls ++ " eval in " ++ showTime start end 1 ++ ")"
    return ()


main = do
    --let !z = appLog s x

    let f x y = (333.75 - x^2)*y^6 + x^2*(11*x^2*y^2 - 121*y^4 - 2) + 5.5 * y^8 + x / (2*y)
    let z1 = f 77617 33096
    let z2 = f 77617 33096
    measure (\() -> z1 :: Ratio Integer)   "in rational      "

    measure (\() -> approximate (z2 :: RealNum) (prec RoundDown 5))   "in real      "
