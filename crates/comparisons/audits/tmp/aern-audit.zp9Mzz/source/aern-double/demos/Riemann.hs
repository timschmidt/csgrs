
module Main where

import Numeric.AERN.DoubleBasis.RealIntervalApprox

import Control.Monad.ST (ST, runST)

type R = RealIntervalApprox
type RI = RealIntervalApprox

main =
  do
  putStrLn $ "riemann 0.1 (\\x -> x^2) (Interval 0 1) = " ++
    show (riemann 0.1 (\x -> x^2) (0 </\> 1))
  putStrLn "erf e x = 2/(sqrt pi) * riemann e (\\t -> exp (-t^2)) (0 </\\> x)"
  putStrLn $ "erf 0.001 1 = " ++ show (erf 0.001 1) 

-- compute the error function to within e accuracy
erf :: R -> R -> R
erf e x =
  2/(sqrt piOut) * riemann e (\t -> exp (-t^2)) (0 </\> x)

-- compute the integral of f over d to within e accuracy
riemann :: R -> (R -> R) -> RI -> R
riemann e f d =
  riemann' e f (width d) [d] 0

riemann' _ _ _ []     result = result 
riemann' e f initWidth (d:ds) result
  | reachedSufficientAccuracy =
      riemann' e f initWidth ds (dArea + result)
  | otherwise = 
      riemann' e f initWidth (dl:dr:ds) result
  where
  reachedSufficientAccuracy =
    case dError <? e * dWidth / initWidth of
      Just True -> True
      _ -> False
  dError = dWidth * (width fd)
  dArea = dWidth * fd
  fd = f d
  dWidth = width d
  (dl, dr) = bisect Nothing d

