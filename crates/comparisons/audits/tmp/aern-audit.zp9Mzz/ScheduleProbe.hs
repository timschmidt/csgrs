{-# LANGUAGE TypeFamilies #-}
module Main where

import Control.Monad (unless)
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.RefinementOrder as RefOrd

-- Exact singleton imprecision endpoints, independent of AERN's interval code.
newtype Result = Result Int deriving Show
instance HasImprecision Result where
    type Imprecision Result = Int
    type ImprecisionEffortIndicator Result = ()
    imprecisionDefaultEffort _ = ()
    imprecisionOfEff _ (Result n) = n
instance RefOrd.IntervalLike Int where
    type GetEndpointsEffortIndicator Int = ()
    type FromEndpointsEffortIndicator Int = ()
    getEndpointsDefaultEffort _ = ()
    fromEndpointsDefaultEffort _ = ()
    getEndpointsOutEff _ n = (n,n)
    fromEndpointsInEff _ (a,_) = a
    fromEndpointsOutEff _ (a,_) = a

ensure :: String -> Bool -> IO ()
ensure name ok = unless ok (error name)

main :: IO ()
main = do
    let effortOf (eff,_,_) = eff
        fixed = iterateUntilAccurate () 3 0 (\() -> Result 0)
        initial = iterateUntilAccurate (0::Int) 3 0 Result
    ensure "fixed effort returns no evaluation" (null fixed)
    ensure "simple iterator skips initial effort" (map effortOf initial == [1,2,3])
    putStrLn ("fixed-effort result count: " ++ show (length fixed))
    putStrLn ("simple iterator, already accurate initial effort 0: " ++ show initial)
    let greedy = iterateUntilAccurate2 (0::Int) 2 3 (-1) Result
    ensure "greedy iterator accepts worse results" (map effortOf greedy == [0,1,2])
    putStrLn ("coordinate iterator imprecision increases despite no improvement: " ++ show greedy)
    let zeroLimit = iterateUntilAccurate2 (0::Int) 0 3 (-1) Result
    ensure "zero limit still yields initial result" (length zeroLimit == 1)
    putStrLn "all deterministic scheduling observations reproduced"
