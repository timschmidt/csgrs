module Main where

-- the MPFR interval type:
import qualified Numeric.AERN.MPFRBasis.Interval as MI

-- real arithmetic operators and imprecision measure:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators ((|*))
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (RoundedReal)
import Numeric.AERN.RealArithmetic.Measures (imprecisionOf, iterateUntilAccurate)

-- ability to change precision of numbers:
import Numeric.AERN.Basics.SizeLimits (changeSizeLimitsOut)

-- generic tools for controlling formatting of variable precision approximations:
import Numeric.AERN.Basics.ShowInternals (showInternals)


import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))
import System.Environment (getArgs)

type RealApprox = MI.MI -- MPFR interval
type Precision = MI.Precision

usage :: String
usage = "Usage: LogisticMap <number of iterations> <result digits>"

processArgs :: [String] -> (Int, Int)
processArgs [itersS, digitsS] =
    case (reads itersS, reads digitsS) of
        ((iters, []):_,(digits, []):_) -> (iters, digits)
        _ -> error $ usage
processArgs _ = error $ usage

r :: Rational
r = 375 / 100

x0 :: Rational
x0 = 0.5 -- 0.671875

main =
    do
    -- set console to print each line asap:
    hSetBuffering stdout LineBuffering
    -- process command line arguments:
    args <- getArgs
    let (iters, digits) = processArgs args 

    -- compute and print the result for each precision:
    mapM_ (reportAttempt digits) $ attemptsWithIncreasingPrecision iters digits
    
    where
    attemptsWithIncreasingPrecision iters digits =
        -- invoke an iRRAM-style procedure for automatic precision/effort incrementing 
        --    on the computation of iters-many iterations of the logistic map:
        iterateUntilAccurate initPrec maxAttempts maxImprecision $ 
            \ prec -> logisticMapIterateNTimes r (fromRationalWithPrec prec x0) iters
        where
        initPrec = 50 -- try with this precision first
        maxAttempts = 100 -- try to increase precision 100 times before giving up
        maxImprecision = 10^^(-digits) -- target result precision
        
    -- print the result of an attempt, printing sufficiently many digits, according to the digits parameter
    reportAttempt :: Int -> (Precision, RealApprox, MI.MI) -> IO ()
    reportAttempt digits (prec, res, imprecision) =    
        putStrLn $ formatRes prec res 
        where
        formatRes prec res =
            show prec ++ ": " 
            ++ (showInternals shouldShowInternals res) 
            ++ "; prec = " ++ (show imprecision)
        shouldShowInternals = (digitsW+2, False)
        digitsW = fromIntegral digits        
       
fromRationalWithPrec :: 
    Precision -> Rational -> RealApprox
    
fromRationalWithPrec prec x =
    x |* (changeSizeLimitsOut prec 1)
    -- The operator |<*> stands for mixed-type outwards-rounded multiplication.
    -- The <> surrounding the operator * indicate outwards rounding.
    -- The | preceding the operator <*> indicate that the type of the first
    -- operand is different than the type of the second operand and the result.

logisticMapIterateNTimes ::
    (Num real, RoundedReal real)
    =>
    Rational {-^ @r@ scaling constant -} -> 
    real {-^ @x0@ initial value  -} ->
    Int {-^ @n@ number of iterations -} ->
    real {-^ @logisticMap^n(x0)@ -} 

logisticMapIterateNTimes r x0 n = 
    (iterate (logisticMap r) x0) !! n
    

logisticMap ::
    (Num real, RoundedReal real)
    =>
    Rational {-^ scaling constant r -} -> 
    real {-^ previous value x -} ->
    real {-^ rx(1-x) -} 

logisticMap r xPrev =
    r |* (xPrev * (1 - xPrev))

             