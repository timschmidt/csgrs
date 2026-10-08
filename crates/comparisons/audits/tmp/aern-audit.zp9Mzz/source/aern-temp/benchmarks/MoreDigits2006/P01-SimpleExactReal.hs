module AuditP01Simple where

-- the MPFR interval type:
import qualified Numeric.AERN.MPFRBasis.Interval as MI

-- real arithmetic operators and imprecision measure:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators ((|<*>))
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (RoundedReal)
import Numeric.AERN.RealArithmetic.Measures (imprecisionOf, iterateUntilAccurate)

-- ability to change precision of numbers:
import Numeric.AERN.Basics.SizeLimits (changeSizeLimitsOut)

-- generic tools for controlling formatting of variable precision approximations:
import Numeric.AERN.Basics.ShowInternals (showInternals)


import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))
import System.Environment (getArgs)

import Data.Ratio

type RealApprox = MI.MI -- MPFR interval
type Precision = MI.Precision

data Limit a = 
   Limit
   {
      sequence :: Int -> a
   }

main =
    mapM_ print $ approximationWithErrorBound (realRootFinder xHard cHard) nHard
   
approximation :: Limit a -> Int -> a
approximation (Limit seq) n = seq n   
   
type ExactReal = Limit RealApprox

instance Functor Limit where
   fmap f (Limit seq) = Limit (\k -> f (seq k))

approximationWithErrorBound :: ExactReal -> Int -> [(Int, RealApprox)]
approximationWithErrorBound x decimalDigits =
   tryPrecision 0
   where 
   tryPrecision :: Int -> [(Int, RealApprox)]
   tryPrecision n =
      if surelyGoodEnough (approx) then
         [(n,approx)]
      else  
         (n,approx) : (tryPrecision (n + 1))
      where
      approx = (approximation x n)
      maybeGoodEnough :: RealApprox -> (Maybe Bool)
      maybeGoodEnough x = MI.width x MI.<? epsilon
      surelyGoodEnough :: RealApprox -> Bool
      surelyGoodEnough x = goodResult (maybeGoodEnough x) && (fromJust (maybeGoodEnough x))
      epsilon :: RealApprox
      epsilon = 1/((changeSizeLimitsOut (MI.getPrec approx) 10)^(decimalDigits))
      goodResult (Just _) = True
      goodResult Nothing = False
      fromJust (Just something) = something

realFromRational :: Rational -> ExactReal
realFromRational x =
   Limit sequence
   where
   sequence :: Int -> RealApprox
   sequence n = x |<*> (changeSizeLimitsOut (prec n) 1)
   prec :: Int -> Precision
   prec n = if n == 0 then 50 else 2*prec (n - 1)

--- some simple examples of exact reals in use

realSqrt :: ExactReal -> ExactReal
realSqrt = fmap MI.sqrtOut

sqrt2 :: ExactReal
sqrt2 = realSqrt (realFromRational 2)

square :: RealApprox -> RealApprox
square x = (x*x)

two :: ExactReal
two = (fmap square) sqrt2

-- the benchmark P01 with exact reals

iterationsFromDigits :: Int -> Int
iterationsFromDigits n = ceiling ((logBase 2 (logBase 2 10)) + (logBase 2 (fromIntegral n)))


xEasy :: ExactReal
xEasy = realFromRational(2500%345678)
nEasy :: Int
nEasy = 25000 
cEasy :: Int
cEasy = 1008 --note that "c" is the "n" in "nth" root...

xHard :: ExactReal
xHard = realFromRational (1234567%20000)
cHard :: Int
cHard = 543219876
nHard :: Int
nHard = 209953

xSuperEasy :: ExactReal
xSuperEasy = realFromRational 2
cSuperEasy :: Int
cSuperEasy = 2
nSuperEasy :: Int
nSuperEasy = 1000

benchmark = do
            putStrLn "easy example:"
            let approximateRoot = snd $ last (approximationWithErrorBound (realRootFinder xEasy cEasy) nEasy) --approximation up to n decimal digit of the cth root of x
            putStrLn $ "enclosure: " ++ show approximateRoot
            putStrLn $ "width: " ++ show (MI.width approximateRoot)
            putStrLn "------------------"
            putStrLn "hard example:"
            let approximateRoot = snd $ last (approximationWithErrorBound (realRootFinder xHard cHard) nHard) --approximation up to n decimal digit of the cth root of x
            putStrLn $ "enclosure: " ++ show approximateRoot
            putStrLn $ "width: " ++ show (MI.width approximateRoot)
            putStrLn "------------------"
            

realRootFinder :: ExactReal -> Int -> ExactReal
realRootFinder x n =
   if surelyGreaterZero then newtonRootFinder x n
   else bisectionRootFinder x n
   where
   surelyGreaterZero = goodResult maybeGreaterZero && fromJust maybeGreaterZero
   maybeGreaterZero = (approximation x 5) MI.>? 0
   goodResult (Just _) = True
   goodResult Nothing = False
   fromJust (Just something) = something

bisectionRootFinder :: ExactReal -> Int -> ExactReal --TODO
bisectionRootFinder x n =
   Limit (\k -> keepTrying x k n (initialUpper MI.</\> initialLower))
   where
   goodResult (Just _) = True
   goodResult Nothing = False
   fromJust (Just something) = something
   initialUpper = (approximation x 0) + 1 -- we add 1 to get a guaranteed upper bound in case that x < 1
   initialLower = 0
   keepTrying :: ExactReal -> Int -> Int -> RealApprox -> RealApprox
   keepTrying x accuracy n enclosure =
      if goodResult isGuessSmaller then -- comparison yielded well-defined result?
         if fromJust isGuessSmaller then keepTrying x accuracy n upperEnclosure
         else keepTrying x accuracy n lowerEnclosure
      else                              -- if not, give up for now.
         enclosure --TODO we could alternatively try with higher precision.
                   -- generally, it is probably not too great to let the user
                   -- decide which element precisely is the nth approximation 
      where
      approx = approximation x accuracy
      isGuessSmaller = guess MI.<? approx
      (endpointA, endpointB) = MI.getEndpoints enclosure
      mpfrMidpoint = (endpointA + endpointB)/2
      midpoint = MI.fromEndpoints (mpfrMidpoint, mpfrMidpoint)
      guess = midpoint^n
      upperEnclosure = MI.fromEndpoints (mpfrMidpoint, endpointB)
      lowerEnclosure = MI.fromEndpoints (endpointA, mpfrMidpoint)

newtonRootFinder :: ExactReal -> Int -> ExactReal
newtonRootFinder x n =
   Limit sequence
   where
   sequence k = newton (approximation x k) x0 n
   x0 = bisectUntilCloseReal x n

newton :: RealApprox -> RealApprox -> Int -> RealApprox
newton x x0 n =
   (lowerBound bounds) MI.</\> (upperBound bounds)
   where
   bounds = nthRootNewtonIt x n 22 x0 (makedelta x0) -- TODO quick and dirty hack: do 22 iterations. This suffices for every problem in practice.
                                                     -- this is easy to improve though: stop iterating when delta is sufficiently small.
   nReal :: RealApprox
   nReal = n |<*> (changeSizeLimitsOut (MI.getPrec x) 1)
   makedelta y = 1/nReal * (x/(y^(n - 1)) - y)
   lowerBound (it,delta) = it + delta
   upperBound (it, _) = it
   forgetErrors :: RealApprox -> RealApprox
   forgetErrors y =
      changeSizeLimitsOut (MI.getPrec x) yExact
      where
      ympfr = snd (MI.getEndpoints y)
      yExact = MI.fromEndpoints (ympfr,ympfr)
   nthRootNewtonIt :: RealApprox -> Int -> Int -> RealApprox -> RealApprox -> (RealApprox, RealApprox)
   nthRootNewtonIt x n k it delta =
      if k == 0 then (it,delta)
      else nthRootNewtonIt x n (k - 1) nextIt (makedelta nextIt) 
      where
         nextIt = forgetErrors (it + delta)
         
   
bisectUntilCloseReal :: ExactReal -> Int -> RealApprox
bisectUntilCloseReal x n =
   keepTrying x 0 n (initialUpper MI.</\> initialLower)
   where
   goodResult (Just _) = True
   goodResult Nothing = False
   fromJust (Just something) = something
   initialUpper = (approximation x 0) + 1 -- we add 1 to get a guaranteed upper bound in case that x < 1
   initialLower = 0
   nontrivialLowerBound :: ExactReal -> Int -> RealApprox -- find lower bound on x (not on the root!) greater than 0.
   nontrivialLowerBound x accuracy = 
      if goodResult greaterZero then
         if fromJust greaterZero then
            lowerBound
         else 
            nontrivialLowerBound  x (accuracy + 1)
      else -- try with higher precision!
         nontrivialLowerBound  x (accuracy + 1)
      where
      approx = approximation x accuracy
      greaterZero = approx MI.>? 0
      (ampfr, bmpfr) = MI.getEndpoints approx
      (a, b) = (MI.fromEndpoints (ampfr,ampfr), MI.fromEndpoints (bmpfr,bmpfr))
      lowerBound = MI.minOut a b
   keepTrying :: ExactReal -> Int -> Int -> RealApprox ->  RealApprox
   keepTrying x accuracy n enclosure =
      if surelyClose then enclosure
      else
        if goodResult isGuessSmaller then -- comparison yielded well-defined result?
           if fromJust isGuessSmaller then keepTrying x accuracy n upperEnclosure
           else keepTrying x accuracy n lowerEnclosure
        else                              -- if not, we need higher precision!
           keepTrying x (accuracy + 1) n enclosure
      where
      approx = approximation x accuracy
      epsilon = ((changeSizeLimitsOut (MI.getPrec approx) lower) MI.</>| n)
      lower = MI.minOut (nontrivialLowerBound x 0) 1 -- lower bound on the nth root. If x >= 1, then we use 1. Otherwise we use a lower bound on x, using that in this case this is also a lower bound on the root.
      maybeClose = (MI.width enclosure) MI.<? epsilon
      surelyClose = goodResult maybeClose && fromJust maybeClose
      isGuessSmaller = guess MI.<? approx
      (endpointA, endpointB) = MI.getEndpoints enclosure
      mpfrMidpoint = (endpointA + endpointB)/2
      midpoint = MI.fromEndpoints (mpfrMidpoint, mpfrMidpoint)
      guess = midpoint^n
      upperEnclosure = MI.fromEndpoints (mpfrMidpoint, endpointB)
      lowerEnclosure = MI.fromEndpoints (endpointA, mpfrMidpoint)
