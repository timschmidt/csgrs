{-|
    More Digits 2006 problem P01
    (http://rnc7.loria.fr/competition.html)
    
    Compute n decimal digits of c-root(a/b).
-}
module AuditP01Hmpfr where

-- the MPFR interval type:
import qualified Numeric.AERN.MPFRBasis.Interval as MI

import Data.Maybe (fromJust, isJust)

type RealApprox = MI.MI -- MPFR interval
type Precision = MI.Precision

lowPrec :: Precision
lowPrec = 50

improvePrec :: Double -> Precision -> Precision
improvePrec ratio p = ceiling $ (fromIntegral p) * ratio

main :: IO ()
main =
    do
    let params = params_hard
    putStrLn $ "Computing benchmark P01 from http://rnc7.loria.fr/competition.html:"
    putStrLn $ show params
    mapM_ printPrecResult $ benchmark params
    where
    printPrecResult (prec, _newtonIterWidths, result, resultWidth) =
        do
        putStrLn $ "prec = " ++ show (fromIntegral prec :: Int)
        putStrLn $ "  result = " ++ show result
        putStrLn $ "  |result| = " ++ show resultWidth
--        putStrLn $ "  Newton iteration intermediate result widths:"
--        mapM_ (\s -> putStrLn $ "        " ++ show s) _newtonIterWidths

{-|
    Compute n digits of the expression c-th root of a/b.
-}
data Params =
    Params 
    { 
        p_n :: Int, -- required number of digits
        p_a :: Int, -- numerator of root arg
        p_b :: Int, -- denominator of root arg
        p_c :: Int -- base of root
    }
    deriving (Show)

params_hard :: Params
params_hard = Params { p_n = 209953, p_a = 1234567, p_b = 20000, p_c = 543219876 }

--params_easy :: Params
--params_easy = Params { p_n = 25000, p_a = 345678, p_b = 1000, p_c = 1008 }
--
--params_super_easy :: Params
--params_super_easy = Params { p_n = 1000, p_a = 2, p_b = 1, p_c = 2 }

benchmark ::
    Params -> [(Precision, [RealApprox], RealApprox, RealApprox)]
benchmark (Params n a b c)
   | a == 0 = [(50 :: Precision, [], 0, 0)]
        --since the benchmark takes only rational inputs, we don't have to use different algorithms
        --depending on whether or not the argument is small.
   | otherwise = tryPrecision lowPrec
   where
   -- First compute a rough estimate
   -- that is in a region where Newton converges "fast":
   it0 = forgetErrors (fromJust (bisectUntilClose (aLowPrec/bLowPrec) c))
   aLowPrec :: RealApprox
   aLowPrec = a MI.|<*> (MI.setPrecOut lowPrec 1)
   bLowPrec :: RealApprox
   bLowPrec = b MI.|<*> (MI.setPrecOut lowPrec 1)

   -- Now apply Newton repeatedly with increasing precisions:
   tryPrecision :: Precision -> [(Precision, [RealApprox], RealApprox, RealApprox)]
   tryPrecision prec
      | goodEnough result2 = [resultTuple]
      | otherwise = resultTuple : (tryPrecision (improvePrec 2 prec))
      where
      resultTuple = (prec, widths, result2, MI.width result2)
      (result2, widths) = nthRootNewton c (aReal/bReal) (MI.setPrecOut prec it0)
      aReal :: RealApprox
      aReal =  a MI.|<*> (MI.setPrecOut prec 1)
      bReal :: RealApprox
      bReal = b MI.|<*> (MI.setPrecOut prec 1)
      goodEnough r = isJust (maybeGoodEnough r) && fromJust (maybeGoodEnough r)
      maybeGoodEnough x = (MI.width x) MI.<? 1/((MI.setPrecOut prec 10)^(n))

nthRootNewton ::
    Int {-^ n - the base of the root -} -> 
    RealApprox {-^ x -} -> 
    RealApprox {-^ itInit - initial guess -} -> 
    (RealApprox, [RealApprox])
nthRootNewton n x itInit =
  ((lowerBound bounds) MI.</\> (upperBound bounds), iters)
  where
  lowerBound (it,delta) = it + delta -- delta is always negative
  upperBound (it,_delta) = it
  (bounds, iters) = nthRootNewtonIt [] itInit
  nthRootNewtonIt :: [RealApprox] -> RealApprox -> ((RealApprox, RealApprox), [RealApprox])
  nthRootNewtonIt prevWidths it
     | not significantImprovement = ((it, delta), reverse prevWidths)
     | otherwise = nthRootNewtonIt (width : prevWidths) nextIt
     where
     significantImprovement = 
--        length prevWidths < 10
        case prevWidths of
            prevWidth : _ -> (width MI.<? prevWidth) == Just True
            _ -> True
     nextIt = forgetErrors (it + delta)
     delta = 
        1/nRA * (x/(it^(n - 1)) - it)
        where
        nRA = n MI.|<*> (MI.setPrecOut (MI.getPrec x) 1)
     width = - delta


-- TODO currently assumes that x >> 0.
-- The problem comes from an analytic error estimate in Newton's iteration, which assumes
-- that the function is twice differentiable near the root, which is not the case in zero.
-- Also, we have to do a lot of (slow) bisections if x is close to zero, since the error estimate
-- goes (linearly) to infinity as x approaches 0.
bisectUntilClose :: RealApprox -> Int -> Maybe RealApprox
bisectUntilClose xOrig n =
   keepTrying xOrig (initialUpper MI.</\> initialLower) (MI.getPrec xOrig)
   where
   initialUpper = xOrig + 1 -- we add 1 to get a guaranteed upper bound in case that x < 1
   initialLower = 0
   keepTrying :: RealApprox -> RealApprox -> Precision -> Maybe RealApprox
   keepTrying x enclosure prec
      | surelyClose = Just enclosure
      | isMidpointSmaller == Just True =
            keepTrying x upperEnclosure prec
      | isMidpointSmaller == Just False =
            keepTrying x lowerEnclosure prec
      | otherwise = -- comparison has not yielded a well-defined result
            keepTrying (MI.setPrecOut (improvePrec 1.5 prec) x) enclosure (improvePrec 1.5 prec)
      where
      surelyClose = isJust maybeClose && fromJust maybeClose
      maybeClose = (MI.width enclosure) MI.<? epsilon
         where
         epsilon = ((MI.setPrecOut prec lower) MI.</>| n) --TODO here we assume that x >> 0
         lower = MI.minOut lowerBoundOfX 1
         lowerBoundOfX = MI.fromEndpoints (lowerBoundMPFR, lowerBoundMPFR)
         (lowerBoundMPFR, _) = MI.getEndpoints x -- assuming x is consistent

            -- lower bound on the nth root. If x >= 1, then we use 1.
            -- Otherwise we use a lower bound on x, using that
            -- in this case this is also a lower bound on the root.
      isMidpointSmaller = (midpoint^n) MI.<? x
         where
         midpoint = MI.fromEndpoints (mpfrMidpoint, mpfrMidpoint)
      upperEnclosure = MI.fromEndpoints (mpfrMidpoint, endpointB)
      lowerEnclosure = MI.fromEndpoints (endpointA, mpfrMidpoint)
      mpfrMidpoint = (endpointA + endpointB)/2
      (endpointA, endpointB) = MI.getEndpoints enclosure

forgetErrors :: RealApprox -> RealApprox
forgetErrors y =
  yExact
  where
  ympfr = snd (MI.getEndpoints y)
  yExact = MI.fromEndpoints (ympfr,ympfr)
