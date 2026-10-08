{-# LANGUAGE PolyKinds #-}
module ODE where

import FAD
import Taylor
import Data.Number.IReal -- .Rounded
import Data.Number.IReal(VarPrec(..))
import Data.Number.IReal.Auxiliary
import Data.Maybe
import Plot

{- 

A solver for the initial value problem 

 x' = f t x, t >= t0
 x t0 = x0

using a Taylor method, combined with Picard-Lindelöf's method to 
ensure a validated enclosure. 

The function solve below employs user-determined step-sizes and
number of steps to illustrate the method. Also, some tolerances 
are chosen in an ad hoc way.

-}

-- Solver ----------------------------------------------------------------------

odeDerivs :: (Num a, Num b) => (Dif a -> Dif b -> Dif b) -> a -> b -> [b]
odeDerivs f t0 x0 = fromDif x
    where xder = f (var t0) x
          x = mkDif x0 xder

--solve :: (Dif IReal -> Dif IReal -> Dif IReal) -> (IReal,IReal) -> Int -> [(IReal,Int)] ->  [(IReal,IReal)] 
solve f p _ [] = [p]
solve f p n ((h,s):ps) = rs ++ solve f (head us) n ps
   where (rs,us) = splitAt s (iterate (g h) p)
         g h (t,x) = (t+h, prec 14 (hull [step (t, lower x), step (t, upper x)]))
         hs = generalTerms h
         step (t,x) = sum (take n $ zipWith (*) (odeDerivs f t x) hs) + err
          where ti = t -+- (t+h)
                xi = bound 0.1 Nothing 
                err = odeDerivs f ti xi !! n * hs !! n

                bound rad maybeAns
                 | rad > 100 = error "Cannot verify existence"
                 | i1 `containedIn` i0 `atDecimals` (12+2) = bound (rad/2) (Just i0)
                 | otherwise = maybe (bound (2*rad) Nothing) id maybeAns
                  where i0 = (x - rad) -+- (x + rad)
                        i1 = x + h * unDif (f (var ti)) i0

{- 
Examples from Tucker:

6.4.2 solve (\t x -> -t*x) (0,0+-1) 3 [(0.1,60),(0.05,100)]
6.4.3 solve (\t x -> x^2) (0,1 -+- 1.25) 3 [(0.05,8),(0.01,30),(0.001,50)]
6.4.4 solve (\t x -> -x^3) (0,1) 4 [(0.1,10),(0.2,15),(0.8,10)]
6.4.5 solve (\t x -> x*(x-1)) (0,1) 7 [(0.1,100)]
6.4.6 solve (\t x -> 5+sin t - x) (1,5+-1) 6 [(0.3,30)]
6.4.7 solve (\t x -> (exp (exp (-t*x)) + 0.01*x^3 + 0.1*x + 2 + 10*cos x+4*sin t - log x)/(0.02*x^3+4*x^2+3*x+4+(x+1)**0.75*0.001*sin (1.5*t*x)+0.001*cos(3.14*t))) (0,3 -+- (3+recip(2^52))) 3 [(0.25,40)]
-}


{-
If you have gnuplot, you may consider uncommenting this function and the import of module Plot, 
to visualize the solutions. This gnuplot connection is, however, only a hack which should be 
redone properly.
-}


psolve f (t0,x0) n hns = plotData [fu, fl] (sh t0) (sh xs)
    where ss = solve f (t0,x0) n hns
          sh x = showIReal 10 x 
          xs = t0 + sum (map (\(h,n) -> h * fromIntegral n) hns)
          fu = map (\(t,x) -> sh (mid t) ++ " " ++ sh (upper x)) ss
          fl = map (\(t,x) -> sh (mid t) ++ " " ++ sh (lower x)) ss

