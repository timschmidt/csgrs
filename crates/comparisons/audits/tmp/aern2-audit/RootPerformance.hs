{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM_, unless, when)
import Data.IORef
import qualified Data.Map as Map
import qualified Data.Vector as Vec
import Data.Ratio ((%))
import qualified AERN2.Poly.Power.Type as P
import qualified AERN2.Poly.Power.RootsIntVector as V
import qualified AERN2.Poly.Power.RootsIntMap as A
import qualified AERN2.MP as B
import GHC.Clock (getMonotonicTimeNSec)
import System.Environment (getArgs)
import System.Exit (exitFailure,exitSuccess)

sample :: Int -> Int -> [Integer]
sample d s = [toInteger ((s*17+k*11) `mod` 19-9) | k <- [0..d-1]]++[toInteger (s+1)]
poly :: [Integer] -> P.PowPoly Integer
poly = P.fromList . zip [0..]
zero :: B.ErrorBound
zero = B.errorBound (0 :: Integer)
normV :: V.Terms -> (Integer,[Integer])
normV (_,c,bs) = (c,Vec.toList bs)
normA :: A.Terms -> (Integer,[Integer])
normA (_,c,bs) = (c,Map.elems bs)
unscale :: (Integer,[Integer]) -> [Rational]
unscale (c,bs) = map (% c) bs

choose :: Int -> Int -> Rational
choose n k = fromInteger (product [toInteger (n-k+1)..toInteger n] `div` product [1..toInteger k])
oracle :: [Integer] -> Rational -> Rational -> [Rational]
oracle cs l r = [sum [as!!j * choose i j / choose d j | j <- [0..i]] | i <- [0..d]]
  where d=length cs-1
        as=[sum [fromInteger (cs!!k)*choose k j*l^(k-j)*(r-l)^j | k <- [j..d]] | j <- [0..d]]

{-# NOINLINE initial #-}
initial :: String -> P.PowPoly Integer -> [(Integer,[Integer])]
initial method p = [if method == "map" then normA (A.initialBernsteinCoefs p zero 0 1)
                                     else normV (V.initialBernsteinCoefs p zero 0 1)]
{-# NOINLINE subdivide #-}
subdivide :: String -> (A.Terms,V.Terms) -> [(Integer,[Integer])]
subdivide method (a,v) = if method == "map"
  then let (l,r)=A.bernsteinCoefs 0 1 (1%2) a in [normA l,normA r]
  else let (l,r)=V.bernsteinCoefs 0 1 (1%2) v in [normV l,normV r]

main :: IO ()
main = do
  [mode,method,sd,sc] <- getArgs
  let d=read sd; count=read sc :: Int
      inputs=[sample d s | s <- [0..12]]
      polys=map poly inputs
      terms=[(A.initialBernsteinCoefs p zero 0 1,V.initialBernsteinCoefs p zero 0 1) | p <- polys]
  when (mode == "validate") $ do
    let valid = [map unscale (initial meth p) == [oracle cs 0 1] &&
                 map unscale (subdivide meth pair) == [oracle cs 0 (1%2),oracle cs (1%2) 1]
                 | (cs,p,pair) <- zip3 inputs polys terms, meth <- ["map","vector"]]
    print (length (filter id valid),length valid)
    unless (and valid) exitFailure
    exitSuccess
  -- Fully force all inputs before timing. Rotate through 13 independent
  -- operands, and force every output coefficient and multiplier per call.
  evaluate (force inputs)
  forM_ polys $ \p -> evaluate (force (show p))
  forM_ terms $ \(a,v) -> evaluate (force (normA a,normV v))
  state <- newIORef (cycle (zip polys terms))
  let one = do
        (p,t):rest <- readIORef state
        writeIORef state rest
        evaluate (force (if mode == "initial" then initial method p else subdivide method t))
  forM_ [1..13 :: Int] (const one)
  start <- getMonotonicTimeNSec
  forM_ [1..count] (const one)
  stop <- getMonotonicTimeNSec
  print (fromIntegral (stop-start)/fromIntegral count :: Double)
