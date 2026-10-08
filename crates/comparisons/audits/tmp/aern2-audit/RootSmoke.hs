{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import qualified AERN2.Poly.Power.Type as P
import qualified AERN2.Poly.Power.RootsIntVector as V
import qualified AERN2.Poly.Power.RootsIntMap as M
import qualified AERN2.MP as B
main :: IO ()
main = do
  let p = P.fromList [(0,-1),(2,1)]
  print (V.signVars (V.initialBernsteinCoefs p (B.errorBound (0 :: Integer)) (-2) 2))
  print (M.signVars (M.initialBernsteinCoefs p (B.errorBound (0 :: Integer)) (-2) 2))
