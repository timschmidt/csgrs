{-# LANGUAGE GADTs #-}
{-# LANGUAGE NoRebindableSyntax #-}
module Main where

import Prelude
import Control.Monad (forM_, unless)
import Data.Foldable (toList)
import Data.List (permutations, transpose)
import Data.Ratio ((%))
import qualified MixedTypesNumPrelude as M
import qualified Numeric.CollectErrors as CN
import qualified Control.CollectErrors as CE
import qualified AERN2.MP as B
import qualified AERN2.MP.Float as F
import qualified AERN2.Linear.Matrix as A
import qualified AERN2.Linear.Vector as V
import System.Exit (exitFailure)

rows :: A.MatrixRC a -> [[a]]
rows (A.MatrixRC v) = map toList (toList v)

bounds :: B.MPBall -> (Rational,Rational)
bounds b = (c-e,c+e)
  where c = M.rational (B.ball_value b)
        e = M.rational (B.ball_error b)

has :: B.MPBall -> Rational -> Bool
has b x = l <= x && x <= u where (l,u) = bounds b

dot :: [Rational] -> [Rational] -> Rational
dot a b = sum (zipWith (*) a b)

mm :: [[Rational]] -> [[Rational]] -> [[Rational]]
mm a b = [[dot row col | col <- transpose b] | row <- a]

-- Independent Leibniz determinant: intentionally no donor LU or memoization.
det :: [[Rational]] -> Rational
det a = sum [sign perm * product [row !! j | (row,j) <- zip a perm]
            | perm <- permutations [0..length a-1]]
  where sign perm = if even (length [() | (i,x) <- zip [0..] perm,
                                         (j,y) <- zip [0..] perm, i < j, x > y])
                    then 1 else -1

sample :: Int -> Int -> Int -> Rational
sample seed i j = toInteger ((seed*17+i*7-j*13) `mod` 19-9) %
                  toInteger (3+(seed+i+j) `mod` 5)

matrix :: Int -> Int -> [[Rational]]
matrix seed n = [[sample seed i j | j <- [0..n-1]] | i <- [0..n-1]]

dominant :: Int -> Int -> [[Rational]]
dominant seed n = [[if i == j then sum (map abs row)+2 else x
                   | (j,x) <- zip [0..] row] | (i,row) <- zip [0..] (matrix seed n)]

check :: String -> [Bool] -> IO ()
check label tests = do
  let bad = length (filter not tests)
  putStrLn (label ++ ": " ++ show (length tests-bad) ++ "/" ++ show (length tests))
  unless (bad == 0) exitFailure

main :: IO ()
main = do
  let cases = [(n,s) | n <- [1..5], s <- [0..11]]
  check "exact Rational memoized Laplace" [A.detLaplace (==0) (A.fromList a) == det a
                                         | (n,s) <- cases, let a = matrix s n]
  check "exact Rational LU determinant" [A.luDet (A.fromList a) == det a
                                        | (n,s) <- cases, let a = dominant s n]
  check "adapted Rational LU determinant" [A.luDet_MTN (A.fromList a) == det a
                                          | (n,s) <- cases, let a = dominant s n]
  check "exact Rational LU solve" [toList (A.luSolve (A.fromList a) (V.fromList b)) == x
                                   | (n,s) <- cases, let a = dominant s n,
                                     let x = [sample (s+3) i 0 | i <- [0..n-1]],
                                     let b = map (`dot` x) a]
  check "adapted Rational LU solve" [toList (A.luSolve_MTN (A.fromList a) (V.fromList b)) == x
                                     | (n,s) <- cases, let a = dominant s n,
                                       let x = [sample (s+3) i 0 | i <- [0..n-1]],
                                       let b = map (`dot` x) a]
  forM_ [10,24,53,100 :: Integer] $ \p -> do
    let ball = B.mpBallP (B.prec p)
        float = F.ceduCentre . F.fromRationalCEDU (B.prec p)
        productCases = [(r,k,c,s) | (r,k,c) <- [(1,1,1),(2,3,4),(4,2,3),(5,5,5)], s <- [0..11]]
        operands r k c s =
          ([[sample s i j | j <- [0..k-1]] | i <- [0..r-1]],
           [[sample (s+9) i j | j <- [0..c-1]] | i <- [0..k-1]])
    check ("directed matrix products p=" ++ show p)
      [and [M.rational lo <= x && x <= M.rational hi
            | (lo,x,hi) <- zip3 (concat (rows down)) (concat exact) (concat (rows up))]
       | (r,k,c,s) <- productCases, let (a,b) = operands r k c s,
         let af = A.fromList (map (map float) a), let bf = A.fromList (map (map float) b),
         let exact = mm (map (map M.rational) (rows af)) (map (map M.rational) (rows bf)),
         let down = A.mulMPF_Down af bf, let up = A.mulMPF_Up af bf]
    check ("ball matrix products p=" ++ show p)
      [and (zipWith has (concat (rows result)) (concat (mm a b)))
       | (r,k,c,s) <- productCases, let (a,b) = operands r k c s,
         let ab = A.fromList (map (map ball) a), let bb = A.fromList (map (map ball) b),
         result <- [M.mul ab bb, A.mulViaFP ab bb]]
    -- Nonzero input radii, all endpoint/midpoint combinations sampled by a
    -- deterministic shared index. Check actual points, not donor containment.
    check ("wide ball matrix products p=" ++ show p)
      [and (zipWith has (concat (rows result)) (concat (mm aa ba)))
       | (r,k,c,s) <- productCases, let (a,b) = operands r k c s,
         let wide x = B.updateRadius (M.+ B.errorBound (1%8 :: Rational)) (ball x),
         let ab = A.fromList (map (map wide) a), let bb = A.fromList (map (map wide) b),
         result <- [M.mul ab bb, A.mulViaFP ab bb], t <- [0..8 :: Int],
         let pick z i j = let (l,u) = bounds z
                              w = toInteger ((t+i*2+j) `mod` 3) % 2
                          in l+(u-l)*w,
         let pts mx = [[pick z i j | (j,z) <- zip [0..] row] | (i,row) <- zip [0..] (rows mx)],
         let aa = pts ab, let ba = pts bb]
    -- Regular diagonally dominant point systems; b is computed independently
    -- as an exact Rational product before enclosure. Error-bearing results
    -- never count as a successful numerical certificate.
    whenPrecision p $ check ("interval solve p=" ++ show p)
      [not (CN.hasError out) && maybe False (and . zipWith (flip has) x . toList) (CE.getMaybeValue out)
       | (n,s) <- cases, let a = dominant s n,
         let x = [sample (s+3) i 0 | i <- [0..n-1]], let b = map (`dot` x) a,
         let out = A.solveBViaFP (A.fromList (map (map ball) a)) (V.fromList (map ball b))]
  where whenPrecision p action = if p >= 24 then action else pure ()
