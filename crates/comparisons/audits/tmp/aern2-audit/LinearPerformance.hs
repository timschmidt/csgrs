{-# LANGUAGE NoRebindableSyntax #-}
{-# LANGUAGE GADTs #-}
module Main where
import Prelude
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM_, when, unless)
import Data.Foldable (toList)
import Data.IORef
import Data.List (transpose, sort)
import Data.Ratio ((%))
import GHC.Clock (getMonotonicTimeNSec)
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified AERN2.Linear.Matrix as A
import System.Environment (getArgs)
import System.Exit (exitSuccess, exitFailure)

rows :: A.MatrixRC a -> [[a]]
rows (A.MatrixRC v) = map toList (toList v)

{-# NOINLINE makeInput #-}
makeInput :: Integer -> Int -> Int -> A.MatrixRC B.MPBall
makeInput p n s = A.fromList (map (map (B.mpBallP (B.prec p))) (exactInput n s))

exactInput :: Int -> Int -> [[Rational]]
exactInput n s = [[toInteger ((s*17+i*7-j*13) `mod` 19-9) % toInteger (3+(s+i+j) `mod` 5)
                  | j <- [0..n-1]] | i <- [0..n-1]]

{-# NOINLINE applyOp #-}
applyOp :: String -> A.MatrixRC B.MPBall -> A.MatrixRC B.MPBall -> [B.MPBall]
applyOp mode a b = concat (rows (if mode == "plain" then M.mul a b else A.mulViaFP a b))

main :: IO ()
main = do
  [mode,sp,sn,sc] <- getArgs
  let p = read sp; n = read sn; count = read sc :: Int
  when (mode == "validate") $ do
    let has ball x = let c = M.rational (B.ball_value ball)
                         e = M.rational (B.ball_error ball)
                     in c-e <= x && x <= c+e
        tests = [and (zipWith has actual target)
                 | s <- [0..12], which <- ["plain","via"],
                   let actual = applyOp which (makeInput p n s) (makeInput p n (s+9)),
                   let target = [sum (zipWith (*) row col) | row <- exactInput n s,
                                                            col <- transpose (exactInput n (s+9))]]
    print (length (filter id tests), length tests)
    unless (and tests) exitFailure
    exitSuccess
  when (mode == "quality") $ do
    let radii = [(M.rational (B.ball_error a), M.rational (B.ball_error b))
                 | s <- [0..12], (a,b) <- zip
                     (applyOp "plain" (makeInput p n s) (makeInput p n (s+9)))
                     (applyOp "via" (makeInput p n s) (makeInput p n (s+9)))]
        ratios = sort [fromRational (b/a) :: Double | (a,b) <- radii, a /= 0]
        zeroA = length [() | (a,b) <- radii, a == 0, b /= 0]
    print (length ratios, minimum ratios, ratios !! (length ratios `div` 2), maximum ratios, zeroA)
    exitSuccess
  -- Force input balls once, excluding construction and comparison from timed
  -- multiplication. Rotate through distinct matrix pairs, no expression cache.
  let pairs = [(makeInput p n s, makeInput p n (s+9)) | s <- [0..12]]
  forM_ pairs $ \(a,b) -> evaluate (force (concat (rows a) ++ concat (rows b))) >> pure ()
  state <- newIORef (cycle pairs)
  let one = do
        (a,b):rest <- readIORef state
        writeIORef state rest
        evaluate (force (applyOp mode a b)) >> pure ()
  forM_ [1..13 :: Int] $ \_ -> one
  start <- getMonotonicTimeNSec
  forM_ [1..count] $ \_ -> one
  stop <- getMonotonicTimeNSec
  print (fromIntegral (stop-start) / fromIntegral count :: Double)
