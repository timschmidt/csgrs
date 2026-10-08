{-# LANGUAGE BangPatterns #-}
module Main where
import Prelude
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM_, when)
import Data.IORef
import Data.Ratio ((%), numerator, denominator)
import qualified MixedTypesNumPrelude as M
import qualified Numeric.CollectErrors as CN
import qualified Control.CollectErrors as CE
import qualified AERN2.MP as B
import qualified AERN2.Real as R
import GHC.Clock (getMonotonicTimeNSec)
import System.Environment (getArgs)

input :: Int -> Rational
input i = (1048576 + toInteger (i `mod` 97)) % 3145728

{-# NOINLINE expression #-}
expression :: String -> Int -> R.CReal
expression op i = case op of
  "sqrt" -> sqrt x
  "exp" -> exp x
  "sin" -> sin x
  _ -> error "operation"
  where x = R.creal (input i)

{-# NOINLINE approximation #-}
approximation :: Integer -> R.CReal -> B.MPBall
approximation p x = case CE.getMaybeValue result of
  Just b | not (CN.hasError result) -> b
  _ -> error "unexpected numerical error"
  -- AERN2 Accuracy is rough. Two guard bits prevent its one-bit radius
  -- overstatement from weakening the benchmark's common absolute-error goal.
  where result = x R.? B.bits (p+2)

{-# NOINLINE observe #-}
observe :: Integer -> R.CReal -> IO ()
observe p x = do
  _ <- evaluate (force (approximation p x))
  pure ()

benchmark :: String -> String -> Integer -> Int -> IO ()
benchmark op mode p count = do
  shared <- newIORef (expression op 0)
  let action i = if mode == "warm" then readIORef shared >>= observe p
                 else observe p (expression op i)
      loop !i !end | i == end = pure ()
                   | otherwise = action i >> loop (i+1) end
  loop 0 (min 100 count)
  start <- getMonotonicTimeNSec
  loop 0 count
  end <- getMonotonicTimeNSec
  print (fromIntegral (end-start) / fromIntegral count :: Double)

exportChecks :: IO ()
exportChecks = forM_ [53,200] $ \p ->
  forM_ ["sqrt","exp","sin"] $ \op -> forM_ [0..96] $ \i -> do
    let b = approximation p (expression op i)
        c = M.rational (B.ball_value b)
        e = M.rational (B.ball_error b)
        x = input i
    when (e > 1 % (2^p)) (error "insufficient strict accuracy")
    putStrLn (unwords [op,show p,show (numerator x),show (denominator x),
                      show (numerator (c-e)),show (denominator (c-e)),
                      show (numerator (c+e)),show (denominator (c+e))])

main :: IO ()
main = do
  args <- getArgs
  case args of
    ["export"] -> exportChecks
    [op,mode,p,count] -> benchmark op mode (read p) (read count)
    _ -> error "args: op mode bits count, or export"
